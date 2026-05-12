/*
 * JLR G1 CAN Shifter for M5Stack Dial
 *
 * Sends 0x312 to the Zombieverter VCU to select Park/Reverse/Neutral/Drive.
 *
 * Protocol (from JLR_G1.cpp JLR_G1::DecodeCAN):
 *   The VCU reads 0x312 from the physical shifter knob.
 *   It reads ONLY two things:
 *     byte[3] >> 4  = gear  (0=Park, 1=Reverse, 2=Neutral, 3=Drive)
 *     byte[7] & 0x0F = rolling counter (stored but never validated)
 *   All other bytes are ignored.
 *
 *   The VCU generates 0x3F3 itself as output to other car modules.
 *   We do NOT send 0x3F3 — that's the VCU's job.
 *
 * Hardware:
 *   M5Stack Dial (ESP32-S3)
 *   Grove Port: GPIO2=CAN_TX, GPIO1=CAN_RX  → IS3050G transceiver
 *   EC11 encoder: A=GPIO41, B=GPIO40, button=GPIO42
 */

#include <M5Unified.h>
#include <ESP32Encoder.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ── CAN Pins ─────────────────────────────────────────────────────────────────
#define CAN_TX_PIN  GPIO_NUM_2
#define CAN_RX_PIN  GPIO_NUM_1

// ── Encoder / Button ─────────────────────────────────────────────────────────
#define ENC_A_PIN   41
#define ENC_B_PIN   40
#define BTN_PIN     42

// ── Gear IDs ──────────────────────────────────────────────────────────────────
#define JLR_PARK    0
#define JLR_REVERSE 1
#define JLR_NEUTRAL 2
#define JLR_DRIVE   3

#define CAN_TX_ID    0x312   // Physical shifter → VCU
#define ACK_DELAY_MS 200

// ── Colours ───────────────────────────────────────────────────────────────────
#define COL_BG      0x0000
#define COL_PENDING 0x07FF   // Cyan
#define COL_ACKED   0xF800   // Red
#define COL_DIM     0x8410   // Grey

const char* gearLabel[4] = { "P", "R", "N", "D" };

// ── State ─────────────────────────────────────────────────────────────────────
volatile int8_t   dialPos   = 0;
volatile int8_t   sentGear  = -1;
volatile bool     gearAcked = false;
volatile uint32_t sentTime  = 0;
volatile uint32_t txCount   = 0;
volatile uint32_t txFail    = 0;

uint8_t cnt312 = 0;   // free-running, only touched by canTask

ESP32Encoder encoder;
M5Canvas      canvas(&M5.Display);

void initCAN();
void drawScreen();
void canTask(void* pvParameters);

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("JLR CAN Shifter  TX:0x312 → Zombieverter");

    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(0);
    M5.Display.setBrightness(200);
    canvas.createSprite(240, 240);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachSingleEdge(ENC_A_PIN, ENC_B_PIN);
    encoder.setCount(0);
    pinMode(BTN_PIN, INPUT_PULLUP);

    initCAN();
    xTaskCreatePinnedToCore(canTask, "CAN", 4096, NULL, 5, NULL, 0);

    drawScreen();
    Serial.println("Ready. Rotate=select, Press=send.");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    M5.update();

    // ── Encoder ───────────────────────────────────────────────────────────────
    static int32_t lastEnc = 0;
    int32_t enc = (int32_t)encoder.getCount();
    if (enc != lastEnc) {
        int32_t delta = enc - lastEnc;
        lastEnc = enc;
        int8_t pos = dialPos + (delta > 0 ? 1 : -1);
        if (pos < 0) pos = 3;
        if (pos > 3) pos = 0;
        dialPos = pos;
        drawScreen();
    }

    // ── Button ────────────────────────────────────────────────────────────────
    static bool lastBtn = HIGH;
    bool curBtn = (bool)digitalRead(BTN_PIN);
    if (curBtn == LOW && lastBtn == HIGH) {
        delay(20);
        if (digitalRead(BTN_PIN) == LOW) {
            sentGear  = dialPos;
            gearAcked = false;
            sentTime  = millis();
            drawScreen();
            Serial.printf(">> Gear: %s\n", gearLabel[(int)sentGear]);
        }
    }
    lastBtn = curBtn;

    // ── Optimistic ack ────────────────────────────────────────────────────────
    if (sentGear >= 0 && !gearAcked &&
        (millis() - sentTime >= ACK_DELAY_MS)) {
        gearAcked = true;
        drawScreen();
    }

    delay(10);
}

// ─────────────────────────────────────────────────────────────────────────────
// CAN task on Core 0 — TX every 20ms, drain RX continuously
void canTask(void* pvParameters) {
    TickType_t lastTx = xTaskGetTickCount();

    for (;;) {
        // ── Drain RX to prevent FIFO overflow on busy bus ─────────────────────
        twai_message_t msg;
        while (twai_receive(&msg, 0) == ESP_OK) {
            (void)msg;  // discard — use SavvyCAN for bus monitoring
        }

        // ── Bus health ────────────────────────────────────────────────────────
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK) {
            if (st.state == TWAI_STATE_BUS_OFF) {
                Serial.println("CAN bus-off — recovering");
                twai_initiate_recovery();
            } else if (st.state == TWAI_STATE_STOPPED) {
                twai_start();
            }
            txFail = st.tx_failed_count;

            static uint32_t lastLog = 0;
            if (millis() - lastLog > 5000) {
                lastLog = millis();
                Serial.printf("CAN state=%d txOK=%lu txFail=%lu rxErr=%lu\n",
                    st.state, (unsigned long)txCount,
                    st.tx_failed_count, st.rx_error_counter);
            }
        }

        // ── TX every 20ms ─────────────────────────────────────────────────────
        if (xTaskGetTickCount() - lastTx >= pdMS_TO_TICKS(20)) {
            lastTx = xTaskGetTickCount();

            int8_t gear = sentGear;
            if (gear < 0) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            // Minimal 0x312 frame — VCU only reads byte[3] and byte[7]
            uint8_t mes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            mes[3] = ((uint8_t)gear << 4) & 0xF0;  // gear in upper nibble
            mes[7] = cnt312 & 0x0F;                 // counter in lower nibble
            cnt312 = (cnt312 + 1) & 0x0F;

            twai_message_t txMsg;
            txMsg.identifier       = CAN_TX_ID;
            txMsg.extd             = 0;
            txMsg.rtr              = 0;
            txMsg.data_length_code = 8;
            memcpy(txMsg.data, mes, 8);

            esp_err_t r = twai_transmit(&txMsg, 0);
            if (r == ESP_OK) txCount++;

            // Log on gear change only
            static int8_t lastLogGear = -1;
            if (gear != lastLogGear) {
                lastLogGear = gear;
                Serial.printf("TX 0x312 [%s]: 00 00 00 %02X 00 00 00 %02X\n",
                    gearLabel[gear], mes[3], mes[7]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void initCAN() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("ERROR: CAN driver install failed"); return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("ERROR: CAN start failed"); return;
    }
    Serial.println("CAN OK  500kbps  NO_ACK  TX=GPIO2 RX=GPIO1");
}

// ─────────────────────────────────────────────────────────────────────────────
void drawScreen() {
    const int CX = 120, CY = 120;
    canvas.fillSprite(COL_BG);

    canvas.drawCircle(CX, CY, 116, 0x4208);
    canvas.drawCircle(CX, CY, 115, 0x8410);
    canvas.drawCircle(CX, CY, 114, 0x4208);

    const int gx[4] = { CX,     CX+72,  CX,     CX-72 };
    const int gy[4] = { CY-72,  CY,     CY+72,  CY    };

    int8_t sg = sentGear;
    bool   ak = gearAcked;
    int8_t dp = dialPos;

    for (int i = 0; i < 4; i++) {
        uint32_t col;
        uint8_t  sz;
        if      (i==sg && ak)        { col=COL_ACKED;   sz=7; }
        else if (i==sg || i==dp)     { col=COL_PENDING; sz=7; }
        else                         { col=COL_DIM;     sz=4; }

        canvas.setTextColor(col, COL_BG);
        canvas.setTextSize(sz);
        canvas.setTextDatum(CC_DATUM);
        canvas.drawString(gearLabel[i], gx[i], gy[i]);
    }

    canvas.setTextSize(1);
    canvas.setTextDatum(BC_DATUM);
    if (sg < 0) {
        canvas.setTextColor(0x8410, COL_BG);
        canvas.drawString("Turn to select  |  Press to send", CX, 228);
    } else if (ak) {
        char buf[32];
        snprintf(buf, sizeof(buf), "TX:%lu  FAIL:%lu",
            (unsigned long)txCount, (unsigned long)txFail);
        canvas.setTextColor(COL_ACKED, COL_BG);
        canvas.drawString("CONFIRMED", CX, 220);
        canvas.setTextColor(0x8410, COL_BG);
        canvas.drawString(buf, CX, 234);
    } else {
        canvas.setTextColor(COL_PENDING, COL_BG);
        canvas.drawString("Sending...", CX, 228);
    }

    canvas.fillCircle(CX, CY, 5, 0x8410);
    canvas.pushSprite(0, 0);
}

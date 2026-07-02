/*
 * JLR G1 CAN Shifter for M5Stack Dial
 *
 * Three full-screen arc dial pages, swiped left/right:
 *   Page 0 (PRND):     CAN 0x312, labels P/R/N/D
 *   Page 1 (H/L/AUTO): CAN 0x300, labels L/H/AUTO
 *   Page 2 (MG):       CAN 0x301, labels 1/2/1+2/BLD
 *
 * On each page: letters fixed on arc, wedge moves to selected letter.
 * Encoder scrolls selection, button confirms/sends.
 * Swipe left/right on touch to change page.
 * Page 0 shows small status badges for pages 1 and 2.
 * Three navigation dots at bottom show current page.
 *
 * Idle: logo screen after IDLE_TIMEOUT_S seconds.
 * IMU: logo counter-rotates to stay gravity-upright (MPU6886 on red Grove port).
 * Power: shuts down after SHUTDOWN_TIMEOUT_S seconds with no 12V input.
 *
 * Hardware:
 *   M5Stack Dial (ESP32-S3)
 *   Grove black: GPIO2=CAN_TX, GPIO1=CAN_RX -> IS3050G
 *   Grove red:   GPIO13=SDA, GPIO15=SCL -> MPU6886
 *   EC11:  A=GPIO41, B=GPIO40, btn=GPIO42
 */

#include <M5Unified.h>
#include <ESP32Encoder.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "logo_lines.h"

// -- CAN -----------------------------------------------------------------------
#define CAN_TX_PIN      GPIO_NUM_2
#define CAN_RX_PIN      GPIO_NUM_1
#define CAN_ID_GEAR     0x312
#define CAN_ID_HL       0x300
#define CAN_ID_MG       0x301

// -- SDO (CANopen) constants ---------------------------------------------------
// Used to poll confirmed state from Zombieverter
#define SDO_TX_ID       0x603   // ESP32 -> VCU
#define SDO_RX_ID       0x583   // VCU -> ESP32
#define SDO_BASE_INDEX  0x2100
#define SDO_CMD_READ    0x40
#define SDO_RESP_4B     0x43
#define SDO_RESP_2B     0x4B
#define SDO_RESP_ABORT  0x80
// Zombieverter param IDs (from param_prj.h)
#define PARAM_GEAR      27    // Gear: 0=L 1=H 2=AUTO 3=H/L  (page 1)
#define PARAM_MOTACTIVE 129   // MotActive: 0=MG1 1=MG2 2=Both 3=Blend (page 2)
// Values are fixed-point x32: raw/32 = integer value for these integer params
#define SDO_POLL_MS     500   // how often to poll each param

// -- Encoder / Button ----------------------------------------------------------
#define ENC_A_PIN       41
#define ENC_B_PIN       40
#define BTN_PIN         42

// -- Build-time options --------------------------------------------------------
#define IDLE_TIMEOUT_S      10    // seconds before logo screen (0=disabled)
#define ENABLE_IMU          1     // 1=logo rotates with IMU, 0=fixed
#define SHUTDOWN_TIMEOUT_S  120   // seconds on battery before poweroff (0=disabled)

// -- Timing --------------------------------------------------------------------
#define ACK_DELAY_MS        200
#define IDLE_TIMEOUT_MS     (IDLE_TIMEOUT_S * 1000)
#define ANIM_SPEED          0.18f

// -- Display geometry ----------------------------------------------------------
#define CX              120
#define CY              120
#define R               116
#define ARC_R           86
#define ARC_START_DEG   (-135)
#define ARC_END_DEG     (-45)
#define ARC_WEDGE_DEG   18.0f

// -- Logo ----------------------------------------------------------------------
#define LOGO_CENTER_X   120
#define LOGO_CENTER_Y   120
#define LOGO_RADIUS     108

// -- IMU physics ---------------------------------------------------------------
#define IMU_SPRING      0.12f
#define IMU_DAMPING     0.72f
#define IMU_POLL_MS     16

// -- Colours (RGB565) ----------------------------------------------------------
#define COL_BG          0x0000
#define COL_PENDING     0x07FF   // Cyan  - highlighted/pending
#define COL_ACKED       0xF800   // Red   - confirmed
#define COL_WEDGE_BG    0x2104   // Top half background
#define COL_DIVIDER     0x2104
#define COL_LOGO_BG     0x0419
#define COL_LOGO_FG     0xC618
#define COL_DOT_ACT     0xFFFF   // Nav dot active
#define COL_DOT_DIM     0x4208   // Nav dot inactive
#define COL_P1_BADGE    0x07FF   // H/L badge colour on page 0
#define COL_P2_BADGE    0xFD20   // MG badge colour on page 0

// -- Page definitions ----------------------------------------------------------
// Each page has: labels, count, CAN ID, confirmed index, wedge angle state
#define NUM_PAGES       3

const char* pageLabels[NUM_PAGES][4] = {
    { "P", "R", "N", "D"   },   // page 0: PRND
    { "L", "H", "AUTO", "H/L" }, // page 1: H/L/AUTO/HiFwdLoRev
    { "1", "2", "1+2", "BLD" }, // page 2: MG
};
const int pageCount[NUM_PAGES] = { 4, 4, 4 };
const uint16_t pageCanId[NUM_PAGES] = { CAN_ID_GEAR, CAN_ID_HL, CAN_ID_MG };

// -- State ---------------------------------------------------------------------
volatile int8_t   page          = 0;      // current page (0-2)
volatile int8_t   dialPos[NUM_PAGES]  = { 0, 1, 3 };  // P=0, H=1, BLD=3 defaults
volatile int8_t   confirmed[NUM_PAGES]= {-1,-1,-1 };   // confirmed selection per page
volatile bool     acked[NUM_PAGES]    = {false,false,false};
volatile uint32_t sentTime[NUM_PAGES] = {0,0,0};

// Wedge animation per page (wedge moves to selected letter)
float wedgeAngle[NUM_PAGES]  = {0,0,0};   // current rendered wedge angle
float wedgeTarget[NUM_PAGES] = {0,0,0};   // target wedge angle

// CAN
uint8_t           cnt312       = 0;
volatile uint32_t txCount      = 0;
volatile uint32_t txFail       = 0;

// Screen
enum Screen { SCREEN_GEAR, SCREEN_LOGO };
volatile Screen   currentScreen  = SCREEN_GEAR;
volatile uint32_t lastActivityMs = 0;
uint32_t          powerLostMs    = 0;

// Logo / IMU
float logoAngle  = 0.0f;
float logoVel    = 0.0f;
bool  imuPresent = false;

ESP32Encoder encoder;
M5Canvas      canvas(&M5.Display);

// -- Forward declarations ------------------------------------------------------
void initCAN();
void drawPage();
void drawLogoScreen(float angle);
void canTask(void* p);
void animTask(void* p);
void imuTask(void* p);
void bumpActivity();

// -- Angle helpers -------------------------------------------------------------
static float indexToAngle(int idx, int count) {
    float startR = ARC_START_DEG * M_PI / 180.0f;
    float endR   = ARC_END_DEG   * M_PI / 180.0f;
    if (count <= 1) return (startR + endR) / 2.0f;
    return startR + (endR - startR) * idx / (float)(count - 1);
}

// -- Setup ---------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("JLR CAN Shifter - 3 page arc dial");

    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(0);
    M5.Display.setBrightness(200);
    canvas.createSprite(240, 240);

#if ENABLE_IMU
    M5.Ex_I2C.begin();
    if (M5.Imu.begin(&M5.Ex_I2C) != m5::imu_none) {
        imuPresent = true;
        Serial.println("IMU detected on Ex_I2C");
    } else {
        Serial.println("IMU not found on Ex_I2C - check wiring");
    }
#else
    Serial.println("IMU disabled at build time");
#endif

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachSingleEdge(ENC_A_PIN, ENC_B_PIN);
    encoder.setCount(0);
    pinMode(BTN_PIN, INPUT_PULLUP);

    // Initialise wedge angles to default selections
    for (int p = 0; p < NUM_PAGES; p++) {
        wedgeAngle[p] = wedgeTarget[p] =
            indexToAngle(dialPos[p], pageCount[p]);
    }

    initCAN();

    xTaskCreatePinnedToCore(canTask,  "CAN",  4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(imuTask,  "IMU",  4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(animTask, "ANIM", 4096, NULL, 4, NULL, 1);

    lastActivityMs = millis();
    drawPage();
    Serial.println("Ready.");
}

// -- Loop ----------------------------------------------------------------------
void loop() {
    M5.update();
    uint32_t now = millis();

    // -- Touch: swipe left/right to change page, wake from logo ---------------
    {
        static bool    touching  = false;
        static int16_t startTX   = 0, startTY = 0;
        static int16_t lastTX    = 0, lastTY  = 0;
        static bool    didDrag   = false;

        int tc = M5.Touch.getCount();
        if (tc > 0) {
            auto t0 = M5.Touch.getDetail(0);
            int16_t tx = t0.x, ty = t0.y;

            if (!touching) {
                touching = true;
                startTX = lastTX = tx;
                startTY = lastTY = ty;
                didDrag = false;
            } else {
                int16_t dx = tx - lastTX, dy = ty - lastTY;
                // Logo spin
                if (currentScreen == SCREEN_LOGO && (abs(dx) > 1 || abs(dy) > 1)) {
                    float rx = (float)(lastTX - LOGO_CENTER_X);
                    float ry = (float)(lastTY - LOGO_CENTER_Y);
                    float r2 = rx*rx + ry*ry;
                    if (r2 < 100.0f) r2 = 100.0f;
                    logoVel += (rx*(float)dy - ry*(float)dx) / r2 * 0.15f;
                    if (logoVel >  0.3f) logoVel =  0.3f;
                    if (logoVel < -0.3f) logoVel = -0.3f;
                    didDrag = true;
                    bumpActivity();
                }
                lastTX = tx; lastTY = ty;
            }
        } else if (touching) {
            touching = false;
            int16_t totalDX = lastTX - startTX;
            int16_t totalDY = lastTY - startTY;

            if (currentScreen == SCREEN_LOGO) {
                // Any touch wakes to gear screen
                currentScreen = SCREEN_GEAR;
                bumpActivity();
            } else if (abs(totalDX) > 40 && abs(totalDX) > abs(totalDY)*2) {
                // Horizontal swipe - change page
                if (totalDX < 0) {
                    // Swipe left - next page
                    if (page == 0)      page = 1;
                    else if (page == 1) page = 2;
                    // page 2 swipe left does nothing
                } else {
                    // Swipe right - previous page
                    if (page == 2)      page = 1;
                    else if (page == 1) page = 0;
                    // page 0 swipe right does nothing
                }
                bumpActivity();
            }
            // Taps (no drag) do nothing on gear screen
        }
    }

    // -- Idle timeout -> logo --------------------------------------------------
#if IDLE_TIMEOUT_S > 0
    if (currentScreen == SCREEN_GEAR &&
        (now - lastActivityMs) >= IDLE_TIMEOUT_MS) {
        currentScreen = SCREEN_LOGO;
    }
#endif

    // -- Encoder ---------------------------------------------------------------
    static int32_t lastEnc = 0;
    int32_t enc = (int32_t)encoder.getCount();
    if (enc != lastEnc) {
        int32_t delta = enc - lastEnc;
        lastEnc = enc;
        bumpActivity();

#if IDLE_TIMEOUT_S > 0
        if (currentScreen == SCREEN_LOGO) {
            currentScreen = SCREEN_GEAR;
        } else {
#else
        {
#endif
            int8_t pg = page;
            int8_t pos = dialPos[pg] + (delta > 0 ? 1 : -1);
            if (pos < 0) pos = 0;
            if (pos >= pageCount[pg]) pos = pageCount[pg] - 1;
            dialPos[pg]    = pos;
            wedgeTarget[pg] = indexToAngle(pos, pageCount[pg]);
        }
    }

    // -- Button ----------------------------------------------------------------
    static bool lastBtn = HIGH;
    bool curBtn = (bool)digitalRead(BTN_PIN);
    if (curBtn == LOW && lastBtn == HIGH) {
        delay(20);
        if (digitalRead(BTN_PIN) == LOW) {
            bumpActivity();
#if IDLE_TIMEOUT_S > 0
            if (currentScreen == SCREEN_LOGO) {
                currentScreen = SCREEN_GEAR;
            } else {
#else
            {
#endif
                int8_t pg = page;
                confirmed[pg] = dialPos[pg];
                acked[pg]     = false;
                sentTime[pg]  = millis();
                Serial.printf(">> Page %d: %s\n", pg,
                    pageLabels[pg][dialPos[pg]]);
            }
        }
    }
    lastBtn = curBtn;

    // -- Optimistic ack for page 0 (PRND) only --------------------------------
    // Pages 1 and 2 use SDO polling for real confirmation from the VCU.
    if (confirmed[0] >= 0 && !acked[0] && (now - sentTime[0]) >= ACK_DELAY_MS) {
        acked[0] = true;
    }

    // -- 12V loss shutdown -----------------------------------------------------
#if SHUTDOWN_TIMEOUT_S > 0
    {
        static uint32_t lastPowerCheck = 0;
        if (millis() - lastPowerCheck > 2000) {
            lastPowerCheck = millis();
            if (M5.Power.isCharging()) {
                if (powerLostMs != 0) Serial.println("12V restored");
                powerLostMs = 0;
            } else {
                if (powerLostMs == 0) {
                    powerLostMs = millis();
                    Serial.println("12V lost - shutdown timer started");
                }
                uint32_t secsGone = (millis() - powerLostMs) / 1000;
                if (secsGone >= SHUTDOWN_TIMEOUT_S) {
                    Serial.printf("Shutting down after %us on battery\n", secsGone);
                    delay(100);
                    M5.Power.powerOff();
                }
            }
        }
    }
#endif

    delay(10);
}

// -- Animation task ------------------------------------------------------------
void animTask(void* p) {
    for (;;) {
        if (currentScreen == SCREEN_GEAR) {
            // Animate wedge toward target on all pages
            for (int pg = 0; pg < NUM_PAGES; pg++) {
                float diff = wedgeTarget[pg] - wedgeAngle[pg];
                if (fabsf(diff) > 0.001f) {
                    wedgeAngle[pg] += diff * ANIM_SPEED;
                }
            }
            drawPage();
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// -- IMU task ------------------------------------------------------------------
void imuTask(void* p) {
    for (;;) {
        float targetAngle = 0.0f;
#if ENABLE_IMU
        if (imuPresent) {
            float ax, ay, az;
            M5.Imu.getAccel(&ax, &ay, &az);
            targetAngle = atan2f(ax, ay);
        }
#endif
        float diff = targetAngle - logoAngle;
        while (diff >  M_PI) diff -= 2.0f * M_PI;
        while (diff < -M_PI) diff += 2.0f * M_PI;
        logoVel   = logoVel * IMU_DAMPING + diff * IMU_SPRING;
        logoAngle += logoVel;

        if (currentScreen == SCREEN_LOGO) drawLogoScreen(logoAngle);

        vTaskDelay(pdMS_TO_TICKS(IMU_POLL_MS));
    }
}

// -- SDO helper: send an expedited read request --------------------------------
static void sdo_send_read(uint16_t param_id) {
    uint16_t index    = SDO_BASE_INDEX | (param_id >> 8);
    uint8_t  subindex = param_id & 0xFF;
    twai_message_t tx = {0};
    tx.identifier       = SDO_TX_ID;
    tx.data_length_code = 8;
    tx.data[0] = SDO_CMD_READ;
    tx.data[1] = index & 0xFF;
    tx.data[2] = index >> 8;
    tx.data[3] = subindex;
    twai_transmit(&tx, pdMS_TO_TICKS(5));
}

// -- SDO helper: parse response frame, return raw value or -1 on error --------
static int32_t sdo_parse_response(const twai_message_t *msg, uint16_t param_id) {
    if (msg->identifier != SDO_RX_ID) return -1;
    if (msg->data[0] == SDO_RESP_ABORT) return -1;
    if (msg->data[0] != SDO_RESP_4B && msg->data[0] != SDO_RESP_2B) return -1;
    // Verify param matches
    uint16_t index    = (uint16_t)(msg->data[1] | (msg->data[2] << 8));
    uint8_t  subindex = msg->data[3];
    uint16_t got_id   = (uint16_t)(((index & 0xFF) << 8) | subindex);
    if (got_id != param_id) return -1;
    int32_t raw = (int32_t)(msg->data[4] | (msg->data[5] << 8) |
                             (msg->data[6] << 16) | (msg->data[7] << 24));
    return raw;
}

// -- CAN task ------------------------------------------------------------------
void canTask(void* p) {
    TickType_t lastTx   = xTaskGetTickCount();
    TickType_t lastSdo  = xTaskGetTickCount();
    uint8_t    sdoPg    = 1;   // which page to poll next (1=Gear, 2=MotActive)

    // Param IDs for pages 1 and 2
    const uint16_t sdoParamId[NUM_PAGES] = { 0, PARAM_GEAR, PARAM_MOTACTIVE };

    for (;;) {
        // -- Drain RX, route SDO responses ------------------------------------
        twai_message_t msg;
        while (twai_receive(&msg, 0) == ESP_OK) {
            if (msg.identifier == SDO_RX_ID) {
                // Try to match against the page we last polled
                int32_t raw = sdo_parse_response(&msg, sdoParamId[sdoPg]);
                if (raw >= 0) {
                    // raw is x32 fixed-point; for integer params divide by 32
                    int8_t vcu_val = (int8_t)(raw / 32);
                    Serial.printf("SDO pg=%d param=%d raw=%d val=%d conf=%d\n",
                        (int)sdoPg, (int)sdoParamId[sdoPg], (int)raw, (int)vcu_val,
                        (int)confirmed[sdoPg]);
                    if (confirmed[sdoPg] >= 0 && vcu_val == confirmed[sdoPg]) {
                        acked[sdoPg] = true;
                    }
                }
            }
        }

        // -- Bus health -------------------------------------------------------
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK) {
            if      (st.state == TWAI_STATE_BUS_OFF)  twai_initiate_recovery();
            else if (st.state == TWAI_STATE_STOPPED)  twai_start();
            txFail = st.tx_failed_count;
        }

        // -- TX every 20ms ----------------------------------------------------
        if (xTaskGetTickCount() - lastTx >= pdMS_TO_TICKS(20)) {
            lastTx = xTaskGetTickCount();

            for (int pg = 0; pg < NUM_PAGES; pg++) {
                if (pg == 0 && confirmed[0] < 0) continue;
                twai_message_t tx = {0};
                tx.identifier       = pageCanId[pg];
                tx.data_length_code = 8;
                if (pg == 0) {
                    tx.data[3] = ((uint8_t)confirmed[0] << 4) & 0xF0;
                    tx.data[7] = cnt312 & 0x0F;
                    cnt312 = (cnt312 + 1) & 0x0F;
                } else {
                    tx.data[0] = (uint8_t)(confirmed[pg] >= 0 ?
                        confirmed[pg] : dialPos[pg]);
                }
                if (pg == 0) {
                    if (twai_transmit(&tx, 0) == ESP_OK) txCount++;
                } else {
                    twai_transmit(&tx, 0);
                }
            }
        }

        // -- SDO poll: alternate between page 1 and 2 every 500ms ------------
        if (xTaskGetTickCount() - lastSdo >= pdMS_TO_TICKS(SDO_POLL_MS)) {
            lastSdo = xTaskGetTickCount();
            // Alternate: poll page 1 (Gear) then page 2 (MotActive)
            sdoPg = (sdoPg == 1) ? 2 : 1;
            if (confirmed[sdoPg] >= 0) {
                sdo_send_read(sdoParamId[sdoPg]);
                Serial.printf("SDO poll pg=%d param=%d\n", (int)sdoPg, (int)sdoParamId[sdoPg]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// -- Utility -------------------------------------------------------------------
void initCAN() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("CAN install failed"); return;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("CAN start failed"); return;
    }
    Serial.println("CAN OK 500kbps NO_ACK TX=GPIO2 RX=GPIO1");
}

void bumpActivity() { lastActivityMs = millis(); }

// -- Draw page (called only from animTask) -------------------------------------
void drawPage() {
    int8_t pg = page;
    canvas.fillSprite(COL_BG);

    // Top half background
    canvas.fillRect(0, 0, 240, 220, COL_WEDGE_BG);

    // Outer ring
    canvas.drawCircle(CX, CY, R,     0x2104);
    canvas.drawCircle(CX, CY, R - 1, 0x4208);
    canvas.drawCircle(CX, CY, R - 2, 0x2104);

    // -- Moving wedge at current wedge angle ----------------------------------
    float wHalf  = ARC_WEDGE_DEG * M_PI / 180.0f;
    float wAngle = wedgeAngle[pg];
    float wStart = wAngle - wHalf;
    float wEnd   = wAngle + wHalf;

    // Convert to M5GFX arc degrees (0=right, clockwise)
    // M5GFX drawArc: angles in degrees, 0=right, increases clockwise
    // Our angles: 0=right, increasing CCW (standard math)
    // Conversion: m5gfx_deg = (360 - rad_deg) % 360
    auto toM5Deg = [](float rad) -> int {
        int d = (int)(rad * 180.0f / M_PI);
        d = (-d + 360) % 360;
        return d;
    };

    canvas.fillArc(CX, CY, R-3, 40,
        toM5Deg(wEnd), toM5Deg(wStart), 0x0419);
    canvas.drawLine(CX, CY,
        CX + (int)((R-3)*cosf(wStart)), CY + (int)((R-3)*sinf(wStart)), 0x03DF);
    canvas.drawLine(CX, CY,
        CX + (int)((R-3)*cosf(wEnd)),   CY + (int)((R-3)*sinf(wEnd)),   0x03DF);
    canvas.drawArc(CX, CY, R-3, R-5,
        toM5Deg(wEnd), toM5Deg(wStart), 0x07FF);

    // -- Fixed letters on arc -------------------------------------------------
    int   cnt      = pageCount[pg];
    float startR   = ARC_START_DEG * M_PI / 180.0f;
    float endR     = ARC_END_DEG   * M_PI / 180.0f;
    float halfSpan = (endR - startR) / 2.0f;

    for (int i = 0; i < cnt; i++) {
        float ang = indexToAngle(i, cnt);
        int gx = CX + (int)(ARC_R * cosf(ang));
        int gy = CY + (int)(ARC_R * sinf(ang));

        bool isSelected = (i == dialPos[pg]);
        bool isConfirmed = acked[pg] && (confirmed[pg] == i);

        // Distance from wedge centre for fade
        float diff = ang - wAngle;
        while (diff >  M_PI) diff -= 2.0f * M_PI;
        while (diff < -M_PI) diff += 2.0f * M_PI;
        float normDist = fabsf(diff) / halfSpan;
        float fade = 1.0f - normDist * 0.55f;
        if (fade < 0.1f) fade = 0.1f;

        uint32_t col;
        if (isConfirmed) col = COL_ACKED;
        else if (isSelected) col = COL_PENDING;
        else col = 0x8410;

        canvas.setTextColor(col);
        canvas.setTextDatum(CC_DATUM);
        if (isSelected || isConfirmed) {
            canvas.setFont(&fonts::FreeSansBold24pt7b);
            canvas.setTextSize(1);
        } else if (normDist < 0.6f) {
            canvas.setFont(&fonts::FreeSansBold18pt7b);
            canvas.setTextSize(1);
        } else {
            canvas.setFont(&fonts::FreeSans9pt7b);
            canvas.setTextSize(1);
        }
        canvas.drawString(pageLabels[pg][i], gx, gy);
        canvas.setFont(nullptr);
    }

    // -- Navigation dots -------------------------------------------------------
    // 3 dots centred at bottom, y=226
    int dotY = 226;
    int dotSpacing = 14;
    int dotStartX = CX - dotSpacing;
    for (int d = 0; d < NUM_PAGES; d++) {
        int dx = dotStartX + d * dotSpacing;
        if (d == pg) {
            canvas.fillCircle(dx, dotY, 4, COL_DOT_ACT);
        } else {
            canvas.drawCircle(dx, dotY, 3, COL_DOT_DIM);
        }
    }

    // -- Page 0: status badges for pages 1 and 2 ------------------------------
    if (pg == 0) {
        // H/L badge bottom-left
        const char* hlStr = (confirmed[1] >= 0) ?
            pageLabels[1][confirmed[1]] : pageLabels[1][dialPos[1]];
        canvas.setFont(&fonts::FreeSans9pt7b);
        canvas.setTextSize(1);
        canvas.setTextColor(COL_P1_BADGE);
        canvas.setTextDatum(BL_DATUM);
        canvas.drawString(hlStr, 14, 218);

        // MG badge bottom-right
        const char* mgStr = (confirmed[2] >= 0) ?
            pageLabels[2][confirmed[2]] : pageLabels[2][dialPos[2]];
        canvas.setTextColor(COL_P2_BADGE);
        canvas.setTextDatum(BR_DATUM);
        canvas.drawString(mgStr, 226, 218);
        canvas.setFont(nullptr);
    }

    canvas.pushSprite(0, 0);
}

// -- Logo screen ---------------------------------------------------------------
static inline void tp(float nx, float ny, float cosA, float sinA,
                      int16_t &sx, int16_t &sy) {
    sx = (int16_t)(LOGO_RADIUS * (nx*cosA - ny*sinA)) + LOGO_CENTER_X;
    sy = (int16_t)(LOGO_RADIUS * (nx*sinA + ny*cosA)) + LOGO_CENTER_Y;
}

void drawLogoScreen(float angle) {
    canvas.fillSprite(COL_LOGO_BG);
    float cosA = cosf(angle), sinA = sinf(angle);

    canvas.fillCircle(LOGO_CENTER_X, LOGO_CENTER_Y, LOGO_RADIUS,     COL_LOGO_FG);
    canvas.fillCircle(LOGO_CENTER_X, LOGO_CENTER_Y, LOGO_RADIUS - 3, COL_LOGO_BG);

    for (int i = 0; i < LOGO_OUTER_TRI_COUNT; i++) {
        int16_t x1,y1,x2,y2,x3,y3;
        tp(logo_outer_tris[i][0], logo_outer_tris[i][1], cosA, sinA, x1, y1);
        tp(logo_outer_tris[i][2], logo_outer_tris[i][3], cosA, sinA, x2, y2);
        tp(logo_outer_tris[i][4], logo_outer_tris[i][5], cosA, sinA, x3, y3);
        canvas.fillTriangle(x1,y1, x2,y2, x3,y3, COL_LOGO_FG);
    }

    for (int i = 0; i < LOGO_LINE_COUNT; i++) {
        if (i >= 72 && i <= 138) continue;
        int16_t sx1,sy1,sx2,sy2;
        tp(logo_lines[i][0], logo_lines[i][1], cosA, sinA, sx1, sy1);
        tp(logo_lines[i][2], logo_lines[i][3], cosA, sinA, sx2, sy2);
        canvas.drawLine(sx1, sy1, sx2, sy2, COL_LOGO_FG);
    }

    canvas.pushSprite(0, 0);
}

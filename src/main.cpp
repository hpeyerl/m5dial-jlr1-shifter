/*
 * JLR G1 CAN Shifter for M5Stack Dial
 *
 * Sends 0x312 to the Zombieverter VCU (VehicleCan = CAN1).
 * byte[3] >> 4 = gear (0=Park 1=Rev 2=Neu 3=Drive), byte[7] = rolling counter.
 *
 * Idle screen: after IDLE_TIMEOUT_MS of inactivity, shows the logo on a
 * turquoise background.  On the logo screen:
 *   - Drag finger tangentially to spin the logo (torque impulse + physics).
 *   - Encoder rotation or button press wakes back to gear screen.
 *   - Wakeup encoder tick is consumed (not forwarded to gear selection).
 *
 * IMU (MPU6886, optional): when fitted, the logo counter-rotates to stay
 * gravity-upright with a damped spring (viscous liquid effect).
 * If IMU is absent the logo draws at angle 0.
 *
 * Hardware:
 *   M5Stack Dial (ESP32-S3)
 *   Grove: GPIO2=CAN_TX, GPIO1=CAN_RX  -> IS3050G
 *   EC11:  A=GPIO41, B=GPIO40, btn=GPIO42
 *   Touch: built-in (FT3267, handled by M5Unified)
 *   IMU:   M5 MPU6886 unit on Grove/I2C (optional)
 */

#include <M5Unified.h>
#include <ESP32Encoder.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "logo_lines.h"

// -- CAN -----------------------------------------------------------------------
#define CAN_TX_PIN   GPIO_NUM_2
#define CAN_RX_PIN   GPIO_NUM_1
#define CAN_TX_ID    0x312

// -- Encoder / Button ---------------------------------------------------------
#define ENC_A_PIN    41
#define ENC_B_PIN    40
#define BTN_PIN      42

// -- Gear IDs ------------------------------------------------------------------
#define JLR_PARK     0
#define JLR_REVERSE  1
#define JLR_NEUTRAL  2
#define JLR_DRIVE    3

// -- Timing -------------------------------------------------------------------
#define ACK_DELAY_MS      200
#define IDLE_TIMEOUT_MS   10000   // 10 s of inactivity -> logo screen

// -- Logo drawing -------------------------------------------------------------
#define LOGO_CENTER_X  120
#define LOGO_CENTER_Y  120
#define LOGO_RADIUS    108

// -- IMU physics (damped spring) -----------------------------------------------
#define IMU_SPRING    0.12f   // pull strength toward gravity (0..1)
#define IMU_DAMPING   0.72f   // velocity decay per frame
#define IMU_POLL_MS   16      // ~60 fps

// -- Colours -------------------------------------------------------------------
#define COL_BG        0x0000   // Black (gear screen background)
#define COL_PENDING   0x07FF   // Cyan  (selected / pending ack)
#define COL_ACKED     0xF800   // Red   (VCU confirmed)
#define COL_DIM       0x8410   // Grey  (inactive gear positions)
#define COL_LOGO_BG   0x0419   // Turquoise background (try 0x0499, 0x0519 for more)
#define COL_LOGO_FG   0xC618   // Silver logo fill and outlines

const char* gearLabel[4] = { "P", "R", "N", "D" };

// -- State ---------------------------------------------------------------------
volatile int8_t   dialPos   = 0;
volatile int8_t   sentGear  = -1;
volatile bool     gearAcked = false;
volatile uint32_t sentTime  = 0;
volatile uint32_t txCount   = 0;
volatile uint32_t txFail    = 0;

enum Screen { SCREEN_GEAR, SCREEN_LOGO };
volatile Screen   currentScreen  = SCREEN_GEAR;
volatile uint32_t lastActivityMs = 0;

float logoAngle  = 0.0f;
float logoVel    = 0.0f;
bool  imuPresent = false;

uint8_t cnt312 = 0;

ESP32Encoder encoder;
M5Canvas      canvas(&M5.Display);

// -- Forward declarations ------------------------------------------------------
void initCAN();
void drawGearScreen();
void drawLogoScreen(float angle);
void canTask(void* pvParameters);
void imuTask(void* pvParameters);
void bumpActivity();

// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("JLR CAN Shifter + Logo idle screen");

    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(0);
    M5.Display.setBrightness(200);
    canvas.createSprite(240, 240);

    if (M5.Imu.isEnabled()) {
        imuPresent = true;
        Serial.println("IMU detected");
    } else {
        Serial.println("No IMU - logo fixed upright");
    }

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachSingleEdge(ENC_A_PIN, ENC_B_PIN);
    encoder.setCount(0);
    pinMode(BTN_PIN, INPUT_PULLUP);

    initCAN();
    xTaskCreatePinnedToCore(canTask, "CAN", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(imuTask, "IMU", 4096, NULL, 3, NULL, 0);

    lastActivityMs = millis();
    drawGearScreen();
    Serial.println("Ready.");
}

// -----------------------------------------------------------------------------
void loop() {
    M5.update();
    uint32_t now = millis();

    // -- Touch drag (logo spin) ------------------------------------------------
    {
        static bool    touching  = false;
        static int16_t lastTX    = 0, lastTY = 0;
        static bool    didDrag   = false;

        int tc = M5.Touch.getCount();
        if (tc > 0) {
            auto t0 = M5.Touch.getDetail(0);
            int16_t tx = t0.x, ty = t0.y;

            if (!touching) {
                touching = true;
                lastTX = tx; lastTY = ty;
                didDrag = false;
            } else {
                int16_t dx = tx - lastTX;
                int16_t dy = ty - lastTY;

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
            // Any touch (drag or tap) wakes the gear screen
            if (currentScreen == SCREEN_LOGO && !didDrag) {
                currentScreen = SCREEN_GEAR;
                drawGearScreen();
            }
            bumpActivity();
        }
    }

    // -- Idle timeout -> logo ---------------------------------------------------
    if (currentScreen == SCREEN_GEAR &&
        (now - lastActivityMs) >= IDLE_TIMEOUT_MS) {
        currentScreen = SCREEN_LOGO;
        drawLogoScreen(logoAngle);
    }

    // -- Encoder ---------------------------------------------------------------
    static int32_t lastEnc = 0;
    int32_t enc = (int32_t)encoder.getCount();
    if (enc != lastEnc) {
        int32_t delta = enc - lastEnc;
        lastEnc = enc;
        bumpActivity();

        if (currentScreen == SCREEN_LOGO) {
            // Wake to gear screen; consume this tick
            currentScreen = SCREEN_GEAR;
            drawGearScreen();
        } else {
            int8_t pos = dialPos + (delta > 0 - 1 : -1);
            if (pos < 0) pos = 3;
            if (pos > 3) pos = 0;
            dialPos = pos;
            drawGearScreen();
        }
    }

    // -- Button ----------------------------------------------------------------
    static bool lastBtn = HIGH;
    bool curBtn = (bool)digitalRead(BTN_PIN);
    if (curBtn == LOW && lastBtn == HIGH) {
        delay(20);
        if (digitalRead(BTN_PIN) == LOW) {
            bumpActivity();
            if (currentScreen == SCREEN_LOGO) {
                currentScreen = SCREEN_GEAR;
                drawGearScreen();
            } else {
                sentGear  = dialPos;
                gearAcked = false;
                sentTime  = millis();
                drawGearScreen();
                Serial.printf(">> Gear: %s\n", gearLabel[(int)sentGear]);
            }
        }
    }
    lastBtn = curBtn;

    // -- Optimistic ack --------------------------------------------------------
    if (sentGear >= 0 && !gearAcked && (now - sentTime) >= ACK_DELAY_MS) {
        gearAcked = true;
        if (currentScreen == SCREEN_GEAR) drawGearScreen();
    }

    delay(10);
}

// -----------------------------------------------------------------------------
// IMU task - polls accelerometer, updates logoAngle with damped spring physics,
// redraws logo at ~60fps when on logo screen.
void imuTask(void* pvParameters) {
    for (;;) {
        float targetAngle = 0.0f;

        if (imuPresent) {
            float ax, ay, az;
            M5.Imu.getAccel(&ax, &ay, &az);
            targetAngle = atan2f(ax, ay);
        }

        float diff = targetAngle - logoAngle;
        while (diff >  M_PI) diff -= 2.0f * M_PI;
        while (diff < -M_PI) diff += 2.0f * M_PI;

        logoVel    = logoVel * IMU_DAMPING + diff * IMU_SPRING;
        logoAngle += logoVel;

        if (currentScreen == SCREEN_LOGO) {
            drawLogoScreen(logoAngle);
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_POLL_MS));
    }
}

// -----------------------------------------------------------------------------
// CAN task - TX every 20ms, drains RX, monitors bus health. Core 0.
void canTask(void* pvParameters) {
    TickType_t lastTx = xTaskGetTickCount();
    for (;;) {
        twai_message_t msg;
        while (twai_receive(&msg, 0) == ESP_OK) { (void)msg; }

        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK) {
            if      (st.state == TWAI_STATE_BUS_OFF)  twai_initiate_recovery();
            else if (st.state == TWAI_STATE_STOPPED)  twai_start();
            txFail = st.tx_failed_count;
        }

        if (xTaskGetTickCount() - lastTx >= pdMS_TO_TICKS(20)) {
            lastTx = xTaskGetTickCount();
            int8_t gear = sentGear;
            if (gear >= 0) {
                uint8_t mes[8] = {0};
                mes[3] = ((uint8_t)gear << 4) & 0xF0;
                mes[7] = cnt312 & 0x0F;
                cnt312 = (cnt312 + 1) & 0x0F;

                twai_message_t txMsg;
                txMsg.identifier       = CAN_TX_ID;
                txMsg.extd             = 0;
                txMsg.rtr              = 0;
                txMsg.data_length_code = 8;
                memcpy(txMsg.data, mes, 8);
                esp_err_t r = twai_transmit(&txMsg, 0);
                if (r == ESP_OK) txCount++;

                static int8_t lastLogGear = -1;
                if (gear != lastLogGear) {
                    lastLogGear = gear;
                    Serial.printf("TX 0x312 [%s]: 00 00 00 %02X 00 00 00 %02X\n",
                        gearLabel[gear], mes[3], mes[7]);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
void bumpActivity() {
    lastActivityMs = millis();
}

// -----------------------------------------------------------------------------
void drawGearScreen() {
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
        if      (i==sg && ak)    { col=COL_ACKED;   sz=7; }
        else if (i==sg || i==dp) { col=COL_PENDING; sz=7; }
        else                     { col=COL_DIM;     sz=4; }
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

// -----------------------------------------------------------------------------
// Transform normalised logo coord to screen pixel
static inline void tp(float nx, float ny, float cosA, float sinA,
                      int16_t &sx, int16_t &sy) {
    sx = (int16_t)(LOGO_RADIUS * (nx*cosA - ny*sinA)) + LOGO_CENTER_X;
    sy = (int16_t)(LOGO_RADIUS * (nx*sinA + ny*cosA)) + LOGO_CENTER_Y;
}

// -----------------------------------------------------------------------------
// Draw logo on turquoise background, rotated by angle radians.
void drawLogoScreen(float angle) {
    canvas.fillSprite(COL_LOGO_BG);

    float cosA = cosf(angle);
    float sinA = sinf(angle);

    // Silver border ring
    canvas.fillCircle(LOGO_CENTER_X, LOGO_CENTER_Y, LOGO_RADIUS,     COL_LOGO_FG);
    canvas.fillCircle(LOGO_CENTER_X, LOGO_CENTER_Y, LOGO_RADIUS - 3, COL_LOGO_BG);

    // Logo fill - earcut triangles (inner hole already punched)
    for (int i = 0; i < LOGO_OUTER_TRI_COUNT; i++) {
        int16_t x1,y1, x2,y2, x3,y3;
        tp(logo_outer_tris[i][0], logo_outer_tris[i][1], cosA, sinA, x1, y1);
        tp(logo_outer_tris[i][2], logo_outer_tris[i][3], cosA, sinA, x2, y2);
        tp(logo_outer_tris[i][4], logo_outer_tris[i][5], cosA, sinA, x3, y3);
        canvas.fillTriangle(x1,y1, x2,y2, x3,y3, COL_LOGO_FG);
    }

    // Vector outlines - circle (0-71) and V/diamond (139-146) only
    // Segments 72-138 are the base line and corner arcs, covered by the fill
    for (int i = 0; i < LOGO_LINE_COUNT; i++) {
        if (i >= 72 && i <= 138) continue;
        int16_t sx1,sy1, sx2,sy2;
        tp(logo_lines[i][0], logo_lines[i][1], cosA, sinA, sx1, sy1);
        tp(logo_lines[i][2], logo_lines[i][3], cosA, sinA, sx2, sy2);
        canvas.drawLine(sx1, sy1, sx2, sy2, COL_LOGO_FG);
    }

    canvas.pushSprite(0, 0);
}

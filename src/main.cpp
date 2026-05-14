/*
 * JLR G1 CAN Shifter for M5Stack Dial
 *
 * Top half:    Arc dial with P/R/N/D, smooth animated rotation,
 *              wedge selector at 12 o'clock. Dial = scroll, Button = confirm.
 *
 * Bottom half: Touch zones for secondary controls.
 *              Row 1 (blue):  H / L  gear  (0x300 byte 0: 0=L 1=H)
 *              Row 2 (amber): MG1 / MG2 / MG1+2 / BLD  (0x301 byte 0: 0/1/2/3)
 *              Tap to select. Both rows send continuously at 20ms.
 *
 * CAN:
 *   0x312 byte[3]>>4 = PRND gear (0-3), byte[7] = rolling counter
 *   0x300 byte[0]    = H/L  (0=LOW 1=HIGH)
 *   0x301 byte[0]    = MG mode (0=MG1 1=MG2 2=MG1+MG2 3=Blend)
 *
 * Idle: 10s inactivity -> logo screen. Encoder/button/touch wakes back.
 * IMU (MPU6886): logo counter-rotates to stay gravity-upright.
 *
 * Hardware:
 *   M5Stack Dial (ESP32-S3)
 *   Grove: GPIO2=CAN_TX, GPIO1=CAN_RX -> IS3050G
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
#define CAN_ID_GEAR     0x312   // JLR PRND
#define CAN_ID_HL       0x300   // H/L transmission
#define CAN_ID_MG       0x301   // MG mode

// -- Encoder / Button ----------------------------------------------------------
#define ENC_A_PIN       41
#define ENC_B_PIN       40
#define BTN_PIN         42

// -- PRND gear IDs -------------------------------------------------------------
#define JLR_PARK        0
#define JLR_REVERSE     1
#define JLR_NEUTRAL     2
#define JLR_DRIVE       3

// -- H/L values ----------------------------------------------------------------
#define HL_LOW          0
#define HL_HIGH         1
#define HL_AUTO         2

// -- MG mode values ------------------------------------------------------------
#define MG_MG1          0
#define MG_MG2          1
#define MG_BOTH         2
#define MG_BLEND        3

// -- Build-time options ----------------------------------------------------------
// Set IDLE_TIMEOUT_S to 0 to disable the idle/logo screen entirely.
#define IDLE_TIMEOUT_S      10    // seconds before logo screen (0 = disabled)
#define ENABLE_IMU          1     // 1 = logo rotates with IMU tilt, 0 = fixed upright
// Shutdown after this many seconds with no 12V input (0 = disabled)
#define SHUTDOWN_TIMEOUT_S  120   // 2 minutes on battery before poweroff

// -- Timing (derived) ------------------------------------------------------------
#define ACK_DELAY_MS    200
#define IDLE_TIMEOUT_MS (IDLE_TIMEOUT_S * 1000)

// -- Display geometry ---------------------------------------------------------
#define CX              120
#define CY              120
#define R               116     // outer radius
#define DIVIDER_Y       130     // top/bottom split

// Arc dial geometry (top half)
#define ARC_R           86      // radius letters travel on

// Pick one arc option by defining ARC_OPT_A, ARC_OPT_B, or ARC_OPT_C
// ARC_OPT_A: wider arc, narrower wedge - all 4 letters visible
// ARC_OPT_B: widest arc, thinnest wedge
// default (neither defined): original layout
//#define ARC_OPT_B

#if defined(ARC_OPT_A)
  #define ARC_START_DEG   (-165)
  #define ARC_END_DEG     (-15)
  #define ARC_WEDGE_DEG   22.0f
#elif defined(ARC_OPT_B)
  #define ARC_START_DEG   (-170)
  #define ARC_END_DEG     (-10)
  #define ARC_WEDGE_DEG   18.0f
#else
  #define ARC_START_DEG   (-135)
  #define ARC_END_DEG     (-45)
  #define ARC_WEDGE_DEG   18.0f
#endif

// -- Logo ----------------------------------------------------------------------
#define LOGO_CENTER_X   120
#define LOGO_CENTER_Y   120
#define LOGO_RADIUS     108

// -- IMU physics ---------------------------------------------------------------
#define IMU_SPRING      0.12f
#define IMU_DAMPING     0.72f
#define IMU_POLL_MS     16

// -- Animation -----------------------------------------------------------------
// Dial angle smoothing - target snaps to gear position, display interpolates
#define ANIM_SPEED      0.18f   // fraction of remaining gap closed per frame

// -- Colours (RGB565) ----------------------------------------------------------
#define COL_BG          0x0000   // Black
#define COL_PENDING     0x07FF   // Cyan  - selected / pending ack
#define COL_ACKED       0xF800   // Red   - VCU confirmed
#define COL_DIM         0x4208   // Dark grey - inactive
#define COL_DIVIDER     0x2104   // Very dark grey divider line
#define COL_WEDGE_BG    0x2104   // Wedge fill - very dark grey (tweak to taste)
#define COL_LOGO_BG     0x0419   // Turquoise logo background
#define COL_LOGO_FG     0xC618   // Silver logo
// Bottom half button colours
#define COL_HL_ACT      0x07FF   // Blue active  (H/L row)
#define COL_HL_DIM      0x0219   // Blue dim
#define COL_MG_ACT      0xFD20   // Amber active (MG row)
#define COL_MG_DIM      0x4200   // Amber dim

const char* gearLabel[4]  = { "P", "R", "N", "D" };
const char* hlLabel[3]    = { "L", "H", "AUTO" };
const char* mgLabel[4]    = { "1", "2", "1+2", "BLD" };

// -- State ---------------------------------------------------------------------
// PRND
volatile int8_t   dialPos       = 0;     // encoder highlight position (0-3)
volatile int8_t   sentGear      = -1;    // confirmed by button press
volatile bool     gearAcked     = false;
volatile uint32_t sentTime      = 0;
float             dialAngle     = 0.0f;  // current rendered angle (smooth)
float             dialTarget    = 0.0f;  // target angle (snaps on encoder tick)

// H/L and MG
volatile int8_t   hlMode        = HL_HIGH;   // default High
volatile int8_t   mgMode        = MG_BLEND;  // default Blend

// CAN counters
uint8_t           cnt312        = 0;
volatile uint32_t txCount       = 0;
volatile uint32_t txFail        = 0;

// Screen
enum Screen { SCREEN_GEAR, SCREEN_LOGO };
volatile Screen   currentScreen  = SCREEN_GEAR;
volatile uint32_t lastActivityMs  = 0;
uint32_t          powerLostMs     = 0;   // millis() when 12V was lost (0=present)

// Logo / IMU
float logoAngle  = 0.0f;
float logoVel    = 0.0f;
bool  imuPresent = false;

ESP32Encoder encoder;
M5Canvas      canvas(&M5.Display);

// -- Forward declarations -------------------------------------------------------
void initCAN();
void drawGearScreen();
void drawLogoScreen(float angle);
void canTask(void* p);
void animTask(void* p);
void imuTask(void* p);
void bumpActivity();
void handleBottomTouch(int16_t tx, int16_t ty);

// -- Angle helpers -------------------------------------------------------------
// Map gear index 0-3 to arc angle in radians
// Arc runs from ARC_START_DEG to ARC_END_DEG, 4 positions evenly spaced
static float gearToAngle(int g) {
    float startR = ARC_START_DEG * M_PI / 180.0f;
    float endR   = ARC_END_DEG   * M_PI / 180.0f;
    return startR + (endR - startR) * g / 3.0f;
}

// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("JLR CAN Shifter v2 - arc dial + bottom touch");

    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(0);
    M5.Display.setBrightness(200);
    canvas.createSprite(240, 240);

#if ENABLE_IMU
    // MPU6886 unit is on the external I2C Grove port (red connector)
    // GPIO13=SDA, GPIO15=SCL. Must init Ex_I2C explicitly since
    // M5.Imu.isEnabled() only checks the internal bus.
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

    // Initialise with Park centred under the wedge
    dialAngle = dialTarget = -M_PI/2.0f - (ARC_START_DEG * M_PI / 180.0f);

    initCAN();

    // Core 0: CAN (highest priority) + IMU
    // Core 1: loop() + animTask (drawing must stay on one core)
    xTaskCreatePinnedToCore(canTask,  "CAN",  4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(imuTask,  "IMU",  4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(animTask, "ANIM", 4096, NULL, 4, NULL, 1);

    lastActivityMs = millis();
    drawGearScreen();
    Serial.println("Ready.");
}

// -----------------------------------------------------------------------------
void loop() {
    M5.update();
    uint32_t now = millis();

    // -- Touch ----------------------------------------------------------------
    {
        static bool    touching = false;
        static int16_t lastTX = 0, lastTY = 0;
        static bool    didDrag = false;

        int tc = M5.Touch.getCount();
        if (tc > 0) {
            auto t0 = M5.Touch.getDetail(0);
            int16_t tx = t0.x, ty = t0.y;

            if (!touching) {
                touching = true;
                lastTX = tx; lastTY = ty;
                didDrag = false;
            } else {
                int16_t dx = tx - lastTX, dy = ty - lastTY;
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
#if IDLE_TIMEOUT_S > 0
            if (currentScreen == SCREEN_LOGO && !didDrag) {
                currentScreen = SCREEN_GEAR;
            } else if (currentScreen == SCREEN_GEAR && !didDrag) {
                handleBottomTouch(lastTX, lastTY);
            }
#endif
            bumpActivity();
        }
    }

    // -- Idle timeout -> logo -------------------------------------------------
#if IDLE_TIMEOUT_S > 0
    if (currentScreen == SCREEN_GEAR &&
        (now - lastActivityMs) >= IDLE_TIMEOUT_MS) {
        currentScreen = SCREEN_LOGO;
        // imuTask will draw the logo screen
    }
#endif

    // -- Encoder --------------------------------------------------------------
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
            // Invert direction: rotating left (negative delta) advances the arc
            // left (higher index). Clamp at ends - no wrap.
            int8_t pos = dialPos + (delta < 0 ? 1 : -1);
            if (pos < 0) pos = 0;
            if (pos > 3) pos = 3;
            dialPos    = pos;
            // Target offset that brings gear[pos] to the top (-pi/2)
            dialTarget = -M_PI/2.0f - gearToAngle(pos);
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
                // animTask will redraw
            } else {
#else
            {
#endif
                sentGear  = dialPos;
                gearAcked = false;
                sentTime  = millis();
                Serial.printf(">> Gear: %s\n", gearLabel[(int)sentGear]);
            }
        }
    }
    lastBtn = curBtn;

    // -- Optimistic ack --------------------------------------------------------
    if (sentGear >= 0 && !gearAcked && (now - sentTime) >= ACK_DELAY_MS) {
        gearAcked = true;
    }

    // -- 12V loss shutdown --------------------------------------------------------
#if SHUTDOWN_TIMEOUT_S > 0
    {
        static uint32_t lastPowerCheck = 0;
        if (millis() - lastPowerCheck > 2000) {   // check every 2s
            lastPowerCheck = millis();
            if (M5.Power.isCharging()) {
                if (powerLostMs != 0) {
                    Serial.println("12V restored");
                }
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

// -----------------------------------------------------------------------------
// Handle a tap in the bottom half of the gear screen
void handleBottomTouch(int16_t tx, int16_t ty) {
    if (ty < DIVIDER_Y) return;   // top half - ignore

    // Layout (see drawGearScreen for button positions):
    // H/L row:  y = DIVIDER_Y+10 .. DIVIDER_Y+40  (height 30)
    //   L button: x = 44..104   (width 60)
    //   H button: x = 136..196  (width 60)
    // MG row:   y = DIVIDER_Y+50 .. DIVIDER_Y+78  (height 28)
    //   MG1: x=18..66   MG2: x=68..116  MG1+2: x=118..176  BLD: x=178..222

    int ry = ty - DIVIDER_Y;

    if (ry >= 10 && ry <= 38) {
        // H/L row - 3 buttons: L(x=27..85) H(x=91..149) AUTO(x=155..213)
        int hlW=58, hlGap=6;
        int hlX0 = CX - (3*hlW + 2*hlGap) / 2;
        for (int i = 0; i < 3; i++) {
            int bx = hlX0 + i*(hlW+hlGap);
            if (tx >= bx && tx <= bx+hlW) {
                hlMode = i;
                Serial.printf("HL -> %s\n", hlLabel[i]);
                break;
            }
        }
        bumpActivity();
    } else if (ry >= 48 && ry <= 70) {
        // MG row
        if      (tx >= 42 && tx <= 77)  { mgMode = MG_MG1;   Serial.println("MG -> MG1");   }
        else if (tx >= 79 && tx <= 114) { mgMode = MG_MG2;   Serial.println("MG -> MG2");   }
        else if (tx >= 116 && tx <= 162) { mgMode = MG_BOTH;  Serial.println("MG -> MG1+2"); }
        else if (tx >= 164 && tx <= 198) { mgMode = MG_BLEND; Serial.println("MG -> Blend"); }
        bumpActivity();
    }
}

// -----------------------------------------------------------------------------
// Animation task -- smooth dial + continuous gear screen redraw at ~60fps
void animTask(void* p) {
    for (;;) {
        if (currentScreen == SCREEN_GEAR) {
            float diff = dialTarget - dialAngle;
            if (fabsf(diff) > 0.001f) {
                dialAngle += diff * ANIM_SPEED;
            }
            drawGearScreen();
        }
        // 60fps during animation feels smooth; at idle this still only
        // redraws when loop() changes state, which is infrequent.
        // 30ms gives ~33fps which is fine and halves draw overhead vs 16ms.
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// -----------------------------------------------------------------------------
// IMU task -- logo physics
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

// -----------------------------------------------------------------------------
// CAN task -- TX 0x312, 0x300, 0x301 every 20ms. Core 0.
void canTask(void* p) {
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

            // 0x312 -- PRND
            if (sentGear >= 0) {
                uint8_t m[8] = {0};
                m[3] = ((uint8_t)sentGear << 4) & 0xF0;
                m[7] = cnt312 & 0x0F;
                cnt312 = (cnt312 + 1) & 0x0F;
                twai_message_t tx = {0};
                tx.identifier = CAN_ID_GEAR;
                tx.data_length_code = 8;
                memcpy(tx.data, m, 8);
                if (twai_transmit(&tx, 0) == ESP_OK) txCount++;
            }

            // 0x300 -- H/L
            {
                twai_message_t tx = {0};
                tx.identifier = CAN_ID_HL;
                tx.data_length_code = 8;
                tx.data[0] = (uint8_t)hlMode;
                twai_transmit(&tx, 0);
            }

            // 0x301 -- MG mode
            {
                twai_message_t tx = {0};
                tx.identifier = CAN_ID_MG;
                tx.data_length_code = 8;
                tx.data[0] = (uint8_t)mgMode;
                twai_transmit(&tx, 0);
            }

            static int8_t lastLogGear = -1;
            int8_t gear = sentGear;
            if (gear >= 0 && gear != lastLogGear) {
                lastLogGear = gear;
                Serial.printf("TX 0x312 [%s] 0x300 [%s] 0x301 [%s]\n",
                    gearLabel[gear], hlLabel[hlMode], mgLabel[mgMode]);
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
void bumpActivity() { lastActivityMs = millis(); }

// -----------------------------------------------------------------------------
// Draw a filled rounded rectangle on the canvas
static void fillRRect(int x, int y, int w, int h, int r, uint32_t col) {
    canvas.fillRoundRect(x, y, w, h, r, col);
}

// -----------------------------------------------------------------------------
void drawGearScreen() {
    canvas.fillSprite(COL_BG);

    // -- Top half background (letters sit on this, no black boxes) ------------
    // fillRect down to DIVIDER_Y so the background reaches the divider.
    // Corners outside the circle are black (invisible on round display).
    canvas.fillRect(0, 0, 240, DIVIDER_Y, COL_WEDGE_BG);

    // -- Outer ring ------------------------------------------------------------
    canvas.drawCircle(CX, CY, R,     0x2104);
    canvas.drawCircle(CX, CY, R - 1, 0x4208);
    canvas.drawCircle(CX, CY, R - 2, 0x2104);

    // -- Wedge selector (fixed at top, -90 degrees) ----------------------------
    float wHalf = ARC_WEDGE_DEG * M_PI / 180.0f;
    float wStart = -M_PI/2.0f - wHalf;
    float wEnd   = -M_PI/2.0f + wHalf;
    // Fill wedge slightly lighter than top half background
    canvas.fillArc(CX, CY, R - 3, 40, (int)(wStart * 180.0f / M_PI + 360) % 360,
                   (int)(wEnd * 180.0f / M_PI + 360) % 360, 0x0419);
    // Wedge border lines
    canvas.drawLine(CX, CY,
        CX + (int)((R-3) * cosf(wStart)), CY + (int)((R-3) * sinf(wStart)), 0x03DF);
    canvas.drawLine(CX, CY,
        CX + (int)((R-3) * cosf(wEnd)),   CY + (int)((R-3) * sinf(wEnd)),   0x03DF);
    // Wedge arc outline
    canvas.drawArc(CX, CY, R-3, R-5,
        (int)(wStart * 180.0f / M_PI + 360) % 360,
        (int)(wEnd   * 180.0f / M_PI + 360) % 360, 0x07FF);

    // -- PRND letters on arc ---------------------------------------------------
    float startR = ARC_START_DEG * M_PI / 180.0f;
    float endR   = ARC_END_DEG   * M_PI / 180.0f;

    for (int i = 0; i < 4; i++) {
        // dialAngle is the rotation offset bringing the selected gear to 12 o'clock.
        // Gear i renders at its base angle plus this offset.
        float ang = gearToAngle(i) + dialAngle;

        // Cull letters that have wandered far off the arc
        float arcMid = -M_PI / 2.0f;   // wedge is at -90 deg (12 o'clock)
        float angDiff = ang - arcMid;
        while (angDiff >  M_PI) angDiff -= 2.0f * M_PI;
        while (angDiff < -M_PI) angDiff += 2.0f * M_PI;
        float normDist = fabsf(angDiff) / ((endR - startR) / 2.0f); // 0=centre 1=edge 2=offscreen

        if (normDist > 2.1f) continue;   // only cull if truly off screen

        int gx = CX + (int)(ARC_R * cosf(ang));
        int gy = CY + (int)(ARC_R * sinf(ang));

        bool isSelected = (i == dialPos);
        bool isAcked    = gearAcked && (sentGear == i);

        // Size and brightness fall off with distance from selector
        float fade  = 1.0f - normDist * 0.55f;
        if (fade < 0.1f) fade = 0.1f;

        uint32_t col;

        if (isAcked) {
            col = COL_ACKED;
        } else if (isSelected) {
            col = COL_PENDING;
        } else {
            col = 0x8410;   // mid-grey for inactive
        }

        // Use smooth TrueType-style fonts scaled by distance
        canvas.setTextColor(col);   // transparent background - no black box
        canvas.setTextDatum(CC_DATUM);
        if (isSelected || isAcked) {
            canvas.setFont(&fonts::FreeSansBold24pt7b);
            canvas.setTextSize(1);
        } else if (normDist < 0.6f) {
            canvas.setFont(&fonts::FreeSansBold18pt7b);
            canvas.setTextSize(1);
        } else {
            canvas.setFont(&fonts::FreeSans9pt7b);
            canvas.setTextSize(1);
        }
        canvas.drawString(gearLabel[i], gx, gy);
        canvas.setFont(nullptr);   // reset to default
    }

    // -- Divider ---------------------------------------------------------------
    canvas.drawFastHLine(20, DIVIDER_Y, 200, COL_DIVIDER);

    // -- Bottom half: H/L row (y=DIVIDER_Y+10 to +38, 28px tall) -------------
    // 3 buttons (L, H, AUTO) evenly spaced
    int by1  = DIVIDER_Y + 10;
    int hlW  = 58, hlGap = 6;
    int hlX0 = CX - (3*hlW + 2*hlGap) / 2;
    for (int i = 0; i < 3; i++) {
        int bx = hlX0 + i * (hlW + hlGap);
        bool active = (i == hlMode);
        uint32_t bg  = active ? 0x0219 : 0x0808;
        uint32_t bdr = active ? COL_HL_ACT : 0x1082;
        uint32_t tc  = active ? COL_HL_ACT : 0x2104;
        fillRRect(bx, by1, hlW, 28, 6, bg);
        canvas.drawRoundRect(bx, by1, hlW, 28, 6, bdr);
        canvas.setTextColor(tc, bg);
        canvas.setTextDatum(CC_DATUM);
        canvas.setFont(active ? &fonts::FreeSansBold9pt7b : &fonts::FreeSans9pt7b);
        canvas.setTextSize(1);
        canvas.drawString(hlLabel[i], bx + hlW/2, by1 + 14);
        canvas.setFont(nullptr);
    }

    // -- Bottom half: MG row (y=DIVIDER_Y+48 to +70, 22px tall) --------------
    int by2 = DIVIDER_Y + 48;
    int mgW[4] = { 35, 35, 46, 34 };
    int mgX[4] = { 42, 79, 116, 164 };
    for (int i = 0; i < 4; i++) {
        bool active = (i == mgMode);
        uint32_t bg  = active ? 0x4200 : 0x0808;
        uint32_t bdr = active ? COL_MG_ACT : 0x1082;
        uint32_t tc  = active ? COL_MG_ACT : 0x2104;
        fillRRect(mgX[i], by2, mgW[i], 22, 4, bg);
        canvas.drawRoundRect(mgX[i], by2, mgW[i], 22, 4, bdr);
        canvas.setTextColor(tc, bg);
        canvas.setTextDatum(CC_DATUM);
        canvas.setFont(active ? &fonts::FreeSansBold9pt7b : &fonts::FreeSans9pt7b);
        canvas.setTextSize(1);
        canvas.drawString(mgLabel[i], mgX[i] + mgW[i]/2, by2 + 11);
        canvas.setFont(nullptr);
    }

    canvas.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// Logo screen (unchanged from previous version)
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

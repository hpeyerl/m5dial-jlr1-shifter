/*
 * main.cpp -- setup(), loop(), global definitions
 *
 * JLR G1 CAN Shifter for M5Stack Dial
 * Three arc-dial pages: PRND / H-L-AUTO / MG modes
 * See globals.h for all defines and shared state declarations.
 */

#include <ESP32Encoder.h>
#include "globals.h"
#include "logo_lines.h"

// -- Global definitions (declared extern in globals.h) -------------------------
const char* pageLabels[NUM_PAGES][4] = {
    { "P", "R", "N", "D"  },
    { "L", "H", "A", "HL" },   // AUTO->A, H/L->HL to fit in wedge
    { "1+2", "MG1", "MG2", "B"  },  // 0=MG1+2, 1=MG1, 2=MG2, 3=Blend
};
const int      pageCount[NUM_PAGES]  = { 4, 4, 4 };
const uint16_t pageCanId[NUM_PAGES]  = { CAN_ID_GEAR, CAN_ID_HL, CAN_ID_MG };

volatile int8_t   page                   = 0;
volatile int8_t   dialPos[NUM_PAGES]     = { 0, 1, 3 };
volatile int8_t   confirmed[NUM_PAGES]   = {-1,-1,-1};
volatile bool     acked[NUM_PAGES]       = {false,false,false};
volatile uint32_t sentTime[NUM_PAGES]    = {0,0,0};

float wedgeAngle[NUM_PAGES]  = {0,0,0};
float wedgeTarget[NUM_PAGES] = {0,0,0};

uint8_t           cnt312       = 0;
volatile uint32_t txCount      = 0;
volatile uint32_t txFail       = 0;

volatile Screen   currentScreen  = SCREEN_GEAR;
volatile uint32_t lastActivityMs = 0;
uint32_t          powerLostMs    = 0;

float logoAngle  = 0.0f;
float logoVel    = 0.0f;
bool  imuPresent = false;

M5Canvas canvas(&M5.Display);

static ESP32Encoder encoder;

// -- Helpers -------------------------------------------------------------------
void bumpActivity() { lastActivityMs = millis(); }

float indexToAngle(int idx, int count) {
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

    for (int p = 0; p < NUM_PAGES; p++) {
        wedgeAngle[p] = wedgeTarget[p] = indexToAngle(dialPos[p], pageCount[p]);
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

    // -- Touch -----------------------------------------------------------------
    {
        static bool    touching = false;
        static int16_t startTX  = 0, startTY = 0;
        static int16_t lastTX   = 0, lastTY  = 0;

        int tc = M5.Touch.getCount();
        if (tc > 0) {
            auto t0 = M5.Touch.getDetail(0);
            int16_t tx = t0.x, ty = t0.y;
            if (!touching) {
                touching = true;
                startTX = lastTX = tx;
                startTY = lastTY = ty;
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
                    bumpActivity();
                }
                lastTX = tx; lastTY = ty;
            }
        } else if (touching) {
            touching = false;
            int16_t totalDX = lastTX - startTX;
            int16_t totalDY = lastTY - startTY;
            if (currentScreen == SCREEN_LOGO) {
                currentScreen = SCREEN_GEAR;
                bumpActivity();
            } else if (abs(totalDX) > 40 && abs(totalDX) > abs(totalDY)*2) {
                if (totalDX < 0) {
                    if      (page == 0) page = 1;
                    else if (page == 1) page = 2;
                } else {
                    if      (page == 2) page = 1;
                    else if (page == 1) page = 0;
                }
                bumpActivity();
            }
        }
    }

    // -- Idle timeout ----------------------------------------------------------
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
            int8_t pg  = page;
            int8_t pos = dialPos[pg] + (delta > 0 ? 1 : -1);
            if (pos < 0) pos = 0;
            if (pos >= pageCount[pg]) pos = pageCount[pg] - 1;
            dialPos[pg]     = pos;
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
                int8_t pg     = page;
                confirmed[pg] = dialPos[pg];
                acked[pg]     = false;
                sentTime[pg]  = millis();
                Serial.printf(">> Page %d: %s\n", (int)pg,
                    pageLabels[pg][dialPos[pg]]);
            }
        }
    }
    lastBtn = curBtn;

    // -- Optimistic ack (page 0 only, pages 1+2 use SDO) ----------------------
    if (confirmed[0] >= 0 && !acked[0] && (now - sentTime[0]) >= ACK_DELAY_MS) {
        acked[0] = true;
    }

    // -- 12V loss / shutdown (in power.cpp) ------------------------------------
    checkPowerState();

    delay(10);
}

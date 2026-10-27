/*
 * globals.h -- shared state, defines, and extern declarations
 * Include this in every .cpp file.
 */
#pragma once

#include <M5Unified.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

// -- CAN -----------------------------------------------------------------------
#define CAN_TX_PIN      GPIO_NUM_2
#define CAN_RX_PIN      GPIO_NUM_1
#define CAN_ID_GEAR     0x312
#define CAN_ID_HL       0x300
#define CAN_ID_MG       0x301

// -- SDO -----------------------------------------------------------------------
#define SDO_TX_ID       0x603
#define SDO_RX_ID       0x583
#define SDO_BASE_INDEX  0x2100
#define SDO_CMD_READ    0x40
#define SDO_RESP_4B     0x43
#define SDO_RESP_2B     0x4B
#define SDO_RESP_ABORT  0x80
#define PARAM_GEAR      27
#define PARAM_MOTACTIVE 129
#define SDO_POLL_MS     500

// -- Encoder / Button ----------------------------------------------------------
#define ENC_A_PIN       41
#define ENC_B_PIN       40
#define BTN_PIN         42

// -- Build-time options --------------------------------------------------------
#define IDLE_TIMEOUT_S      300   // 5 minutes
#define ENABLE_IMU          1
#define SHUTDOWN_TIMEOUT_S  120

// -- Timing --------------------------------------------------------------------
#define ACK_DELAY_MS        200
#define IDLE_TIMEOUT_MS     (IDLE_TIMEOUT_S * 1000)
#define ANIM_SPEED          0.60f   // ~100ms to complete (95% done in 3 frames @30ms)

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
#define COL_PENDING     0x07FF
#define COL_ACKED       0xF800
#define COL_WEDGE_BG    0x2104
#define COL_DIVIDER     0x2104
#define COL_LOGO_BG     0x0419
#define COL_LOGO_FG     0xC618
#define COL_DOT_ACT     0xFFFF
#define COL_DOT_DIM     0x4208
#define COL_P1_BADGE    0x07FF
#define COL_P2_BADGE    0xFD20

// -- Page definitions ----------------------------------------------------------
#define NUM_PAGES       3

extern const char*    pageLabels[NUM_PAGES][4];
extern const int      pageCount[NUM_PAGES];
extern const uint16_t pageCanId[NUM_PAGES];

// -- Shared state --------------------------------------------------------------
extern volatile int8_t   page;
extern volatile int8_t   dialPos[NUM_PAGES];
extern volatile int8_t   confirmed[NUM_PAGES];
extern volatile bool     acked[NUM_PAGES];
extern volatile uint32_t sentTime[NUM_PAGES];

extern float wedgeAngle[NUM_PAGES];
extern float wedgeTarget[NUM_PAGES];

extern uint8_t           cnt312;
extern volatile uint32_t txCount;
extern volatile uint32_t txFail;

extern volatile uint32_t lastActivityMs;
extern uint32_t          powerLostMs;

extern float logoAngle;
extern float logoVel;
extern bool  imuPresent;

enum Screen { SCREEN_GEAR, SCREEN_LOGO };
extern volatile Screen currentScreen;

extern M5Canvas canvas;

// -- Shared functions ----------------------------------------------------------
void bumpActivity();
void drawPage();
void drawLogoScreen(float angle);
void initCAN();
float indexToAngle(int idx, int count);

// -- Task entry points ---------------------------------------------------------
void canTask(void* p);
void animTask(void* p);
void imuTask(void* p);

// -- Power ---------------------------------------------------------------------
void checkPowerState();
bool otaActive();

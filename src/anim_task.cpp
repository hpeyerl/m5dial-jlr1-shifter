/*
 * anim_task.cpp -- animTask(), drawPage(), drawLogoScreen()
 * Only this file calls drawPage() and drawLogoScreen().
 */

#include "globals.h"
#include "logo_lines.h"

// -- Logo pixel transform helper -----------------------------------------------
static inline void tp(float nx, float ny, float cosA, float sinA,
                      int16_t &sx, int16_t &sy) {
    sx = (int16_t)(LOGO_RADIUS * (nx*cosA - ny*sinA)) + LOGO_CENTER_X;
    sy = (int16_t)(LOGO_RADIUS * (nx*sinA + ny*cosA)) + LOGO_CENTER_Y;
}

// -- Draw logo screen ----------------------------------------------------------
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

// -- Draw gear page ------------------------------------------------------------
void drawPage() {
    int8_t pg = page;
    canvas.fillSprite(COL_BG);
    // Outer ring
    canvas.drawCircle(CX, CY, R,     0x2104);
    canvas.drawCircle(CX, CY, R - 1, 0x4208);
    canvas.drawCircle(CX, CY, R - 2, 0x2104);

    // Moving wedge
    float wHalf  = ARC_WEDGE_DEG * M_PI / 180.0f;
    float wAngle = wedgeAngle[pg];
    float wStart = wAngle - wHalf;
    float wEnd   = wAngle + wHalf;

    auto toM5Deg = [](float rad) -> int {
        int d = (int)(rad * 180.0f / M_PI);
        return ((d % 360) + 360) % 360;
    };

    canvas.fillArc(CX, CY, R-3, 0,
        toM5Deg(wStart), toM5Deg(wEnd), 0x0419);
    canvas.drawLine(CX, CY,
        CX + (int)((R-3)*cosf(wStart)), CY + (int)((R-3)*sinf(wStart)), 0x03DF);
    canvas.drawLine(CX, CY,
        CX + (int)((R-3)*cosf(wEnd)),   CY + (int)((R-3)*sinf(wEnd)),   0x03DF);
    canvas.drawArc(CX, CY, R-3, R-5,
        toM5Deg(wStart), toM5Deg(wEnd), 0x07FF);

    // Fixed letters on arc
    int   cnt      = pageCount[pg];
    float startR   = ARC_START_DEG * M_PI / 180.0f;
    float endR     = ARC_END_DEG   * M_PI / 180.0f;
    float halfSpan = (endR - startR) / 2.0f;

    for (int i = 0; i < cnt; i++) {
        float ang = indexToAngle(i, cnt);
        int gx = CX + (int)(ARC_R * cosf(ang));
        int gy = CY + (int)(ARC_R * sinf(ang));

        bool isSelected  = (i == dialPos[pg]);
        bool isConfirmed = acked[pg] && (confirmed[pg] == i);

        float diff = ang - wAngle;
        while (diff >  M_PI) diff -= 2.0f * M_PI;
        while (diff < -M_PI) diff += 2.0f * M_PI;
        float normDist = fabsf(diff) / halfSpan;

        uint32_t col;
        if      (isConfirmed) col = COL_ACKED;
        else if (isSelected)  col = COL_PENDING;
        else                  col = 0x8410;

        // Font size tracks wedge position (not dialPos) so letter grows
        // as wedge arrives, not before. Uses same diff/halfSpan as fade.
        canvas.setTextColor(col);
        canvas.setTextDatum(CC_DATUM);
        if (normDist < 0.25f) {
            canvas.setFont(&fonts::FreeSansBold24pt7b);
        } else {
            canvas.setFont(&fonts::FreeSans9pt7b);
        }
        canvas.setTextSize(1);
        canvas.drawString(pageLabels[pg][i], gx, gy);
        canvas.setFont(nullptr);
    }

    // Nav dots -- y=232 keeps them inside the circle (max half-width ~30px)
    int dotY = 210, dotSpacing = 14;
    int dotStartX = CX - dotSpacing;
    for (int d = 0; d < NUM_PAGES; d++) {
        int dx = dotStartX + d * dotSpacing;
        if (d == pg) canvas.fillCircle(dx, dotY, 4, COL_DOT_ACT);
        else         canvas.drawCircle(dx, dotY, 3, COL_DOT_DIM);
    }

    // Pages 1+2: full-word label below centre, updates with dialPos
    if (pg == 1) {
        static const char* hlFull[4] = { "Low", "High", "Auto", "Hi/Lo" };
        canvas.setFont(&fonts::FreeSans9pt7b);
        canvas.setTextSize(1);
        canvas.setTextColor(COL_PENDING);
        canvas.setTextDatum(TC_DATUM);
        canvas.drawString(hlFull[dialPos[1]], CX, 155);
        canvas.setFont(nullptr);
    } else if (pg == 2) {
        static const char* mgFull[4] = { "MG1+2", "MG1", "MG2", "Blend" };
        canvas.setFont(&fonts::FreeSans9pt7b);
        canvas.setTextSize(1);
        canvas.setTextColor(COL_PENDING);
        canvas.setTextDatum(TC_DATUM);
        canvas.drawString(mgFull[dialPos[2]], CX, 155);
        canvas.setFont(nullptr);
    }

    // Page 0: WiFi symbol when OTA is active -- centred between badges and dots
    if (pg == 0 && otaActive()) {
        // Fan opens upward: M5GFX 45=top-right, 90=top, 135=top-left
        int wx = CX, wy = 200;
        canvas.fillCircle(wx, wy, 3, 0x07FF);
        canvas.drawArc(wx, wy, 10, 8,  45, 135, 0x07FF);
        canvas.drawArc(wx, wy, 17, 15, 45, 135, 0x07FF);
        canvas.drawArc(wx, wy, 24, 22, 45, 135, 0x07FF);
    }

    // Page 0 status badges -- full words for readability
    if (pg == 0) {
        // Full-word labels for display on main page
        static const char* hlFull[4] = { "Low", "High", "Auto", "Hi/Lo" };
        static const char* mgFull[4] = { "MG1+2", "MG1", "MG2", "Blend" };

        int hlIdx = (confirmed[1] >= 0) ? confirmed[1] : dialPos[1];
        int mgIdx = (confirmed[2] >= 0) ? confirmed[2] : dialPos[2];

        canvas.setFont(&fonts::FreeSans9pt7b);
        canvas.setTextSize(1);
        canvas.setTextColor(COL_P1_BADGE);
        canvas.setTextDatum(BC_DATUM);
        canvas.drawString(hlFull[hlIdx], 88, 165);
        canvas.setTextColor(COL_P2_BADGE);
        canvas.setTextDatum(BC_DATUM);
        canvas.drawString(mgFull[mgIdx], 152, 165);
        canvas.setFont(nullptr);
    }

    canvas.pushSprite(0, 0);
}

// -- Animation task ------------------------------------------------------------
void animTask(void* p) {
    for (;;) {
        if (currentScreen == SCREEN_GEAR) {
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

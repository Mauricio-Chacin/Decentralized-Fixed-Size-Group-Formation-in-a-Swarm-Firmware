#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include <Wire.h>
#include "HWCDC.h"
#include <math.h>
#include <esp_system.h>  // esp_random()

// ==========================================
// FORCE VECTOR TEST  (Waveshare AMOLED watch)
// Agent i = the watch, fixed at (0,0).
// Agent j = random position, new one every cycle.
// Every 10 s: show parameters (4 s) -> draw net force vector (6 s) -> repeat.
// Cycles through the 5 pairwise interactions of the paper, in order.
// ==========================================

HWCDC USBSerial;

// ---- Display setup (same as Waveshare 01_HelloWorld) ----
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(bus, LCD_RESET, 0 /* rotation */,
                                      LCD_WIDTH, LCD_HEIGHT,
                                      22, 0, 0, 0);

#define COLOR_BLACK      0x0000
#define COLOR_WHITE      0xFFFF
#define COLOR_RED        0xF800
#define COLOR_GREEN      0x07E0
#define COLOR_BLUE       0x001F
#define COLOR_YELLOW     0xFFE0
#define COLOR_CYAN       0x07FF
#define COLOR_MAGENTA    0xF81F
#define COLOR_ORANGE     0xFD20
#define COLOR_DARKGREY   0x52AA
#define COLOR_GREY       0x8410
#define COLOR_LIGHTGREY  0xBDF7

// ==========================================
// MODEL PARAMETERS (edit these to experiment)
// Units are arbitrary "world units", same as the paper's simulation.
// ==========================================
constexpr float XI = 0.0f, YI = 0.0f;      // agent i (the watch): fixed position
constexpr float VIX = 0.0f, VIY = 0.0f;    // agent i velocity (fixed, at rest)

constexpr int   K_GROUP   = 6;       // target group size K
constexpr float PENALTY_R = 3.0f;    // logistic steepness r (matches the paper's D plot)
constexpr float ALPHA     = 1.0f;    // attraction gain  (must be < BETA)
constexpr float BETA      = 2.0f;    // repulsion gain
constexpr float D_EQ      = 1.0f;    // equilibrium spacing
constexpr float R_SENSE   = 2.5f;    // sensing radius
constexpr float K_COH     = 2.0f;    // spring constant (same group)
constexpr float C_DAMP    = 1.0f;    // damper constant (same group)
constexpr float BETA_GRP  = 4.0f;    // inter-group repulsion (strong)

// Repulsion cutoff is DERIVED, not free:  R_i = d_eq / (1 - alpha/beta)
constexpr float R_I = D_EQ / (1.0f - ALPHA / BETA);
static_assert(ALPHA < BETA, "Paper requires alpha < beta");

// Timing
constexpr unsigned long PARAM_MS = 4000;   // parameter screen
constexpr unsigned long CYCLE_MS = 10000;  // total per vector (params + vector)

// ==========================================
// THE 5 INTERACTIONS
// ==========================================
enum Interaction {
  FREE_FREE = 0,     // case 1: both free (density-gated by D)
  FREE_LOCKED,       // case 2: i free,   j locked
  LOCKED_FREE,       // case 3: i locked, j free
  SAME_GROUP,        // case 4: both locked, same group (spring-damper)
  DIFF_GROUP,        // case 5: both locked, different groups
  NUM_INTERACTIONS
};

const char *CASE_NAME[NUM_INTERACTIONS] = {
  "FREE - FREE", "FREE - LOCKED", "LOCKED - FREE", "SAME GROUP", "DIFFERENT GROUPS"
};
const char *CASE_STATE[NUM_INTERACTIONS] = {
  "i: free     j: free",
  "i: free     j: locked",
  "i: locked   j: free",
  "i, j: locked, same group",
  "i, j: locked, diff groups"
};
const uint16_t CASE_COLOR[NUM_INTERACTIONS] = {
  COLOR_GREEN, COLOR_ORANGE, COLOR_YELLOW, COLOR_CYAN, COLOR_MAGENTA
};

struct Params {
  int type;
  float xj, yj;        // agent j position (random)
  float vjx, vjy;      // agent j velocity (random, only used by SAME_GROUP)
  int rho;             // free-neighbor count (only used by FREE_FREE)
  float D;             // penalty D(rho)   (only used by FREE_FREE)
  uint32_t gi, gj;     // 24-bit group IDs (only shown for locked cases)
};

// ==========================================
// RANDOM + MATH
// ==========================================
float frand(float lo, float hi) {
  return lo + (hi - lo) * ((float)esp_random() / 4294967295.0f);
}

uint32_t randomGroupId() {
  uint32_t a;
  do { a = esp_random() & 0xFFFFFF; } while (a == 0 || a == 0xFFFFFF);
  return a;
}

// Logistic penalty: D(rho) = 1 / (1 + exp(-r((rho+1)/K - 1)))
float penaltyD(int rho) {
  return 1.0f / (1.0f + expf(-PENALTY_R * (((float)rho + 1.0f) / K_GROUP - 1.0f)));
}

// New random scenario for one interaction type
Params generateParams(int type) {
  Params p;
  p.type = type;

  // Agent j: random x,y. Where repulsion is the only force (cases 2,3,5) it is
  // zero beyond R_i, so j is placed inside R_i to always get a visible vector.
  float maxD = (type == FREE_FREE || type == SAME_GROUP) ? R_SENSE : (R_I - 0.1f);
  const float minD = 0.25f;  // avoid the singular d = 0
  float d;
  do {
    p.xj = frand(-maxD, maxD);
    p.yj = frand(-maxD, maxD);
    d = sqrtf((p.xj - XI) * (p.xj - XI) + (p.yj - YI) * (p.yj - YI));
  } while (d < minD || d > maxD);

  p.vjx = frand(-1.0f, 1.0f);
  p.vjy = frand(-1.0f, 1.0f);

  p.rho = (int)(esp_random() % K_GROUP);   // 0 .. K-1 (lock happens at K-1)
  p.D = penaltyD(p.rho);

  p.gi = randomGroupId();
  if (type == SAME_GROUP) {
    p.gj = p.gi;
  } else {
    do { p.gj = randomGroupId(); } while (p.gj == p.gi);
  }
  return p;
}

// Force on agent i caused by agent j (the paper's Algorithm 2).
// d_hat = unit vector from i toward j.
void computeForce(const Params &p, float &fx, float &fy) {
  float dx = p.xj - XI, dy = p.yj - YI;
  float d = sqrtf(dx * dx + dy * dy);
  float ux = dx / d, uy = dy / d;

  // Soft-core: max(0, (R_i - d) / R_i)
  float soft = fmaxf(0.0f, (R_I - d) / R_I);

  float mag = 0.0f;  // signed magnitude along d_hat (+ toward j, - away from j)

  switch (p.type) {
    case FREE_FREE:
      mag = ALPHA * (1.0f - p.D) - BETA * p.D * soft;
      break;

    case FREE_LOCKED:
    case LOCKED_FREE:
      mag = -BETA * soft;
      break;

    case SAME_GROUP: {
      // Assumption: dv_ij = v_i - v_j, so the minus sign in the paper's damper
      // term resists stretching of the link (a true spring-damper).
      float dvx = VIX - p.vjx, dvy = VIY - p.vjy;
      float dvAlong = dvx * ux + dvy * uy;
      mag = K_COH * (d - D_EQ) - C_DAMP * dvAlong;
      break;
    }

    case DIFF_GROUP:
      mag = -BETA_GRP * soft;
      break;
  }

  fx = mag * ux;
  fy = mag * uy;
}

// ==========================================
// DRAWING HELPERS
// ==========================================
void centerText(const char *s, int y, uint8_t size, uint16_t color) {
  int16_t x1, y1;
  uint16_t w, h;
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setCursor((gfx->width() - w) / 2 - x1, y);
  gfx->print(s);
}

void leftText(const char *s, int y, uint8_t size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(14, y);
  gfx->print(s);
}

// Thick line with an arrowhead at (x1, y1)
void drawArrow(int x0, int y0, int x1, int y1, uint16_t color) {
  float dx = (float)(x1 - x0), dy = (float)(y1 - y0);
  float len = sqrtf(dx * dx + dy * dy);
  if (len < 3.0f) {
    gfx->fillCircle(x0, y0, 3, color);
    return;
  }
  float ux = dx / len, uy = dy / len;
  float px = -uy, py = ux;  // perpendicular

  float head = fminf(20.0f, len * 0.6f);
  float halfW = head * 0.5f;

  // Shaft, 3 px thick, stops where the head starts
  int sx = x1 - (int)lroundf(ux * head * 0.8f);
  int sy = y1 - (int)lroundf(uy * head * 0.8f);
  for (int o = -1; o <= 1; o++) {
    gfx->drawLine(x0 + (int)lroundf(px * o), y0 + (int)lroundf(py * o),
                  sx + (int)lroundf(px * o), sy + (int)lroundf(py * o), color);
  }

  // Head
  float bx = x1 - ux * head, by = y1 - uy * head;
  gfx->fillTriangle(x1, y1,
                    (int)lroundf(bx + px * halfW), (int)lroundf(by + py * halfW),
                    (int)lroundf(bx - px * halfW), (int)lroundf(by - py * halfW),
                    color);
}

void waitUntil(unsigned long target) {
  while ((long)(millis() - target) < 0) delay(5);
}

// ==========================================
// SCREEN A: INITIAL PARAMETERS
// ==========================================
void showParamsScreen(int idx, const Params &p) {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextWrap(false);

  float d = sqrtf((p.xj - XI) * (p.xj - XI) + (p.yj - YI) * (p.yj - YI));
  char buf[40];

  snprintf(buf, sizeof(buf), "INTERACTION %d/%d", idx + 1, (int)NUM_INTERACTIONS);
  centerText(buf, 20, 2, COLOR_DARKGREY);
  centerText(CASE_NAME[p.type], 50, 3, CASE_COLOR[p.type]);
  centerText(CASE_STATE[p.type], 88, 2, COLOR_LIGHTGREY);

  int y = 135;
  const int dy = 28;

  snprintf(buf, sizeof(buf), "p_i = (%+.2f, %+.2f)", XI, YI);
  leftText(buf, y, 2, COLOR_WHITE); y += dy;
  snprintf(buf, sizeof(buf), "p_j = (%+.2f, %+.2f)", p.xj, p.yj);
  leftText(buf, y, 2, COLOR_WHITE); y += dy;
  snprintf(buf, sizeof(buf), "d_ij = %.2f   R_i = %.2f", d, R_I);
  leftText(buf, y, 2, COLOR_WHITE); y += dy;

  const char *formula = "";
  switch (p.type) {
    case FREE_FREE:
      snprintf(buf, sizeof(buf), "rho = %d   K = %d", p.rho, K_GROUP);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      snprintf(buf, sizeof(buf), "D = %.2f   r = %.1f", p.D, PENALTY_R);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      snprintf(buf, sizeof(buf), "alpha=%.1f  beta=%.1f", ALPHA, BETA);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      formula = "F=a(1-D)u - bD(R-d)/R u";
      break;

    case FREE_LOCKED:
    case LOCKED_FREE:
      snprintf(buf, sizeof(buf), "beta = %.1f", BETA);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      formula = "F = -b(R-d)/R u";
      break;

    case SAME_GROUP:
      snprintf(buf, sizeof(buf), "G(i)=G(j)=%06lX", (unsigned long)p.gi);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      snprintf(buf, sizeof(buf), "v_i = (%+.2f, %+.2f)", VIX, VIY);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      snprintf(buf, sizeof(buf), "v_j = (%+.2f, %+.2f)", p.vjx, p.vjy);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      snprintf(buf, sizeof(buf), "kcoh=%.1f cdamp=%.1f deq=%.1f", K_COH, C_DAMP, D_EQ);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      formula = "F=kcoh(d-deq)u-cdamp(dv.u)u";
      break;

    case DIFF_GROUP:
      snprintf(buf, sizeof(buf), "G(i) = %06lX", (unsigned long)p.gi);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      snprintf(buf, sizeof(buf), "G(j) = %06lX", (unsigned long)p.gj);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      snprintf(buf, sizeof(buf), "beta_grp = %.1f", BETA_GRP);
      leftText(buf, y, 2, COLOR_CYAN); y += dy;
      formula = "F = -bgrp(R-d)/R u";
      break;
  }

  y += 10;
  leftText(formula, y, 2, COLOR_YELLOW);
  leftText("u = unit vector from i to j", y + 26, 1, COLOR_GREY);

  centerText("Drawing vector...", gfx->height() - 50, 2, COLOR_DARKGREY);
}

// ==========================================
// SCREEN B: RESULTANT (NET FORCE) VECTOR
// ==========================================
void showVectorScreen(const Params &p, float fx, float fy) {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextWrap(false);

  int w = gfx->width(), h = gfx->height();
  int cx = w / 2, cy = h / 2 - 5;
  int plotR = min(w / 2 - 30, h / 2 - 130);        // pixel radius of the sensing circle
  float ppu = (float)plotR / R_SENSE;              // pixels per world unit
  float forceScale = (float)plotR / 2.0f;          // pixels per unit of force

  centerText(CASE_NAME[p.type], 14, 2, CASE_COLOR[p.type]);
  centerText(CASE_STATE[p.type], 38, 2, COLOR_LIGHTGREY);

  // Reference circles: d_eq, R_i, r_sense (with tiny labels)
  struct { float r; const char *name; uint16_t col; } rings[3] = {
    {D_EQ, "d_eq", COLOR_DARKGREY},
    {R_I, "R_i", COLOR_GREY},
    {R_SENSE, "r_sense", COLOR_DARKGREY}
  };
  for (int i = 0; i < 3; i++) {
    int rad = (int)lroundf(rings[i].r * ppu);
    gfx->drawCircle(cx, cy, rad, rings[i].col);
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_LIGHTGREY);
    gfx->setCursor(cx + 4, cy - rad + 3);
    gfx->print(rings[i].name);
  }

  // Positions in pixels (world y points up)
  int ix = cx + (int)lroundf(XI * ppu), iy = cy - (int)lroundf(YI * ppu);
  int jx = cx + (int)lroundf(p.xj * ppu), jy = cy - (int)lroundf(p.yj * ppu);

  gfx->drawLine(ix, iy, jx, jy, COLOR_DARKGREY);  // link i-j

  // Agents (white ring = locked)
  bool iLocked = (p.type == LOCKED_FREE || p.type == SAME_GROUP || p.type == DIFF_GROUP);
  bool jLocked = (p.type == FREE_LOCKED || p.type == SAME_GROUP || p.type == DIFF_GROUP);
  gfx->fillCircle(ix, iy, 7, COLOR_CYAN);
  gfx->fillCircle(jx, jy, 7, COLOR_ORANGE);
  if (iLocked) gfx->drawCircle(ix, iy, 11, COLOR_WHITE);
  if (jLocked) gfx->drawCircle(jx, jy, 11, COLOR_WHITE);
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_CYAN);
  gfx->setCursor(ix + 12, iy + 6);
  gfx->print("i");
  gfx->setTextColor(COLOR_ORANGE);
  gfx->setCursor(jx + 12, jy - 20);
  gfx->print("j");

  // Net force vector, drawn from agent i
  float mag = sqrtf(fx * fx + fy * fy);
  bool clamped = false;
  if (mag > 0.01f) {
    float lenPx = mag * forceScale;
    float maxLen = plotR * 1.15f;
    float s = forceScale;
    if (lenPx > maxLen) { s = forceScale * maxLen / lenPx; clamped = true; }
    int ex = ix + (int)lroundf(fx * s);
    int ey = iy - (int)lroundf(fy * s);
    drawArrow(ix, iy, ex, ey, COLOR_RED);
  }

  // Numbers
  char buf[40];
  int y0 = h - 115;
  snprintf(buf, sizeof(buf), "Fx=%+.2f  Fy=%+.2f", fx, fy);
  centerText(buf, y0, 2, COLOR_WHITE);
  if (mag > 0.01f) {
    int angle = (int)lroundf(atan2f(fy, fx) * 57.29578f);
    snprintf(buf, sizeof(buf), "|F|=%.2f  ang=%+d deg", mag, angle);
  } else {
    snprintf(buf, sizeof(buf), "|F|=%.2f  (no force)", mag);
  }
  centerText(buf, y0 + 26, 2, COLOR_RED);
  if (clamped) centerText("(arrow scaled down to fit)", y0 + 54, 1, COLOR_GREY);
}

// ==========================================
// ARDUINO ENTRY POINTS
// ==========================================
int caseIdx = 0;

void setup() {
  USBSerial.begin(115200);

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  if (!gfx->begin()) {
    USBSerial.println("Display initialization failed!");
  }
  USBSerial.println("Force vector test: agent i fixed at (0,0), agent j random");
}

void loop() {
  unsigned long t0 = millis();

  Params p = generateParams(caseIdx);
  float fx, fy;
  computeForce(p, fx, fy);

  USBSerial.printf("[%d/5] %s  p_j=(%+.2f,%+.2f)  F=(%+.3f,%+.3f)\n",
                   caseIdx + 1, CASE_NAME[p.type], p.xj, p.yj, fx, fy);

  showParamsScreen(caseIdx, p);   // 1) initial parameters
  waitUntil(t0 + PARAM_MS);

  showVectorScreen(p, fx, fy);    // 2) resultant vector
  waitUntil(t0 + CYCLE_MS);       // 3) repeat with new random parameters

  caseIdx = (caseIdx + 1) % NUM_INTERACTIONS;
}

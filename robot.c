/* =====================================================================
 *  ROBOT_COORDINATE.C
 *  Coordinate-based humanoid robot walking simulation
 *  ---------------------------------------------------------------
 *  Compiler : Turbo C 3.0 (C language, BGI graphics)
 *  Graphics : 640 x 480 BGI mode
 *
 *  How it works:
 *    - The robot starts at world coordinate (0,0).
 *    - Enter one destination (X,Y).
 *    - Add as many extra waypoints as needed, or enter N to finish.
 *    - The program draws a connected walkable surface through the points.
 *    - The robot follows the surface using an animated walking cycle.
 *
 *  Coordinates may be positive or negative. Maximum: 20 waypoints.
 *  This is a 2D computer-graphics simulation, not a physical robot.
 * ===================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <conio.h>
#include <math.h>
#include <graphics.h>

#if defined(__TURBOC__) || defined(__BORLANDC__)
#include <dos.h>
#endif

#define PI                 3.14159265358979
#define MAX_POINTS         21       /* origin + 20 user waypoints */
#define FRAMES_PER_SEGMENT 32
#define DEFAULT_DELAY      28
#define STAGGER            7

#define PLOT_L   44
#define PLOT_R   432
#define PLOT_T   26
#define PLOT_B   432
#define PANEL_L  440
#define PANEL_R  636
#define MSG_T    438
#define MSG_B    476

#define HIP_H       54.0
#define THIGH_L     32.0
#define SHIN_L      32.0
#define ANKLE_H      6.0
#define FOOT_FWD    12
#define FOOT_BACK    5
#define FOOT_H       6
#define TORSO_H     34.0
#define NECK_H       6
#define HEAD_W      26
#define HEAD_H      22
#define UPPER_ARM   22.0
#define FOREARM_L   20.0

#ifndef BLACK
#define BLACK 0
#endif
#ifndef BLUE
#define BLUE 1
#endif
#ifndef GREEN
#define GREEN 2
#endif
#ifndef CYAN
#define CYAN 3
#endif
#ifndef RED
#define RED 4
#endif
#ifndef MAGENTA
#define MAGENTA 5
#endif
#ifndef BROWN
#define BROWN 6
#endif
#ifndef LIGHTGRAY
#define LIGHTGRAY 7
#endif
#ifndef DARKGRAY
#define DARKGRAY 8
#endif
#ifndef LIGHTBLUE
#define LIGHTBLUE 9
#endif
#ifndef LIGHTGREEN
#define LIGHTGREEN 10
#endif
#ifndef LIGHTCYAN
#define LIGHTCYAN 11
#endif
#ifndef LIGHTRED
#define LIGHTRED 12
#endif
#ifndef LIGHTMAGENTA
#define LIGHTMAGENTA 13
#endif
#ifndef YELLOW
#define YELLOW 14
#endif
#ifndef WHITE
#define WHITE 15
#endif

/* ------------------------- Global data ------------------------------ */
typedef struct {
    double x;
    double y;
} RoutePoint;

static RoutePoint gPoints[MAX_POINTS];
static int gPointCount = 1;
static int gCurrentIndex = 0;
static int gRouteReady = 0;
static int gDir = 1;
static int gLead = 1;
static int gSpeedDelay = DEFAULT_DELAY;
static double gCurX = 0.0;
static double gCurY = 0.0;
static double gScaleX = 30.0;
static double gScaleY = 30.0;
static double gOriginX = 0.0;
static double gOriginY = 0.0;
static double gMinX = -1.0;
static double gMaxX = 1.0;
static double gMinY = -1.0;
static double gMaxY = 4.0;
static double gLastTX = 1.0;
static double gLastTY = 0.0;
static unsigned char gBuf[20000];
static int gBoxX0, gBoxY0, gBoxX1, gBoxY1;
static int gSaved = 0;
static unsigned long gBlink = 0;

/* --------------------------- Utilities ------------------------------ */
static void w2s(double wx, double wy, int *sx, int *sy)
{
    *sx = (int)(gOriginX + wx * gScaleX + 0.5);
    *sy = (int)(gOriginY - wy * gScaleY + 0.5);
}

static double smoothstep(double t)
{
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return t * t * (3.0 - 2.0 * t);
}

static void txt(int x, int y, int col, const char *s)
{
    setcolor(col);
    outtextxy(x, y, (char *)s);
}

static void txtc(int cx, int y, int col, const char *s)
{
    setcolor(col);
    outtextxy(cx - textwidth((char *)s) / 2, y, (char *)s);
}

static void clearMsg(void)
{
    setfillstyle(SOLID_FILL, BLACK);
    bar(1, MSG_T, 638, MSG_B - 1);
}

static void showMsg(const char *s, int col)
{
    clearMsg();
    txt(8, MSG_T + 6, col, s);
}

static void waitKey(void)
{
    txt(8, MSG_T + 22, DARKGRAY, "Press any key to continue...");
    getch();
}

/* ----------------------- Graphics primitives ------------------------ */
static void disc(int x, int y, int r, int col)
{
    setfillstyle(SOLID_FILL, col);
    setcolor(col);
    pieslice(x, y, 0, 360, r);
}

static void capsule(int x0, int y0, int x1, int y1, int r, int col)
{
    double dx, dy, L, nx, ny;
    int p[8];

    dx = (double)(x1 - x0);
    dy = (double)(y1 - y0);
    L = sqrt(dx * dx + dy * dy);
    if (L < 0.5) {
        disc(x0, y0, r, col);
        return;
    }
    nx = -dy / L * r;
    ny = dx / L * r;

    p[0] = (int)(x0 + nx + 0.5); p[1] = (int)(y0 + ny + 0.5);
    p[2] = (int)(x1 + nx + 0.5); p[3] = (int)(y1 + ny + 0.5);
    p[4] = (int)(x1 - nx + 0.5); p[5] = (int)(y1 - ny + 0.5);
    p[6] = (int)(x0 - nx + 0.5); p[7] = (int)(y0 - ny + 0.5);

    setfillstyle(SOLID_FILL, col);
    setcolor(col);
    fillpoly(4, p);
    disc(x0, y0, r, col);
    disc(x1, y1, r, col);
}

/* Two-link inverse kinematics calculates the knee position. */
static void ik2(double hx, double hy, double tx, double ty,
                double L1, double L2, int bendSign,
                double *ex, double *ey)
{
    double dx, dy, d, a, cosA, A;

    dx = tx - hx;
    dy = ty - hy;
    d = sqrt(dx * dx + dy * dy);
    if (d < 0.001) {
        dx = 0.0;
        dy = 1.0;
        d = 0.001;
    }
    if (d > L1 + L2 - 0.5) d = L1 + L2 - 0.5;

    a = atan2(dy, dx);
    cosA = (L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d);
    if (cosA > 1.0) cosA = 1.0;
    if (cosA < -1.0) cosA = -1.0;
    A = acos(cosA);
    a = a - A * bendSign;

    *ex = hx + L1 * cos(a);
    *ey = hy + L1 * sin(a);
}

typedef struct {
    double hipx, hipy;
    int    dir;
    double ankx[2], anky[2], frot[2];
    double armA[2], armE[2];
    double lean;
    double squash;
    int    swapDepth;
    double headSquash;
    int    headDir;
} Pose;

static void drawFoot(Pose *p, int idx, int far)
{
    int ax = (int)(p->ankx[idx] + 0.5);
    int ay = (int)(p->anky[idx] + 0.5);
    double rot = p->frot[idx];
    double c = cos(rot), s = sin(rot);
    double sq = p->squash;
    int    dir = p->dir;
    int    lx[4], ly[4], poly[8], i;

    lx[0] = -FOOT_BACK; ly[0] = 0;
    lx[1] =  FOOT_FWD;  ly[1] = 0;
    lx[2] =  FOOT_FWD;  ly[2] = FOOT_H;
    lx[3] = -FOOT_BACK; ly[3] = FOOT_H;

    for (i = 0; i < 4; i++) {
        double fwd = lx[i] * c - ly[i] * s;
        double dwn = lx[i] * s + ly[i] * c;
        poly[i * 2]     = ax + (int)(dir * sq * fwd + (dir > 0 ? 0.5 : -0.5));
        poly[i * 2 + 1] = ay + (int)(dwn + 0.5);
    }
    setfillstyle(SOLID_FILL, far ? DARKGRAY : LIGHTGRAY);
    setcolor(far ? DARKGRAY : LIGHTGRAY);
    fillpoly(4, poly);

    disc(ax, ay, 4, far ? BLUE : LIGHTBLUE);
}

static void drawLeg(Pose *p, int idx, int far)
{
    int hx, hy, kx, ky;
    int colBody  = far ? DARKGRAY : LIGHTGRAY;
    int colJoint = far ? BLUE     : LIGHTBLUE;
    double kxd, kyd;

    hx = (int)(p->hipx + 0.5);
    hy = (int)(p->hipy + 0.5) + (far ? 2 : 0);

    ik2((double)hx, (double)hy, p->ankx[idx], p->anky[idx],
        THIGH_L, SHIN_L, p->dir, &kxd, &kyd);
    kx = (int)(kxd + 0.5);
    ky = (int)(kyd + 0.5);

    capsule(hx, hy, kx, ky, 5, colBody);
    capsule(kx, ky, (int)(p->ankx[idx] + 0.5),
                     (int)(p->anky[idx] + 0.5), 4, colBody);

    disc(kx, ky, 5, colJoint);
    drawFoot(p, idx, far);
}

static void drawTorso(Pose *p)
{
    int hx = (int)(p->hipx + 0.5);
    int hy = (int)(p->hipy + 0.5);
    int tx, ty, wb, wt, poly[10];

    wb = (int)(10 * p->squash);
    wt = (int)(14 * p->squash);
    if (wb < 3) wb = 3;
    if (wt < 4) wt = 4;

    tx = hx + (int)(sin(p->lean) * TORSO_H * p->dir * p->squash);
    ty = hy - (int)(cos(p->lean) * TORSO_H);

    /* pelvis */
    setfillstyle(SOLID_FILL, LIGHTGRAY);
    bar(hx - wb, hy - 4, hx + wb, hy + 8);
    disc(hx, hy, 5, LIGHTBLUE);

    /* torso trapezoid */
    poly[0] = hx - wb; poly[1] = hy - 2;
    poly[2] = tx - wt; poly[3] = ty;
    poly[4] = tx + wt; poly[5] = ty;
    poly[6] = hx + wb; poly[7] = hy - 2;
    setfillstyle(SOLID_FILL, LIGHTGRAY);
    setcolor(WHITE);
    fillpoly(4, poly);

    /* chest panel in cyan */
    poly[0] = hx - (int)(6 * p->squash);
    poly[1] = hy - 12;
    poly[2] = tx - (int)(8 * p->squash);
    poly[3] = ty + 8;
    poly[4] = tx + (int)(8 * p->squash);
    poly[5] = ty + 8;
    poly[6] = hx + (int)(6 * p->squash);
    poly[7] = hy - 12;
    setfillstyle(SOLID_FILL, CYAN);
    setcolor(CYAN);
    fillpoly(4, poly);

    /* power core */
    disc(tx, ty + 16, 3, YELLOW);
}

static void drawHead(Pose *p)
{
    int hx = (int)(p->hipx + 0.5);
    int hy = (int)(p->hipy + 0.5);
    int tx, ty, hcx, hcy, hw, hh;
    int hd = p->headDir;
    double hsq = p->headSquash;

    tx = hx + (int)(sin(p->lean) * TORSO_H * p->dir * p->squash);
    ty = hy - (int)(cos(p->lean) * TORSO_H);

    hcx = tx + (int)(sin(p->lean) * p->dir * p->squash *
                    (NECK_H + HEAD_H / 2.0));
    hcy = ty - (int)(cos(p->lean) * (NECK_H + HEAD_H / 2.0));

    /* neck */
    capsule(tx, ty, hcx, hcy + HEAD_H / 2 - 2, 4, LIGHTBLUE);

    hw = (int)(HEAD_W / 2.0 * hsq);
    if (hw < 4) hw = 4;
    hh = HEAD_H / 2;

    /* head box */
    setfillstyle(SOLID_FILL, LIGHTGRAY);
    setcolor(WHITE);
    bar(hcx - hw, hcy - hh, hcx + hw, hcy + hh);
    rectangle(hcx - hw, hcy - hh, hcx + hw, hcy + hh);

    /* visor on the facing side */
    setfillstyle(SOLID_FILL, BLUE);
    setcolor(BLUE);
    if (hd > 0) bar(hcx - hw / 2, hcy - 5, hcx + hw - 2, hcy + 2);
    else        bar(hcx - hw + 2, hcy - 5, hcx + hw / 2, hcy + 2);

    /* eye */
    disc(hcx + hd * (hw / 2), hcy - 1, 2, YELLOW);

    /* antenna with blinking tip */
    setcolor(LIGHTGRAY);
    line(hcx, hcy - hh, hcx, hcy - hh - 8);
    disc(hcx, hcy - hh - 8, 2,
         ((gBlink / 6) % 2) ? RED : YELLOW);
}

static void drawArm(Pose *p, int idx, int far)
{
    int sx, sy, ex, ey, wx, wy;
    double a = p->armA[idx];
    double e = p->armE[idx];
    int dir = p->dir;
    double sq = p->squash;
    int col  = far ? DARKGRAY : LIGHTGRAY;
    int colA = far ? BLUE     : CYAN;

    sx = (int)(p->hipx + 0.5) +
         (int)(sin(p->lean) * TORSO_H * dir * sq) +
         (far ? (int)(-3 * dir * sq) : (int)(3 * dir * sq));
    sy = (int)(p->hipy + 0.5) -
         (int)(cos(p->lean) * TORSO_H) + 4;

    ex = sx + (int)(UPPER_ARM * sin(a) * dir * sq);
    ey = sy + (int)(UPPER_ARM * cos(a));

    wx = ex + (int)(FOREARM_L * sin(a + e) * dir * sq);
    wy = ey + (int)(FOREARM_L * cos(a + e));

    capsule(sx, sy, ex, ey, 4, col);
    capsule(ex, ey, wx, wy, 3, col);
    disc(ex, ey, 4, LIGHTBLUE);
    disc(wx, wy, 4, colA);
}

static void drawFrame(Pose *p)
{
    int farIdx  = p->swapDepth ? 1 : 0;
    int nearIdx = 1 - farIdx;

    drawArm(p, farIdx, 1);
    drawLeg(p, farIdx, 1);
    drawTorso(p);
    drawHead(p);
    drawLeg(p, nearIdx, 0);
    drawArm(p, nearIdx, 0);
}


/* ---------------------- Dynamic scene drawing ----------------------- */
static void computeScale(void)
{
    double minx, maxx, miny, maxy;
    double availW, availH, sx, sy;
    int i;

    minx = maxx = gPoints[0].x;
    miny = maxy = gPoints[0].y;
    for (i = 1; i < gPointCount; i++) {
        if (gPoints[i].x < minx) minx = gPoints[i].x;
        if (gPoints[i].x > maxx) maxx = gPoints[i].x;
        if (gPoints[i].y < miny) miny = gPoints[i].y;
        if (gPoints[i].y > maxy) maxy = gPoints[i].y;
    }

    gMinX = minx - 1.0;
    gMaxX = maxx + 1.0;
    gMinY = miny - 1.0;
    gMaxY = maxy + 4.0;  /* reserve room above the surface for the robot */

    availW = (double)(PLOT_R - PLOT_L - 30);
    availH = (double)(PLOT_B - PLOT_T - 135);
    sx = availW / (gMaxX - gMinX);
    sy = availH / (gMaxY - gMinY);
    gScaleX = (sx < sy) ? sx : sy;
    gScaleY = gScaleX;
    if (gScaleX > 42.0) {
        gScaleX = 42.0;
        gScaleY = 42.0;
    }
    if (gScaleX < 0.5) {
        gScaleX = 0.5;
        gScaleY = 0.5;
    }

    gOriginX = (double)(PLOT_L + 15) - gMinX * gScaleX;
    gOriginY = (double)(PLOT_B - 22) + gMinY * gScaleY;
}

static void drawGrid(void)
{
    int i, x, y, xStep, yStep;
    double v, span;
    char s[12];

    span = gMaxX - gMinX;
    xStep = 1;
    if (span > 18.0) xStep = 2;
    if (span > 45.0) xStep = 5;
    if (span > 100.0) xStep = 10;

    setcolor(DARKGRAY);
    setlinestyle(DOTTED_LINE, 0, NORM_WIDTH);
    for (v = floor(gMinX / xStep) * xStep; v <= gMaxX; v += xStep) {
        w2s(v, 0.0, &x, &y);
        line(x, PLOT_T + 2, x, PLOT_B - 2);
    }
    span = gMaxY - gMinY;
    yStep = 1;
    if (span > 18.0) yStep = 2;
    if (span > 45.0) yStep = 5;
    if (span > 100.0) yStep = 10;
    for (v = floor(gMinY / yStep) * yStep; v <= gMaxY; v += yStep) {
        w2s(0.0, v, &x, &y);
        line(PLOT_L + 2, y, PLOT_R - 2, y);
    }
    setlinestyle(SOLID_LINE, 0, NORM_WIDTH);

    /* Small coordinate labels along the X axis. */
    for (i = (int)floor(gMinX / xStep); i <= (int)ceil(gMaxX / xStep); i++) {
        double wx;
        wx = (double)i * xStep;
        w2s(wx, 0.0, &x, &y);
        if (x > PLOT_L + 3 && x < PLOT_R - 18 && y > PLOT_T && y < PLOT_B - 12) {
            sprintf(s, "%d", (int)wx);
            txt(x - 3, y + 4, DARKGRAY, s);
        }
    }
}

static void drawAxes(void)
{
    int x0, y0, x1, y1;

    setcolor(LIGHTGRAY);
    if (gMinY <= 0.0 && gMaxY >= 0.0) {
        w2s(gMinX, 0.0, &x0, &y0);
        w2s(gMaxX, 0.0, &x1, &y1);
        line(x0, y0, x1, y1);
    }
    if (gMinX <= 0.0 && gMaxX >= 0.0) {
        w2s(0.0, gMinY, &x0, &y0);
        w2s(0.0, gMaxY, &x1, &y1);
        line(x0, y0, x1, y1);
    }
}

static void drawTerrain(void)
{
    int i, x0, y0, x1, y1;
    int sx0, sy0, sx1, sy1;
    int poly[8];
    double dx, dy, L, nx, ny, tx, ty;

    for (i = 0; i < gPointCount - 1; i++) {
        w2s(gPoints[i].x, gPoints[i].y, &x0, &y0);
        w2s(gPoints[i + 1].x, gPoints[i + 1].y, &x1, &y1);
        dx = (double)(x1 - x0);
        dy = (double)(y1 - y0);
        L = sqrt(dx * dx + dy * dy);
        if (L < 1.0) continue;

        tx = dx / L;
        ty = dy / L;
        sx0 = x0;
        sy0 = y0;
        sx1 = x1;
        sy1 = y1;
        /* Extend only the route ends so both feet have a small landing area. */
        if (i == 0) {
            sx0 -= (int)(tx * 18.0);
            sy0 -= (int)(ty * 18.0);
        }
        if (i == gPointCount - 2) {
            sx1 += (int)(tx * 18.0);
            sy1 += (int)(ty * 18.0);
        }

        /* A narrow filled strip below the route line is the walking surface. */
        nx = -ty;
        ny = tx;
        poly[0] = sx0; poly[1] = sy0;
        poly[2] = sx1; poly[3] = sy1;
        poly[4] = sx1 + (int)(nx * 7.0); poly[5] = sy1 + (int)(ny * 7.0);
        poly[6] = sx0 + (int)(nx * 7.0); poly[7] = sy0 + (int)(ny * 7.0);
        setfillstyle(SOLID_FILL, BROWN);
        setcolor(BROWN);
        fillpoly(4, poly);
        setcolor(LIGHTGRAY);
        setlinestyle(SOLID_LINE, 0, THICK_WIDTH);
        line(sx0, sy0, sx1, sy1);
        setlinestyle(SOLID_LINE, 0, NORM_WIDTH);
    }
}

static void drawRouteMarkers(void)
{
    int i, x, y;
    char label[8];

    for (i = 0; i < gPointCount; i++) {
        w2s(gPoints[i].x, gPoints[i].y, &x, &y);
        if (i == 0) {
            disc(x, y, 5, LIGHTGREEN);
            txt(x + 7, y - 16, LIGHTGREEN, "START");
        } else {
            disc(x, y, 5, MAGENTA);
            sprintf(label, "%c", 'A' + i - 1);
            txt(x + 7, y - 16, MAGENTA, label);
        }
    }
}

static void drawTitle(void)
{
    txtc((PLOT_L + PLOT_R) / 2, 6, CYAN,
         "HUMANOID ROBOT - COORDINATE WALKING");
    setcolor(BLUE);
    line(PLOT_L, 20, PLOT_R, 20);
}

static void drawPanelFrame(void)
{
    setcolor(BLUE);
    rectangle(PANEL_L, PLOT_T, PANEL_R, PLOT_B);
    txt(PANEL_L + 8, PLOT_T + 6, CYAN, "CONTROL PANEL");
    setcolor(DARKGRAY);
    line(PANEL_L + 4, PLOT_T + 18, PANEL_R - 4, PLOT_T + 18);
    line(PANEL_L + 4, PLOT_T + 108, PANEL_R - 4, PLOT_T + 108);
}

static void refreshAll(void)
{
    gSaved = 0;
    cleardevice();
    drawGrid();
    drawAxes();
    drawTerrain();
    drawRouteMarkers();
    drawTitle();
    drawPanelFrame();
}

static void updateStatus(void)
{
    char s[48];
    int y;

    y = PLOT_T + 24;
    setfillstyle(SOLID_FILL, BLACK);
    bar(PANEL_L + 2, y - 2, PANEL_R - 2, y + 80);
    txt(PANEL_L + 8, y, LIGHTGRAY, "STATUS :");
    txt(PANEL_L + 70, y, LIGHTGREEN, gRouteReady ? "READY" : "NO ROUTE");

    sprintf(s, "POINTS : %d", gPointCount - 1);
    txt(PANEL_L + 8, y + 16, WHITE, s);
    sprintf(s, "POS : (%.1f,%.1f)", gCurX, gCurY);
    txt(PANEL_L + 8, y + 32, WHITE, s);
    if (gPointCount > 1) {
        sprintf(s, "END : (%d,%d)", (int)gPoints[gPointCount - 1].x,
                (int)gPoints[gPointCount - 1].y);
    } else {
        strcpy(s, "END : --");
    }
    txt(PANEL_L + 8, y + 48, MAGENTA, s);
    txt(PANEL_L + 8, y + 64, LIGHTGRAY, "DIR :");
    txt(PANEL_L + 70, y + 64, YELLOW, gDir > 0 ? "RIGHT" : "LEFT");
}

static void drawMenu(void)
{
    int y;

    y = PLOT_T + 116;
    setfillstyle(SOLID_FILL, BLACK);
    bar(PANEL_L + 2, y - 4, PANEL_R - 2, PLOT_B - 4);
    txt(PANEL_L + 8, y, CYAN, "MENU");
    txt(PANEL_L + 8, y + 18, LIGHTGRAY, "1. Generate new route");
    txt(PANEL_L + 8, y + 36, LIGHTGRAY, "2. Walk route again");
    txt(PANEL_L + 8, y + 54, LIGHTGRAY, "3. Return to origin");
    txt(PANEL_L + 8, y + 72, LIGHTGRAY, "4. Change speed");
    txt(PANEL_L + 8, y + 90, LIGHTGRAY, "5. Exit");
    txt(PANEL_L + 8, y + 116, DARKGRAY, "Press 1 - 5");
}

/* ------------------------ Keyboard input ----------------------------- */
static int readInt(const char *prompt, int maxdigits, int *ok)
{
    char buf[8];
    int n, digits, ch, x0, x1, y0;

    n = 0;
    digits = 0;
    *ok = 0;
    buf[0] = 0;
    clearMsg();
    txt(8, MSG_T + 8, YELLOW, prompt);
    x0 = 8 + textwidth((char *)prompt) + 8;
    x1 = x0 + (maxdigits + 1) * 8 + 10;
    y0 = MSG_T + 6;

    for (;;) {
        setfillstyle(SOLID_FILL, BLACK);
        bar(x0, y0, x1, y0 + 12);
        setcolor(WHITE);
        rectangle(x0, y0, x1, y0 + 12);
        buf[n] = 0;
        txt(x0 + 4, y0 + 3, WHITE, buf);
        setfillstyle(SOLID_FILL, YELLOW);
        bar(x0 + 4 + n * 8, y0 + 3, x0 + 9 + n * 8, y0 + 10);

        ch = getch();
        if (ch == 0 || ch == 224) {
            getch();
            continue;
        }
        if (ch == 13) {
            if (digits > 0) {
                *ok = 1;
                break;
            }
            continue;
        }
        if (ch == 27) {
            *ok = 0;
            return 0;
        }
        if (ch == 8) {
            if (n > 0) {
                if (buf[n - 1] != '-') digits--;
                n--;
            }
            continue;
        }
        if (ch == '-' && n == 0 && digits == 0) {
            buf[n++] = '-';
            continue;
        }
        if (ch >= '0' && ch <= '9' && digits < maxdigits && n < 6) {
            buf[n++] = (char)ch;
            digits++;
        }
    }
    buf[n] = 0;
    return atoi(buf);
}

static int askYesNo(const char *prompt)
{
    int ch;
    clearMsg();
    txt(8, MSG_T + 8, YELLOW, prompt);
    for (;;) {
        ch = getch();
        if (ch == 'y' || ch == 'Y') return 1;
        if (ch == 'n' || ch == 'N' || ch == 27) return 0;
    }
}

/* ----------------------- Robot animation ----------------------------- */
static void computeStandPose(Pose *p)
{
    int sx, sy;
    double angle;

    w2s(gCurX, gCurY, &sx, &sy);
    p->hipx = sx;
    p->hipy = sy - HIP_H;
    p->dir = gDir;
    angle = atan2(gLastTY, gDir * gLastTX);

    p->ankx[gLead] = sx + gLastTX * STAGGER;
    p->anky[gLead] = sy - ANKLE_H + gLastTY * STAGGER;
    p->ankx[1 - gLead] = sx - gLastTX * STAGGER;
    p->anky[1 - gLead] = sy - ANKLE_H - gLastTY * STAGGER;
    p->frot[0] = angle;
    p->frot[1] = angle;
    p->armA[0] = 0.10;
    p->armA[1] = -0.10;
    p->armE[0] = 0.30;
    p->armE[1] = 0.30;
    p->lean = 0.05;
    p->squash = 1.0;
    p->swapDepth = 0;
    p->headSquash = 1.0;
    p->headDir = gDir;
}

static void computeWalkPose(int fromIndex, int toIndex, double t, Pose *p)
{
    int x0, y0, x1, y1;
    double dx, dy, L, tx, ty, cx, cy;
    double u, leadOffset, trailOffset, leadLift, trailLift;
    double angle, phase;
    int lead, trail;

    w2s(gPoints[fromIndex].x, gPoints[fromIndex].y, &x0, &y0);
    w2s(gPoints[toIndex].x, gPoints[toIndex].y, &x1, &y1);
    dx = (double)(x1 - x0);
    dy = (double)(y1 - y0);
    L = sqrt(dx * dx + dy * dy);
    if (L < 1.0) L = 1.0;
    tx = dx / L;
    ty = dy / L;

    if (dx > 0.5) gDir = 1;
    else if (dx < -0.5) gDir = -1;
    gLastTX = tx;
    gLastTY = ty;

    u = smoothstep(t);
    cx = x0 + dx * u;
    cy = y0 + dy * u;
    lead = gLead;
    trail = 1 - gLead;

    leadOffset = -STAGGER + 2.0 * STAGGER * u;
    trailOffset = STAGGER - 2.0 * STAGGER * u;
    leadLift = 13.0 * sin(PI * t);
    trailLift = 0.0;
    if (t > 0.5) trailLift = 10.0 * sin(PI * (t - 0.5) * 2.0);

    p->hipx = cx;
    p->hipy = cy - HIP_H;
    p->dir = gDir;
    p->ankx[lead] = cx + tx * leadOffset + ty * leadLift;
    p->anky[lead] = cy - ANKLE_H + ty * leadOffset - tx * leadLift;
    p->ankx[trail] = cx + tx * trailOffset + ty * trailLift;
    p->anky[trail] = cy - ANKLE_H + ty * trailOffset - tx * trailLift;

    angle = atan2(ty, gDir * tx);
    p->frot[0] = angle;
    p->frot[1] = angle;
    phase = 2.0 * PI * t;
    p->armA[lead] = (lead == 0 ? 0.10 : -0.10) - 0.48 * sin(phase);
    p->armA[trail] = (trail == 0 ? 0.10 : -0.10) + 0.48 * sin(phase);
    p->armE[lead] = 0.30 + 0.20 * fabs(p->armA[lead]);
    p->armE[trail] = 0.30 + 0.20 * fabs(p->armA[trail]);
    p->lean = 0.06 + 0.07 * sin(PI * t);
    p->squash = 1.0;
    p->swapDepth = 0;
    p->headSquash = 1.0;
    p->headDir = gDir;
}

static void robotFrame(Pose *p)
{
    int x0, y0, x1, y1;

    if (gSaved) {
        putimage(gBoxX0, gBoxY0, gBuf, COPY_PUT);
        gSaved = 0;
    }
    x0 = (int)p->hipx - 54;
    x1 = (int)p->hipx + 66;
    y0 = (int)p->hipy - 90;
    y1 = (int)p->hipy + 66;
    if (x0 < PLOT_L) x0 = PLOT_L;
    if (x1 > PLOT_R) x1 = PLOT_R;
    if (y0 < PLOT_T) y0 = PLOT_T;
    if (y1 > PLOT_B) y1 = PLOT_B;
    if (x1 <= x0 || y1 <= y0) return;

    getimage(x0, y0, x1, y1, gBuf);
    gBoxX0 = x0;
    gBoxY0 = y0;
    gBoxX1 = x1;
    gBoxY1 = y1;
    gSaved = 1;
    drawFrame(p);
    gBlink++;
}

static void checkPause(void)
{
    int ch;
    if (kbhit()) {
        ch = getch();
        if (ch == 'p' || ch == 'P') {
            showMsg("PAUSED - press P to resume", YELLOW);
            do {
                ch = getch();
            } while (ch != 'p' && ch != 'P');
            showMsg("Walking...  Press P to pause", LIGHTGREEN);
        }
    }
}

static void animateSegment(int fromIndex, int toIndex)
{
    int f;
    Pose p;
    double t;

    showMsg("Robot walking along generated surface...", LIGHTGREEN);
    for (f = 0; f <= FRAMES_PER_SEGMENT; f++) {
        t = (double)f / (double)FRAMES_PER_SEGMENT;
        computeWalkPose(fromIndex, toIndex, t, &p);
        robotFrame(&p);
        checkPause();
        delay(gSpeedDelay);
    }
    gCurX = gPoints[toIndex].x;
    gCurY = gPoints[toIndex].y;
    gCurrentIndex = toIndex;
    gLead = 1 - gLead;
    computeStandPose(&p);
    robotFrame(&p);
    updateStatus();
}

static void returnToOrigin(void)
{
    int i;
    if (gCurrentIndex <= 0) {
        showMsg("Robot is already at the origin (0,0).", YELLOW);
        waitKey();
        return;
    }
    for (i = gCurrentIndex - 1; i >= 0; i--) {
        animateSegment(i + 1, i);
    }
    showMsg("Robot returned to the origin.", LIGHTGREEN);
    waitKey();
}

static void walkFullRoute(void)
{
    int i;
    if (!gRouteReady || gPointCount < 2) {
        showMsg("Enter a route first.", YELLOW);
        waitKey();
        return;
    }
    if (gCurrentIndex != 0) {
        for (i = gCurrentIndex - 1; i >= 0; i--) animateSegment(i + 1, i);
    }
    for (i = 0; i < gPointCount - 1; i++) animateSegment(i, i + 1);
    showMsg("Destination reached. Press any key for the menu.", LIGHTGREEN);
    waitKey();
}

/* ------------------------- Route input ------------------------------- */
static int enterNewRoute(void)
{
    int x, y, ok;
    char prompt[64];

    gPointCount = 1;
    gPoints[0].x = 0.0;
    gPoints[0].y = 0.0;
    gCurrentIndex = 0;
    gCurX = 0.0;
    gCurY = 0.0;
    gDir = 1;
    gLead = 1;
    gLastTX = 1.0;
    gLastTY = 0.0;
    gRouteReady = 0;
    computeScale();
    refreshAll();
    {
        Pose p;
        computeStandPose(&p);
        robotFrame(&p);
    }

    for (;;) {
        x = readInt("Destination X (-99 to 99): ", 3, &ok);
        if (!ok) {
            showMsg("Coordinate entry cancelled.", YELLOW);
            return 0;
        }
        if (x < -99 || x > 99) {
            showMsg("X must be between -99 and 99.", LIGHTRED);
            waitKey();
            continue;
        }
        y = readInt("Destination Y (-99 to 99): ", 3, &ok);
        if (!ok) {
            showMsg("Coordinate entry cancelled.", YELLOW);
            return 0;
        }
        if (y < -99 || y > 99) {
            showMsg("Y must be between -99 and 99.", LIGHTRED);
            waitKey();
            continue;
        }
        if (x == (int)gPoints[gPointCount - 1].x &&
            y == (int)gPoints[gPointCount - 1].y) {
            showMsg("This is the same as the previous point.", YELLOW);
            waitKey();
            continue;
        }

        gPoints[gPointCount].x = (double)x;
        gPoints[gPointCount].y = (double)y;
        gPointCount++;
        computeScale();
        refreshAll();
        sprintf(prompt, "Point %c added: (%d,%d)", 'A' + gPointCount - 2, x, y);
        showMsg(prompt, LIGHTGREEN);

        if (gPointCount >= MAX_POINTS) {
            showMsg("Maximum 20 waypoints reached.", YELLOW);
            waitKey();
            break;
        }
        if (!askYesNo("Add another coordinate? (Y/N): ")) break;
    }

    if (gPointCount < 2) {
        showMsg("No destination was entered.", YELLOW);
        waitKey();
        return 0;
    }

    gRouteReady = 1;
    gCurrentIndex = 0;
    gCurX = 0.0;
    gCurY = 0.0;
    gDir = 1;
    gLastTX = 1.0;
    gLastTY = 0.0;
    computeScale();
    refreshAll();
    updateStatus();
    {
        Pose p;
        computeStandPose(&p);
        robotFrame(&p);
    }
    showMsg("Path generated. Press any key to start walking.", YELLOW);
    getch();
    walkFullRoute();
    return 1;
}

static void changeSpeed(void)
{
    int value, ok;
    value = readInt("Frame delay in ms (10-80): ", 2, &ok);
    if (!ok) {
        showMsg("Speed change cancelled.", YELLOW);
        waitKey();
        return;
    }
    if (value < 10 || value > 80) {
        showMsg("Enter a delay from 10 to 80 ms.", LIGHTRED);
        waitKey();
        return;
    }
    gSpeedDelay = value;
    showMsg("Walking speed updated.", LIGHTGREEN);
    waitKey();
}

static int confirmExit(void)
{
    int ch;
    clearMsg();
    txt(8, MSG_T + 8, YELLOW, "Exit the simulation? (Y/N): ");
    ch = getch();
    if (ch == 'y' || ch == 'Y') return 1;
    return 0;
}

static void introScreen(void)
{
    cleardevice();
    setcolor(CYAN);
    rectangle(60, 100, 580, 380);
    txtc(320, 130, CYAN, "ARTICULATED HUMANOID ROBOT");
    txtc(320, 150, CYAN, "COORDINATE-BASED WALKING SIMULATION");
    txtc(320, 200, LIGHTGRAY, "Dynamic 2D walking-surface generation");
    txtc(320, 220, LIGHTGRAY, "Multiple waypoints + inverse kinematics");
    txtc(320, 240, LIGHTGRAY, "Signed X,Y coordinate input");
    txtc(320, 280, MAGENTA, "Example: (6,4), then (9,7)");
    txtc(320, 320, YELLOW, "Press any key to enter coordinates...");
    getch();
}

static void mainLoop(void)
{
    int ch;
    Pose p;

    refreshAll();
    computeStandPose(&p);
    robotFrame(&p);
    updateStatus();

    for (;;) {
        drawMenu();
        updateStatus();
        ch = getch();
        if (ch == 0 || ch == 224) {
            getch();
            continue;
        }
        if (ch == '1') {
            enterNewRoute();
            refreshAll();
            computeStandPose(&p);
            robotFrame(&p);
            updateStatus();
        } else if (ch == '2') {
            walkFullRoute();
            computeStandPose(&p);
            robotFrame(&p);
            updateStatus();
        } else if (ch == '3') {
            returnToOrigin();
            computeStandPose(&p);
            robotFrame(&p);
            updateStatus();
        } else if (ch == '4') {
            changeSpeed();
        } else if (ch == '5') {
            if (confirmExit()) break;
            refreshAll();
            computeStandPose(&p);
            robotFrame(&p);
            updateStatus();
        } else {
            showMsg("Invalid choice. Use 1-5.", LIGHTRED);
        }
    }
}

int main(void)
{
    int gd, gm;

    gd = DETECT;
    initgraph(&gd, &gm, "C:\\TURBOC3\\BGI");
    if (graphresult() != grOk) {
        printf("BGI graphics initialisation failed.\n");
        printf("Check that EGAVGA.BGI is in C:\\TURBOC3\\BGI.\n");
        return 1;
    }

    settextstyle(DEFAULT_FONT, HORIZ_DIR, 1);
    setbkcolor(BLACK);
    gPoints[0].x = 0.0;
    gPoints[0].y = 0.0;
    introScreen();
    enterNewRoute();
    mainLoop();
    closegraph();
    return 0;
}

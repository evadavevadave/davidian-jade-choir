/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* The parts of the amplifier that aren't plain circuits: the chorus/vibrato LFO (IC8), the
 * BBD delay and its clock (IC6 MN3007, IC7 MN3101), the JFET mute switches, the spring reverb
 * tank, and the oversampling filters around the preamp. */
#ifndef JD_FX_H
#define JD_FX_H

#include <stdlib.h>

#include "circuit.h"

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

/* ================= LFO: IC8a integrator and IC8b Schmitt trigger =================
 * IC8 runs on a single +15 V supply. R160 10k / R158 150 / R159 10k make two bias points:
 * IC8a (+) at 7.556 V, and 7.444 V for the RATE pot's lower end (through R155 15k, C93) and
 * the depth pot. IC8b's output (the square) drives the RATE pot VR8 100kB; the wiper feeds the
 * integrator (R166 1M, C95 0.082u) at MANUAL, the pot's lower end does at FIXED (chorus).
 * IC8b compares (27 * square + 39 * triangle) / 66 (R164 39k, R165 27k) with 7.444 V. */
#define LFO_VCC 15.0
#define LFO_BIAS_HI (LFO_VCC * (10e3 + 150.0) / (20e3 + 150.0))
#define LFO_BIAS_LO (LFO_VCC * 10e3 / (20e3 + 150.0))
#define LFO_SWING_LO 1.0           /* NJM4558 swing: within 1 V of each rail (datasheet typ) */
#define LFO_SWING_HI (LFO_VCC - 1.0)
#define LFO_R164 39e3
#define LFO_R165 27e3
#define LFO_R166 1e6
#define LFO_C95 0.082e-6
#define LFO_R155 15e3
#define LFO_VR8 100e3
#define LFO_VREF LFO_BIAS_LO       /* IC8b (-): see README on Q35/Q36 */

/* Triangle limits: where the Schmitt trigger flips at each square level. */
#define LFO_TRI_LO ((66.0 * LFO_VREF - 27.0 * LFO_SWING_HI) / 39.0)
#define LFO_TRI_HI ((66.0 * LFO_VREF - 27.0 * LFO_SWING_LO) / 39.0)

typedef struct {
    double tri; /* IC8a output, volts */
    int high;   /* IC8b output high */
    double T;
} Lfo;

static void lfo_init(Lfo* l, double fs)
{
    l->T = 1.0 / fs;
    l->tri = LFO_TRI_LO;
    l->high = 0;
}

/* rate: VR8 rotation (0-1); fixed: the FIXED (chorus) contact instead of the wiper. */
static double lfo_tick(Lfo* l, double rate, int fixed)
{
    const double vsq = l->high ? LFO_SWING_HI : LFO_SWING_LO;
    const double x = fixed ? 0.0 : clampd(rate, 0.0, 1.0);
    const double rtop = (1.0 - x) * LFO_VR8, rbot = x * LFO_VR8 + LFO_R155;
    const double vth = LFO_BIAS_LO + (vsq - LFO_BIAS_LO) * rbot / (rtop + rbot);
    const double rth = rtop * rbot / (rtop + rbot);
    const double i = (vth - LFO_BIAS_HI) / (rth + LFO_R166);
    double vp;
    l->tri = clampd(l->tri - i * l->T / LFO_C95, LFO_SWING_LO, LFO_SWING_HI);
    vp = (LFO_R165 * vsq + LFO_R164 * l->tri) / (LFO_R164 + LFO_R165);
    if (l->high && vp < LFO_VREF)
        l->high = 0;
    else if (!l->high && vp > LFO_VREF)
        l->high = 1;
    return l->tri;
}

/* Rate in Hz for a pot setting, from the same equations (for the display and the tests). */
static double lfo_rate_hz(double rate, int fixed)
{
    double t = 0.0;
    int k;
    for (k = 0; k < 2; ++k) {
        const double vsq = k ? LFO_SWING_HI : LFO_SWING_LO;
        const double x = fixed ? 0.0 : clampd(rate, 0.0, 1.0);
        const double rtop = (1.0 - x) * LFO_VR8, rbot = x * LFO_VR8 + LFO_R155;
        const double vth = LFO_BIAS_LO + (vsq - LFO_BIAS_LO) * rbot / (rtop + rbot);
        const double rth = rtop * rbot / (rtop + rbot);
        const double i = fabs(vth - LFO_BIAS_HI) / (rth + LFO_R166);
        t += (LFO_TRI_HI - LFO_TRI_LO) * LFO_C95 / i;
    }
    return 1.0 / t;
}

/* ================= BBD clock and delay =================
 * The schematic gives the MN3101 clock range as 80-350 kHz. The MN3007 has 1024 stages, so
 * its delay is 512 / f_clock: 6.40 ms at 80 kHz, 1.46 ms at 350 kHz. The triangle reaches the
 * clock through Q34 (emitter follower), R151 10k / C84 0.047u and D26; the full triangle
 * (as at FIXED, or DEPTH at full) spans the full clock range, higher voltage giving a higher
 * clock. The delay is taken as linear in the control voltage (see README). */
#define BBD_STAGES 1024
#define BBD_CLK_MIN 80e3
#define BBD_CLK_MAX 350e3
#define BBD_DELAY_MAX (BBD_STAGES / 2.0 / BBD_CLK_MIN)
#define BBD_DELAY_MIN (BBD_STAGES / 2.0 / BBD_CLK_MAX)
#define BBD_CV_TAU (10e3 * 0.047e-6) /* R151, C84 */
/* BBD voltage gain, from the schematic's signal levels: 760 mV at IC5a's output, 850 mV at
 * Q27's emitter. The output filter (C79 to Q27) passes 0.9536 at 1 kHz (test_dsp works this
 * out from its netlist), so the chip itself gives 850 / 760 / 0.9536. */
#define BBD_POST_1K 0.9536
#define BBD_GAIN (850.0 / 760.0 / BBD_POST_1K)

static double bbd_delay_for(double v)
{
    const double n = clampd((v - LFO_TRI_LO) / (LFO_TRI_HI - LFO_TRI_LO), 0.0, 1.0);
    return BBD_DELAY_MAX - n * (BBD_DELAY_MAX - BBD_DELAY_MIN);
}

typedef struct {
    float* buf;
    int size, mask, w;
    double fs, cv, cv_k;
} Bbd;

static int bbd_init(Bbd* b, double fs)
{
    int need = (int)(BBD_DELAY_MAX * fs) + 8;
    b->size = 1;
    while (b->size < need)
        b->size <<= 1;
    b->mask = b->size - 1;
    b->buf = (float*)calloc((size_t)b->size, sizeof(float));
    b->w = 0;
    b->fs = fs;
    b->cv = 0.5 * (LFO_TRI_LO + LFO_TRI_HI);
    b->cv_k = 1.0 - exp(-1.0 / (BBD_CV_TAU * fs));
    return b->buf != NULL;
}

static void bbd_reset(Bbd* b)
{
    memset(b->buf, 0, sizeof(float) * (size_t)b->size);
}

/* Writes x, returns the sample delayed by the clock that control voltage v sets (4-point
 * cubic interpolation). */
static double bbd_tick(Bbd* b, double x, double v)
{
    double d, frac, y0, y1, y2, y3, c1, c2, c3;
    int i;
    b->cv += b->cv_k * (v - b->cv);
    b->buf[b->w] = (float)x;
    d = bbd_delay_for(b->cv) * b->fs;
    i = (int)d;
    frac = d - i;
    y0 = b->buf[(b->w - i + 1) & b->mask];
    y1 = b->buf[(b->w - i) & b->mask];
    y2 = b->buf[(b->w - i - 1) & b->mask];
    y3 = b->buf[(b->w - i - 2) & b->mask];
    c1 = 0.5 * (y2 - y0);
    c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
    c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
    b->w = (b->w + 1) & b->mask;
    return BBD_GAIN * (((c3 * frac + c2) * frac + c1) * frac + y1);
}

/* ================= JFET switch gate (Q17, Q21) =================
 * Q18 and Q19 swing each gate network between the rails. The gate follows its capacitor
 * (C53 / C57, 0.047u, charged through R86 / R94 1M, plus R91 / R92 100k when pulled up)
 * through D17 / D21 (0.41 V at 15 uA), and the channel conducts in proportion to how far the
 * gate is above pinch-off. */
#define SW_C 0.047e-6
#define SW_R_DOWN 1e6
#define SW_R_UP 1.1e6
#define SW_DIODE 0.41
#define JFET_VP (-0.85) /* 2SK117 pinch-off, middle of the datasheet's -0.2 to -1.5 V */

typedef struct { double vc, k_up, k_down; } Gate;

static void gate_init(Gate* g, double fs, int on)
{
    g->k_up = 1.0 - exp(-1.0 / (SW_R_UP * SW_C * fs));
    g->k_down = 1.0 - exp(-1.0 / (SW_R_DOWN * SW_C * fs));
    g->vc = on ? 15.0 : -15.0;
}

/* Advances n samples toward on or off; returns the conduction 0-1. */
static double gate_tick(Gate* g, int on, int n)
{
    const double target = on ? 15.0 : -15.0;
    const double k = on ? g->k_up : g->k_down;
    double vgs;
    g->vc = target + (g->vc - target) * pow(1.0 - k, n);
    vgs = g->vc + SW_DIODE;
    if (vgs > 0.0)
        vgs = 0.0;
    return clampd((vgs - JFET_VP) / -JFET_VP, 0.0, 1.0);
}

static double gate_level(const Gate* g)
{
    double vgs = g->vc + SW_DIODE;
    if (vgs > 0.0)
        vgs = 0.0;
    return clampd((vgs - JFET_VP) / -JFET_VP, 0.0, 1.0);
}

/* ================= Spring reverb tank (three springs) =================
 * The tank is a mechanical part with no values on the schematic, and no published data was
 * found for it, so this is a generic three-spring model and its numbers are estimates (see
 * README): each spring is a delay loop with stretched all-pass dispersion (the "chirp"),
 * a loop low-pass, and a decay set by TANK_T60. */
#define TANK_SPRINGS 3
#define TANK_AP 24
#define TANK_T60 2.5
#define TANK_LP_HZ 4500.0
#define TANK_AP_COEF 0.62
#define TANK_GAIN 0.25
static const double TANK_TRANSIT[TANK_SPRINGS] = { 0.0295, 0.0337, 0.0371 }; /* seconds */

typedef struct {
    float* buf;
    int size, mask, w, d1, d2;
    double ap_x[TANK_AP][4], ap_y[TANK_AP][4]; /* stretched all-passes, up to 4 samples stretch */
    int stretch, ph;
    double g, lp, lp_k, dc_x, dc_y, norm;
} Spring;

typedef struct {
    Spring s[TANK_SPRINGS];
    double fs;
} Tank;

static int tank_init(Tank* t, double fs)
{
    int i;
    t->fs = fs;
    for (i = 0; i < TANK_SPRINGS; ++i) {
        Spring* s = &t->s[i];
        const int need = (int)(2.0 * TANK_TRANSIT[i] * fs) + 8;
        double loop;
        memset(s, 0, sizeof *s);
        s->size = 1;
        while (s->size < need)
            s->size <<= 1;
        s->mask = s->size - 1;
        s->buf = (float*)calloc((size_t)s->size, sizeof(float));
        if (!s->buf)
            return 0;
        s->d1 = (int)(TANK_TRANSIT[i] * fs);
        s->d2 = 2 * s->d1;
        s->stretch = (int)(fs / 24000.0 + 0.5);
        if (s->stretch < 1) s->stretch = 1;
        if (s->stretch > 4) s->stretch = 4;
        /* Loop time includes the all-passes' low-frequency group delay. */
        loop = s->d2 / fs + TANK_AP * s->stretch * (1.0 - TANK_AP_COEF) / (1.0 + TANK_AP_COEF) / fs;
        s->g = pow(10.0, -3.0 * loop / TANK_T60);
        s->lp_k = 1.0 - exp(-2.0 * M_PI * TANK_LP_HZ / fs);
        s->norm = sqrt(1.0 - s->g * s->g) / sqrt((double)TANK_SPRINGS);
    }
    return 1;
}

static void tank_free(Tank* t)
{
    int i;
    for (i = 0; i < TANK_SPRINGS; ++i) {
        free(t->s[i].buf);
        t->s[i].buf = NULL;
    }
}

static void tank_reset(Tank* t)
{
    int i;
    for (i = 0; i < TANK_SPRINGS; ++i) {
        Spring* s = &t->s[i];
        memset(s->buf, 0, sizeof(float) * (size_t)s->size);
        memset(s->ap_x, 0, sizeof s->ap_x);
        memset(s->ap_y, 0, sizeof s->ap_y);
        s->lp = s->dc_x = s->dc_y = 0.0;
    }
}

static double tank_tick(Tank* t, double x)
{
    double out = 0.0;
    int i, j;
    for (i = 0; i < TANK_SPRINGS; ++i) {
        Spring* s = &t->s[i];
        const int st = s->stretch;
        double fb = s->buf[(s->w - s->d2) & s->mask], y;
        /* Dispersion on each round trip: H(z) = (a + z^-k) / (1 + a z^-k). */
        for (j = 0; j < TANK_AP; ++j) {
            const int k = s->ph;
            const double yy = TANK_AP_COEF * (fb - s->ap_y[j][k]) + s->ap_x[j][k];
            s->ap_x[j][k] = fb;
            s->ap_y[j][k] = yy;
            fb = yy;
        }
        s->ph = (s->ph + 1) % st;
        s->lp += s->lp_k * (fb - s->lp);
        /* DC block in the loop */
        y = s->lp - s->dc_x + 0.995 * s->dc_y;
        s->dc_x = s->lp;
        s->dc_y = y;
        s->buf[s->w] = (float)(x + s->g * y);
        out += (i & 1 ? -1.0 : 1.0) * s->norm * s->buf[(s->w - s->d1) & s->mask];
        s->w = (s->w + 1) & s->mask;
    }
    return TANK_GAIN * out;
}

/* ================= Oversampling filters =================
 * 8th-order Chebyshev type I low-pass (0.1 dB ripple) at 0.45 of the base rate, run at twice
 * the base rate, for the 2x up- and down-sampling around the preamp's clipping stages. */
#define OS_SECTIONS 4

typedef struct { double b0, b1, b2, a1, a2, z1, z2; } Biquad;
typedef struct { Biquad s[OS_SECTIONS]; } OsFilter;

static void os_design(OsFilter* f, double fc, double fs)
{
    const int n = 2 * OS_SECTIONS;
    const double eps = sqrt(pow(10.0, 0.1 / 10.0) - 1.0);
    const double mu = asinh(1.0 / eps) / n;
    const double wc = 2.0 * fs * tan(M_PI * fc / fs);
    const double K = 2.0 * fs;
    int i;
    for (i = 0; i < OS_SECTIONS; ++i) {
        const double th = M_PI * (2.0 * (i + 1) - 1.0) / (2.0 * n);
        const double sg = -sinh(mu) * sin(th), om = cosh(mu) * cos(th);
        const double b1 = -2.0 * sg * wc, b0 = (sg * sg + om * om) * wc * wc;
        const double a0 = K * K + b1 * K + b0;
        Biquad* q = &f->s[i];
        q->b0 = b0 / a0;
        q->b1 = 2.0 * b0 / a0;
        q->b2 = b0 / a0;
        q->a1 = 2.0 * (b0 - K * K) / a0;
        q->a2 = (K * K - b1 * K + b0) / a0;
        q->z1 = q->z2 = 0.0;
    }
}

static void os_reset(OsFilter* f)
{
    int i;
    for (i = 0; i < OS_SECTIONS; ++i)
        f->s[i].z1 = f->s[i].z2 = 0.0;
}

static inline double os_tick(OsFilter* f, double x)
{
    int i;
    for (i = 0; i < OS_SECTIONS; ++i) {
        Biquad* q = &f->s[i];
        const double y = q->b0 * x + q->z1;
        q->z1 = q->b1 * x - q->a1 * y + q->z2;
        q->z2 = q->b2 * x - q->a2 * y;
        x = y;
    }
    return x;
}

#endif

/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* The parts of the amplifier that aren't plain circuits: the chorus/vibrato LFO, the BBD delay
 * and its clock (IC3 MN3007, IC4 MN3101), the JFET switch timing, the spring reverb tank, and
 * the oversampling filters around the preamps. */
#ifndef JD_FX_H
#define JD_FX_H

#include <stdlib.h>

#include "circuit.h"

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

/* ================= LFO (IC5) =================
 * The schematic prints the LFO's waveforms next to it, and the model follows them:
 *   CHORUS: a triangle, 9 V peak to peak, 1.2 s period.
 *   VIBRATO: SPEED (VR7 100kA) from a 500 ms period, 10 V p-p, to 120 ms, 13 V p-p, where the
 *   drawing shows the triangle clipped flat at the op amp's limits.
 * The speed law between the printed ends (geometric in rotation) and the growth of the
 * amplitude up to the clipping are estimates (see README). DEPTH (VR8 100kB) scales the
 * vibrato's triangle; CHORUS takes it whole (SW2(B)). */
#define LFO_CHORUS_PERIOD 1.2
#define LFO_CHORUS_PP 9.0
#define LFO_VIB_SLOW 0.5
#define LFO_VIB_FAST 0.12
#define LFO_VIB_PP_SLOW 10.0
#define LFO_VIB_PP_FAST 13.0
#define LFO_CLIP 6.5 /* IC5 on +15 V, biased at 7.5 V: 13 V p-p */

typedef struct {
    double phase; /* 0-1 */
    double T;
} Lfo;

static void lfo_init(Lfo* l, double fs)
{
    l->T = 1.0 / fs;
    l->phase = 0.0;
}

static double lfo_period(double speed, int vib)
{
    if (!vib)
        return LFO_CHORUS_PERIOD;
    return LFO_VIB_SLOW * pow(LFO_VIB_FAST / LFO_VIB_SLOW, clampd(speed, 0.0, 1.0));
}

/* Returns the LFO voltage about its bias: the triangle the BBD clock sees. */
static double lfo_tick(Lfo* l, double speed, double depth, int vib)
{
    double tri, amp;
    l->phase += l->T / lfo_period(speed, vib);
    l->phase -= floor(l->phase);
    tri = l->phase < 0.5 ? 4.0 * l->phase - 1.0 : 3.0 - 4.0 * l->phase; /* -1..1 */
    if (!vib)
        return 0.5 * LFO_CHORUS_PP * tri;
    /* The printed 13 V p-p at full speed is the clipped form of a larger triangle; it grows
     * from the printed 10 V p-p at the slow end. */
    amp = 0.5 * (LFO_VIB_PP_SLOW + 2.0 * (LFO_VIB_PP_FAST - LFO_VIB_PP_SLOW) * clampd(speed, 0.0, 1.0));
    return clampd(depth, 0.0, 1.0) * clampd(amp * tri, -LFO_CLIP, LFO_CLIP);
}

/* ================= BBD clock and delay =================
 * The schematic prints the clock's period in CHORUS as 2.5-12 us across the 9 V triangle. The
 * MN3007 has 1024 stages, so the delay is 512 clock periods: 1.28 ms to 6.14 ms. The clock
 * period is taken as linear in the LFO voltage (see README); vibrato's larger swing extends
 * it, held between 1.5 and 16 us. Q24 (follower), R139 15k / C74 1.5n smooth the control. */
#define BBD_STAGES 1024
#define BBD_T_MIN 2.5e-6
#define BBD_T_MAX 12e-6
#define BBD_T_LO 1.5e-6
#define BBD_T_HI 16e-6
#define BBD_DELAY_MAX (BBD_STAGES / 2.0 * BBD_T_HI)
#define BBD_CV_TAU (15e3 * 1.5e-9)
/* BBD voltage gain, from the schematic's levels: 940 mV into the BBD and 2.2 V at IC2a's
 * output, through the output filter and de-emphasis, which pass BBD_POST_1K at 1 kHz
 * (test_dsp works it out from their netlists). */
#define BBD_POST_1K 1.8400
#define BBD_GAIN (2200.0 / 940.0 / BBD_POST_1K)

static double bbd_delay_for(double v)
{
    const double mid = 0.5 * (BBD_T_MIN + BBD_T_MAX), half = 0.5 * (BBD_T_MAX - BBD_T_MIN);
    const double t = clampd(mid + half * v / (0.5 * LFO_CHORUS_PP), BBD_T_LO, BBD_T_HI);
    return BBD_STAGES / 2.0 * t;
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
    b->cv = 0.0;
    b->cv_k = 1.0 - exp(-1.0 / (BBD_CV_TAU * fs));
    return b->buf != NULL;
}

static void bbd_reset(Bbd* b)
{
    memset(b->buf, 0, sizeof(float) * (size_t)b->size);
}

/* Writes x, returns the sample delayed by the clock that LFO voltage v sets (4-point cubic
 * interpolation). */
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

/* ================= JFET switch gates (Q4, Q9, Q10) =================
 * Each 2SK246 switch's gate follows a 1M / 0.047u network (R71/C37, R73/C89, and C14 for Q4)
 * that Q12 or the footswitch swings between the rails, through a diode (0.41 V at 15 uA). The
 * channel conducts in proportion to how far the gate is above pinch-off, taken as -1.5 V
 * (the 2SK246 datasheet gives -0.7 to -6 V; estimate). So each switch fades over about 50 ms. */
#define SW_TAU (1e6 * 0.047e-6)
#define SW_DIODE 0.41
#define GATE_VP (-1.5)

typedef struct { double vc, k; } Gate;

static void gate_init(Gate* g, double fs, int on)
{
    g->k = 1.0 - exp(-1.0 / (SW_TAU * fs));
    g->vc = on ? 15.0 : -15.0;
}

static double gate_level(const Gate* g)
{
    double vgs = g->vc + SW_DIODE;
    if (vgs > 0.0)
        vgs = 0.0;
    return clampd((vgs - GATE_VP) / -GATE_VP, 0.0, 1.0);
}

/* Advances n samples toward on or off; returns the conduction 0-1. */
static double gate_tick(Gate* g, int on, int n)
{
    const double target = on ? 15.0 : -15.0;
    g->vc = target + (g->vc - target) * pow(1.0 - g->k, n);
    return gate_level(g);
}

/* ================= Spring reverb tank (three springs) =================
 * The original is Roland's Z-3F; its listed replacement, the Accutronics 8AB2D1A, is a short
 * (9-1/4") three-spring type 8 tank with medium decay (1.75-3.0 s). Its coils are in amp.h.
 * Between the coils the springs are modelled generically, and those numbers are estimates
 * (see README): each spring is a delay loop with stretched all-pass dispersion (the "chirp"),
 * a loop low-pass, and a decay of TANK_T60, the middle of the medium range. The input is the
 * drive coil's current and the output the pickup coil's voltage; TANK_GAIN (volts per amp) is
 * set so the reverb at full REVERB is about as loud as the dry signal. */
#define TANK_SPRINGS 3
#define TANK_AP 24
#define TANK_T60 2.375
#define TANK_LP_HZ 4500.0
#define TANK_AP_COEF 0.62
#define TANK_GAIN 0.05
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

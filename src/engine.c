/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* The Jade Choir engine: controls, routing and the per-sample loop. The LV2, VST3 and Audio
 * Unit wrappers all drive it through engine.h.
 *
 * Signal flow, as on the schematic:
 *   input jack -> IC2b -> IC2a + clippers -> tone stack -> volume -> IC3a -> IC3b
 *   IC3b -> IC1a (left mixer, "normal")                                  -> left line out
 *   IC3b -> Q17 switch -> IC4a (right mixer, "effect")                   -> right line out
 *   IC3b -> IC5a -> BBD -> filter, IC5b -> Q21 switch -> IC4a
 *   IC3b -> reverb driver -> tank -> IC4b -> REVERB pot -> both mixers
 * The preamp up to IC3b runs at twice the host rate below 88.2 kHz. */
#include <stdlib.h>

#include "amp.h"
#include "engine.h"
#include "fx.h"
#include "ports.h"

/* Denormal flushing: decaying signals would otherwise slow the CPU. x86 uses the SSE control
 * register (FTZ and DAZ bits), 64-bit ARM (Apple silicon, Linux arm64) the FPCR FZ bit. */
#if defined(_M_X64) || defined(__SSE2__)
#    include <xmmintrin.h>
#    define JD_HAS_CSR 1
typedef unsigned int fpstate;
static inline fpstate fp_flush_denormals(void)
{
    const unsigned int csr = _mm_getcsr();
    _mm_setcsr(csr | 0x8040);
    return csr;
}
static inline void fp_restore(fpstate s) { _mm_setcsr(s); }
#elif defined(__aarch64__) && (defined(__GNUC__) || defined(__clang__))
#    include <stdint.h>
#    define JD_HAS_CSR 1
typedef uint64_t fpstate;
static inline fpstate fp_flush_denormals(void)
{
    uint64_t fpcr;
    __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
    __asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr | (UINT64_C(1) << 24)));
    return fpcr;
}
static inline void fp_restore(fpstate s) { __asm__ __volatile__("msr fpcr, %0" : : "r"(s)); }
#endif

#define SUB 32                /* control updates and circuit rebuilds happen every SUB samples */
#define SMOOTH_SECONDS 0.015
#define BYPASS_SECONDS 0.02

/* Smoothed pots */
enum { K_VOL, K_TREBLE, K_MIDDLE, K_BASS, K_HITREBLE, K_DIST, K_REVERB, K_COUNT };

typedef struct JdEngine {
    const float* port[PORT_COUNT];

    double fs;
    int os;               /* preamp oversampling factor, 1 or 2 */
    double k_sub;         /* pot smoothing per SUB samples */
    double k_fast;        /* level smoothing per sample */
    double fade_step, fade;
    double knob[K_COUNT];
    double volts, gain;
    int count;            /* samples left in this SUB block */

    PreampSet pre;        /* as last built */
    MixSet mix;
    int built;

    Circuit c_in, c_drive, c_ic3a, c_ic3b, c_cpre, c_cpost, c_send, c_ret, c_mix, c_outl, c_outr;
    OsFilter up, down;
    Lfo lfo;
    Bbd bbd;
    Tank tank;
    Gate g_dry, g_wet;
    double cv;            /* BBD clock control voltage (held while the mode switch is at OFF) */
} Engine;

static double ctl(const Engine* e, int port, double lo, double hi, double fallback)
{
    const float* v = e->port[port];
    if (!v || !isfinite(*v))
        return fallback;
    return clampd(*v, lo, hi);
}

static int sel(const Engine* e, int port, int max, int fallback)
{
    return (int)(ctl(e, port, 0.0, max, fallback) + 0.5);
}

static void knob_targets(const Engine* e, double* t)
{
    t[K_VOL] = ctl(e, PORT_VOLUME, 0, 1, 0.4);
    t[K_TREBLE] = ctl(e, PORT_TREBLE, 0, 1, 0.5);
    t[K_MIDDLE] = ctl(e, PORT_MIDDLE, 0, 1, 0.5);
    t[K_BASS] = ctl(e, PORT_BASS, 0, 1, 0.5);
    t[K_HITREBLE] = ctl(e, PORT_HI_TREBLE, 0, 1, 0.0);
    t[K_DIST] = ctl(e, PORT_DISTORTION, 0, 1, 0.0);
    t[K_REVERB] = ctl(e, PORT_REVERB, 0, 1, 0.3);
}

static int effect_on(const Engine* e)
{
    return sel(e, PORT_MODE, 2, MODE_CHORUS) != MODE_OFF && ctl(e, PORT_FS_CHORUS, 0, 1, 1) > 0.5;
}

/* Rebuilds whatever the controls changed. force: everything. */
static void update_circuits(Engine* e, int force)
{
    const double rate = e->fs * e->os;
    PreampSet p;
    MixSet m;
    memset(&p, 0, sizeof p); /* compared with memcmp, so no stray padding */
    memset(&m, 0, sizeof m);
    p.volume = e->knob[K_VOL];
    p.treble = e->knob[K_TREBLE];
    p.middle = e->knob[K_MIDDLE];
    p.bass = e->knob[K_BASS];
    p.hitreble = e->knob[K_HITREBLE];
    p.distortion = e->knob[K_DIST];
    p.low_input = sel(e, PORT_INPUT_JACK, 1, 0);
    p.dist_fs = ctl(e, PORT_FS_DIST, 0, 1, 1) > 0.5;
    p.dist_switch = ctl(e, PORT_DISTORTION, 0, 1, 0) > 0.0;
    m.reverb = e->knob[K_REVERB];
    m.reverb_fs = ctl(e, PORT_FS_REVERB, 0, 1, 1) > 0.5;
    m.s_dry = gate_level(&e->g_dry);
    m.s_wet = gate_level(&e->g_wet);

    if (force || p.low_input != e->pre.low_input)
        build_input(&e->c_in, &p, rate);
    if (force || memcmp(&p, &e->pre, sizeof p) != 0)
        build_drive(&e->c_drive, &p, rate);
    if (force || memcmp(&m, &e->mix, sizeof m) != 0)
        build_mixer(&e->c_mix, &m, e->fs);
    e->pre = p;
    e->mix = m;
}

JdEngine* jd_engine_new(double sample_rate)
{
    Engine* e = (Engine*)calloc(1, sizeof(Engine));
    if (!e)
        return NULL;
    e->fs = sample_rate;
    e->os = sample_rate < 88200.0 ? 2 : 1;
    e->k_sub = 1.0 - exp(-(double)SUB / (SMOOTH_SECONDS * sample_rate));
    e->k_fast = 1.0 - exp(-1.0 / (SMOOTH_SECONDS * sample_rate));
    e->fade_step = 1.0 / (BYPASS_SECONDS * sample_rate);
    os_design(&e->up, 0.45 * sample_rate, sample_rate * e->os);
    os_design(&e->down, 0.45 * sample_rate, sample_rate * e->os);
    build_ic3a(&e->c_ic3a, sample_rate * e->os);
    build_ic3b(&e->c_ic3b, sample_rate * e->os);
    build_chorus_pre(&e->c_cpre, sample_rate);
    build_chorus_post(&e->c_cpost, sample_rate);
    build_send(&e->c_send, sample_rate);
    build_return(&e->c_ret, sample_rate);
    build_lineout(&e->c_outl, sample_rate);
    build_lineout(&e->c_outr, sample_rate);
    lfo_init(&e->lfo, sample_rate);
    gate_init(&e->g_dry, sample_rate, 0);
    gate_init(&e->g_wet, sample_rate, 1);
    if (!bbd_init(&e->bbd, sample_rate) || !tank_init(&e->tank, sample_rate)) {
        jd_engine_free(e);
        return NULL;
    }
    return e;
}

void jd_engine_connect(JdEngine* h, uint32_t port, void* data)
{
    Engine* e = (Engine*)h;
    if (port < PORT_COUNT)
        e->port[port] = (const float*)data;
}

static void reset_circuits(Engine* e)
{
    Circuit* all[] = { &e->c_in, &e->c_drive, &e->c_ic3a, &e->c_ic3b, &e->c_cpre, &e->c_cpost,
                       &e->c_send, &e->c_ret, &e->c_mix, &e->c_outl, &e->c_outr };
    size_t i;
    for (i = 0; i < sizeof all / sizeof all[0]; ++i)
        ck_reset(all[i]);
    os_reset(&e->up);
    os_reset(&e->down);
    bbd_reset(&e->bbd);
    tank_reset(&e->tank);
}

void jd_engine_activate(JdEngine* h)
{
    Engine* e = (Engine*)h;
    const int on = effect_on(e);
    knob_targets(e, e->knob);
    e->volts = ctl(e, PORT_VOLTS, 0.1, 10.0, 1.0);
    e->gain = pow(10.0, ctl(e, PORT_OUTPUT, -24.0, 12.0, 0.0) / 20.0);
    gate_init(&e->g_dry, e->fs, !on);
    gate_init(&e->g_wet, e->fs, on);
    lfo_init(&e->lfo, e->fs);
    e->cv = e->lfo.tri;
    e->bbd.cv = e->cv;
    reset_circuits(e);
    update_circuits(e, 1);
    e->count = SUB;
    e->fade = ctl(e, PORT_ENABLED, 0, 1, 1) > 0.5 ? 1.0 : 0.0;
}

/* Preamp, input jack to IC3b, at the oversampled rate. */
static inline double preamp_tick(Engine* e, double x)
{
    ck_step(&e->c_in, &x);
    x = ck_node(&e->c_in, IN_O);
    ck_step(&e->c_drive, &x);
    x = ck_node(&e->c_drive, DR_OUT);
    ck_step(&e->c_ic3a, &x);
    x = ck_node(&e->c_ic3a, A3_O);
    ck_step(&e->c_ic3b, &x);
    return ck_node(&e->c_ic3b, B3_O);
}

void jd_engine_run(JdEngine* h, uint32_t n)
{
    Engine* e = (Engine*)h;
    const float* in = e->port[PORT_IN];
    float* out_l = (float*)e->port[PORT_OUT_L];
    float* out_r = (float*)e->port[PORT_OUT_R];
    const int mode = sel(e, PORT_MODE, 2, MODE_CHORUS);
    const int on = effect_on(e);
    const int mixed = sel(e, PORT_LINE_OUT, 1, 0);
    const double speed = ctl(e, PORT_SPEED, 0, 1, 0.5);
    const double depth = ctl(e, PORT_DEPTH, 0, 1, 0.5);
    const double volts_to = ctl(e, PORT_VOLTS, 0.1, 10.0, 1.0);
    const double gain_to = pow(10.0, ctl(e, PORT_OUTPUT, -24.0, 12.0, 0.0) / 20.0);
    const double fade_to = ctl(e, PORT_ENABLED, 0, 1, 1) > 0.5 ? 1.0 : 0.0;
    double t[K_COUNT];
    uint32_t i;
    int k;
#ifdef JD_HAS_CSR
    fpstate csr;
#endif

    if (!in || !out_l || !out_r)
        return;
#ifdef JD_HAS_CSR
    csr = fp_flush_denormals();
#endif
    knob_targets(e, t);

    for (i = 0; i < n; ++i) {
        double x = in[i], y3, rev, wet, u[3], ol, or_, yl, yr, dry;
        if (!isfinite(x))
            x = 0.0;
        dry = x;

        if (--e->count <= 0) {
            e->count = SUB;
            for (k = 0; k < K_COUNT; ++k) {
                e->knob[k] += e->k_sub * (t[k] - e->knob[k]);
                if (fabs(t[k] - e->knob[k]) < 1e-5)
                    e->knob[k] = t[k];
            }
            gate_tick(&e->g_dry, !on, SUB);
            gate_tick(&e->g_wet, on, SUB);
            update_circuits(e, 0);
        }
        e->volts += e->k_fast * (volts_to - e->volts);
        e->gain += e->k_fast * (gain_to - e->gain);
        if (e->fade < fade_to)
            e->fade = fade_to < e->fade + e->fade_step ? fade_to : e->fade + e->fade_step;
        else if (e->fade > fade_to)
            e->fade = fade_to > e->fade - e->fade_step ? fade_to : e->fade - e->fade_step;

        /* Volts at the input jack. */
        x *= e->volts;
        if (e->os == 2) {
            const double a = preamp_tick(e, os_tick(&e->up, 2.0 * x));
            const double b = preamp_tick(e, os_tick(&e->up, 0.0));
            os_tick(&e->down, a);
            y3 = os_tick(&e->down, b);
        } else {
            y3 = preamp_tick(e, x);
        }

        /* Reverb: driver, tank, recovery amp. */
        ck_step(&e->c_send, &y3);
        u[0] = tank_tick(&e->tank, ck_node(&e->c_send, RS_A));
        ck_step(&e->c_ret, u);
        rev = ck_node(&e->c_ret, RR_O);

        /* Chorus / vibrato: the LFO holds still with the mode switch at OFF (S1c opens R166). */
        if (mode != MODE_OFF) {
            const double tri = lfo_tick(&e->lfo, speed, mode == MODE_CHORUS);
            e->cv = mode == MODE_CHORUS ? tri : LFO_BIAS_LO + depth * (tri - LFO_BIAS_LO);
        }
        ck_step(&e->c_cpre, &y3);
        u[0] = bbd_tick(&e->bbd, ck_node(&e->c_cpre, CP_O), e->cv);
        ck_step(&e->c_cpost, u);
        wet = ck_node(&e->c_cpost, CO_O);

        /* Mixers and line outs */
        u[0] = y3;
        u[1] = rev;
        u[2] = wet;
        ck_step(&e->c_mix, u);
        ol = clampd(ck_node(&e->c_mix, MX_OL), -OPAMP_SWING, OPAMP_SWING);
        or_ = clampd(ck_node(&e->c_mix, MX_OR), -OPAMP_SWING, OPAMP_SWING);
        ck_step(&e->c_outl, &ol);
        ck_step(&e->c_outr, &or_);
        yl = ck_node(&e->c_outl, LO_A);
        yr = ck_node(&e->c_outr, LO_A);
        if (mixed) /* NORMAL (MIXED) jack alone: both outputs through 5.6k each */
            yl = yr = 0.5 * (yl + yr);

        yl = yl / e->volts * e->gain;
        yr = yr / e->volts * e->gain;
        if (!isfinite(yl) || !isfinite(yr)) {
            reset_circuits(e);
            yl = yr = 0.0;
        }
        out_l[i] = (float)((1.0 - e->fade) * dry + e->fade * yl);
        out_r[i] = (float)((1.0 - e->fade) * dry + e->fade * yr);
    }

    if (e->port[PORT_LFO])
        *(float*)e->port[PORT_LFO] =
            (float)clampd((e->lfo.tri - LFO_TRI_LO) / (LFO_TRI_HI - LFO_TRI_LO), 0.0, 1.0);

#ifdef JD_HAS_CSR
    fp_restore(csr);
#endif
}

void jd_engine_free(JdEngine* h)
{
    Engine* e = (Engine*)h;
    if (!e)
        return;
    free(e->bbd.buf);
    tank_free(&e->tank);
    free(e);
}

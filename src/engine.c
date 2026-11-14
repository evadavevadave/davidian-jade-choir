/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* The Jade Choir engine: controls, routing and the per-sample loop. The LV2, VST3 and Audio
 * Unit wrappers all drive it through engine.h.
 *
 * Signal flow, as on the schematic:
 *   CH-1: input -> Q1 -> tone stack -> VOLUME/BRIGHT -> Q3 -> Q2 ------------> left and right
 *   CH-2: input -> Q1 -> tone stack -> Q2 -> (clean | Q3 distortion) -> VOLUME/BRIGHT -> Q5
 *         Q5 -> Q16/Q15 -> dry: left always; right when the effect is off (Q10)
 *         Q5 -> reverb driver -> tank -> IC1b -> REV -> left and right
 *         Q5 -> Q19 -> IC2b -> BBD -> filter, IC2a -> right when the effect is on (Q9)
 *   left: CH-1's power amp and speaker; right: CH-2's.
 * The preamps run at twice the host rate below 88.2 kHz. */
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
enum {
    K_VOL1, K_TRE1, K_MID1, K_BASS1, K_VOL2, K_TRE2, K_MID2, K_BASS2, K_DIST, K_REV, K_COUNT
};

/* Preamp outputs carried across the oversampling filters. */
enum { D_E1, D_P, D_REV, D_S19, D_COUNT };

typedef struct JdEngine {
    const float* port[PORT_COUNT];

    double fs;
    int os;
    double k_sub, k_fast;
    double fade_step, fade;
    double knob[K_COUNT];
    double volts, gain;
    int count;

    ChanSet c1, c2;
    DriveSet drv;
    MixSet mix;
    int rev_fs;

    Circuit in1, q3, in2, drive, q5, out2, rdrive, rret, cpre, cpost, mix_k, pwr_l, pwr_r;
    OsFilter up1, up2, down[D_COUNT];
    Lfo lfo;
    Bbd bbd;
    Tank tank;
    Gate g_dry, g_wet, g_q4;
    double lfo_v;
    long run1, run2;      /* samples each preamp keeps running after its input goes quiet */
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
    t[K_VOL1] = ctl(e, PORT_VOLUME1, 0, 1, 0.4);
    t[K_TRE1] = ctl(e, PORT_TREBLE1, 0, 1, 0.5);
    t[K_MID1] = ctl(e, PORT_MIDDLE1, 0, 1, 0.5);
    t[K_BASS1] = ctl(e, PORT_BASS1, 0, 1, 0.5);
    t[K_VOL2] = ctl(e, PORT_VOLUME2, 0, 1, 0.4);
    t[K_TRE2] = ctl(e, PORT_TREBLE2, 0, 1, 0.5);
    t[K_MID2] = ctl(e, PORT_MIDDLE2, 0, 1, 0.5);
    t[K_BASS2] = ctl(e, PORT_BASS2, 0, 1, 0.5);
    t[K_DIST] = ctl(e, PORT_DISTORTION, 0, 1, 0.0);
    t[K_REV] = ctl(e, PORT_REVERB, 0, 1, 0.3);
}

static int effect_on(const Engine* e)
{
    return sel(e, PORT_MODE, 2, MODE_CHORUS) != MODE_OFF && ctl(e, PORT_FS_CHORUS, 0, 1, 1) > 0.5;
}

static int dist_on(const Engine* e)
{
    /* DIST's switch opens at its OFF end; the footswitch turns Q4 off too. */
    return ctl(e, PORT_DISTORTION, 0, 1, 0) > 0.0 && ctl(e, PORT_FS_DIST, 0, 1, 1) > 0.5;
}

/* The settings the circuits are built from, from the smoothed knobs and the switches. */
static void settings(const Engine* e, ChanSet* c1, ChanSet* c2, DriveSet* d, MixSet* m, int* rev_fs)
{
    const int low = sel(e, PORT_INPUT_JACK, 1, 0);
    memset(c1, 0, sizeof *c1); /* compared with memcmp, so no stray padding */
    memset(c2, 0, sizeof *c2);
    memset(d, 0, sizeof *d);
    memset(m, 0, sizeof *m);
    c1->volume = e->knob[K_VOL1];
    c1->treble = e->knob[K_TRE1];
    c1->middle = e->knob[K_MID1];
    c1->bass = e->knob[K_BASS1];
    c1->bright = ctl(e, PORT_BRIGHT1, 0, 1, 0) > 0.5;
    c1->low_input = low;
    c2->volume = e->knob[K_VOL2];
    c2->treble = e->knob[K_TRE2];
    c2->middle = e->knob[K_MID2];
    c2->bass = e->knob[K_BASS2];
    c2->bright = ctl(e, PORT_BRIGHT2, 0, 1, 0) > 0.5;
    c2->low_input = low;
    d->volume = e->knob[K_VOL2];
    d->distortion = e->knob[K_DIST];
    d->bright = c2->bright;
    d->s_q4 = gate_level(&e->g_q4);
    m->reverb = e->knob[K_REV];
    m->s_dry = gate_level(&e->g_dry);
    m->s_wet = gate_level(&e->g_wet);
    *rev_fs = ctl(e, PORT_FS_REVERB, 0, 1, 1) > 0.5;
}

/* Rebuilds whatever the controls changed. */
static void update_circuits(Engine* e)
{
    const double rate = e->fs * e->os;
    ChanSet c1, c2;
    DriveSet d;
    MixSet m;
    int rev_fs;
    settings(e, &c1, &c2, &d, &m, &rev_fs);
    if (memcmp(&c1, &e->c1, sizeof c1))
        build_chan_in(&e->in1, &c1, 0, rate, 0);
    if (memcmp(&c2, &e->c2, sizeof c2))
        build_chan_in(&e->in2, &c2, 1, rate, 0);
    if (memcmp(&d, &e->drv, sizeof d))
        build_ch2_drive(&e->drive, &d, rate, 0.0, 0);
    if (rev_fs != e->rev_fs)
        build_ch2_q5(&e->q5, rev_fs, rate, 0.0, 0);
    if (memcmp(&m, &e->mix, sizeof m))
        build_mixer(&e->mix_k, &m, e->fs);
    e->c1 = c1;
    e->c2 = c2;
    e->drv = d;
    e->mix = m;
    e->rev_fs = rev_fs;
}

JdEngine* jd_engine_new(double sample_rate)
{
    Engine* e = (Engine*)calloc(1, sizeof(Engine));
    int i;
    if (!e)
        return NULL;
    e->fs = sample_rate;
    e->os = sample_rate < 88200.0 ? 2 : 1;
    e->k_sub = 1.0 - exp(-(double)SUB / (SMOOTH_SECONDS * sample_rate));
    e->k_fast = 1.0 - exp(-1.0 / (SMOOTH_SECONDS * sample_rate));
    e->fade_step = 1.0 / (BYPASS_SECONDS * sample_rate);
    os_design(&e->up1, 0.45 * sample_rate, sample_rate * e->os);
    os_design(&e->up2, 0.45 * sample_rate, sample_rate * e->os);
    for (i = 0; i < D_COUNT; ++i)
        os_design(&e->down[i], 0.45 * sample_rate, sample_rate * e->os);
    build_ch2_out(&e->out2, sample_rate * e->os);
    build_reverb_drive(&e->rdrive, sample_rate);
    build_reverb_return(&e->rret, sample_rate);
    build_chorus_pre(&e->cpre, sample_rate);
    build_chorus_post(&e->cpost, sample_rate);
    build_power(&e->pwr_l, sample_rate);
    build_power(&e->pwr_r, sample_rate);
    lfo_init(&e->lfo, sample_rate);
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

void jd_engine_activate(JdEngine* h)
{
    Engine* e = (Engine*)h;
    const double rate = e->fs * e->os;
    const int on = effect_on(e);
    Circuit* ac[] = { &e->out2, &e->rdrive, &e->rret, &e->cpre, &e->cpost, &e->pwr_l, &e->pwr_r };
    double g;
    size_t i;
    knob_targets(e, e->knob);
    e->volts = ctl(e, PORT_VOLTS, 0.1, 10.0, 1.0);
    e->gain = pow(10.0, ctl(e, PORT_OUTPUT, -24.0, 12.0, 0.0) / 20.0);
    gate_init(&e->g_dry, e->fs, !on);
    gate_init(&e->g_wet, e->fs, on);
    gate_init(&e->g_q4, e->fs, dist_on(e));
    lfo_init(&e->lfo, e->fs);
    e->lfo_v = 0.0;
    e->bbd.cv = 0.0;
    settings(e, &e->c1, &e->c2, &e->drv, &e->mix, &e->rev_fs);

    /* Each JFET stage starts at its DC operating point, fed by the one before. */
    build_chan_in(&e->in1, &e->c1, 0, rate, 1);
    g = ck_node(&e->in1, IN_OUT);
    build_ch1_q3(&e->q3, rate, g, 1);
    build_chan_in(&e->in2, &e->c2, 1, rate, 1);
    g = ck_node(&e->in2, IN_OUT);
    build_ch2_drive(&e->drive, &e->drv, rate, g, 1);
    g = ck_node(&e->drive, DR_OUT);
    build_ch2_q5(&e->q5, e->rev_fs, rate, g, 1);
    for (i = 0; i < sizeof ac / sizeof ac[0]; ++i)
        ck_reset(ac[i]);
    /* The stages fed DC-coupled from a JFET stage start charged to that DC too, so nothing
     * thumps. */
    {
        const double mix_dc[4] = { ck_node(&e->q3, Q3_E), 0.0, 0.0, 0.0 };
        const double s19 = ck_node(&e->q5, Q5_S19);
        build_mixer(&e->mix_k, &e->mix, e->fs);
        ck_settle(&e->mix_k, e->fs, mix_dc);
        ck_settle(&e->cpre, e->fs, &s19);
    }

    /* The oversampling filters start settled on the preamps' DC outputs. */
    os_reset(&e->up1);
    os_reset(&e->up2);
    for (i = 0; i < D_COUNT; ++i)
        os_reset(&e->down[i]);
    {
        const double dc[D_COUNT] = { ck_node(&e->q3, Q3_E), 0.0, 0.0, ck_node(&e->q5, Q5_S19) };
        int n, k;
        for (n = 0; n < 2000; ++n)
            for (k = 0; k < D_COUNT; ++k)
                os_tick(&e->down[k], dc[k]);
    }
    bbd_reset(&e->bbd);
    tank_reset(&e->tank);
    e->count = SUB;
    e->run1 = e->run2 = 0;
    e->fade = ctl(e, PORT_ENABLED, 0, 1, 1) > 0.5 ? 1.0 : 0.0;
}

/* Both preamps, input to the four signals the rest of the amp takes, at the oversampled
 * rate. A channel with nothing at its input stays at its DC operating point, so it is only
 * stepped when it is fed (use1, use2); otherwise its outputs hold. */
static inline void preamp_tick(Engine* e, double x1, double x2, int use1, int use2, double* out)
{
    double v;
    if (use1) {
        ck_step(&e->in1, &x1);
        v = ck_node(&e->in1, IN_OUT);
        ck_step(&e->q3, &v);
    }
    out[D_E1] = ck_node(&e->q3, Q3_E);

    if (!use2) {
        out[D_REV] = ck_node(&e->q5, Q5_REV);
        out[D_S19] = ck_node(&e->q5, Q5_S19);
        out[D_P] = ck_node(&e->out2, OA_O);
        return;
    }
    ck_step(&e->in2, &x2);
    v = ck_node(&e->in2, IN_OUT);
    ck_step(&e->drive, &v);
    v = ck_node(&e->drive, DR_OUT);
    ck_step(&e->q5, &v);
    out[D_REV] = ck_node(&e->q5, Q5_REV);
    out[D_S19] = ck_node(&e->q5, Q5_S19);
    v = ck_node(&e->q5, Q5_G16);
    ck_step(&e->out2, &v);
    out[D_P] = ck_node(&e->out2, OA_O);
}

void jd_engine_run(JdEngine* h, uint32_t n)
{
    Engine* e = (Engine*)h;
    const float* in = e->port[PORT_IN];
    float* out_l = (float*)e->port[PORT_OUT_L];
    float* out_r = (float*)e->port[PORT_OUT_R];
    const int chan = sel(e, PORT_CHANNEL, 2, CHAN_2);
    const int mode = sel(e, PORT_MODE, 2, MODE_CHORUS);
    const int on = effect_on(e);
    const int dist = dist_on(e);
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
        double x = in[i], dry, sig[D_COUNT], u[4], wet, rev, yl, yr, gl, gr;
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
            gate_tick(&e->g_q4, dist, SUB);
            update_circuits(e);
        }
        e->volts += e->k_fast * (volts_to - e->volts);
        e->gain += e->k_fast * (gain_to - e->gain);
        if (e->fade < fade_to)
            e->fade = fade_to < e->fade + e->fade_step ? fade_to : e->fade + e->fade_step;
        else if (e->fade > fade_to)
            e->fade = fade_to > e->fade - e->fade_step ? fade_to : e->fade - e->fade_step;

        /* Volts at the input jack(s). */
        x *= e->volts;
        {
            const double x1 = chan != CHAN_2 ? x : 0.0, x2 = chan != CHAN_1 ? x : 0.0;
            double o[D_COUNT];
            int use1, use2;
            if (chan != CHAN_2)
                e->run1 = (long)(2.0 * e->fs);
            if (chan != CHAN_1)
                e->run2 = (long)(2.0 * e->fs);
            use1 = e->run1 > 0;
            use2 = e->run2 > 0;
            if (e->run1 > 0)
                --e->run1;
            if (e->run2 > 0)
                --e->run2;
            if (e->os == 2) {
                const double a1 = os_tick(&e->up1, 2.0 * x1), a2 = os_tick(&e->up2, 2.0 * x2);
                const double b1 = os_tick(&e->up1, 0.0), b2 = os_tick(&e->up2, 0.0);
                preamp_tick(e, a1, a2, use1, use2, o);
                for (k = 0; k < D_COUNT; ++k)
                    os_tick(&e->down[k], o[k]);
                preamp_tick(e, b1, b2, use1, use2, o);
                for (k = 0; k < D_COUNT; ++k)
                    sig[k] = os_tick(&e->down[k], o[k]);
            } else {
                preamp_tick(e, x1, x2, use1, use2, sig);
            }
        }

        /* Reverb: driver and coils, springs, recovery. */
        ck_step(&e->rdrive, &sig[D_REV]);
        u[0] = tank_tick(&e->tank, ck_node(&e->rdrive, RD_SN) / SENSE_R);
        ck_step(&e->rret, u);
        rev = ck_node(&e->rret, RR_O);

        /* Chorus / vibrato */
        e->lfo_v = lfo_tick(&e->lfo, speed, depth, mode == MODE_VIB);
        ck_step(&e->cpre, &sig[D_S19]);
        u[0] = bbd_tick(&e->bbd, ck_node(&e->cpre, CP_O), e->lfo_v);
        ck_step(&e->cpost, u);
        wet = ck_node(&e->cpost, CO_O);

        /* Mixer, MAIN IN and the power amps */
        u[0] = sig[D_E1];
        u[1] = sig[D_P];
        u[2] = rev;
        u[3] = wet;
        ck_step(&e->mix_k, u);
        gl = ck_node(&e->mix_k, MX_GL);
        gr = ck_node(&e->mix_k, MX_GR);
        ck_step(&e->pwr_l, &gl);
        ck_step(&e->pwr_r, &gr);
        yl = ck_node(&e->pwr_l, PA_O);
        yr = ck_node(&e->pwr_r, PA_O);

        /* Speaker volts to samples: full scale is the power amp's clipping level. */
        yl = yl / POWER_CLIP * e->gain;
        yr = yr / POWER_CLIP * e->gain;
        if (!isfinite(yl) || !isfinite(yr)) {
            jd_engine_activate(h);
            yl = yr = 0.0;
        }
        out_l[i] = (float)((1.0 - e->fade) * dry + e->fade * yl);
        out_r[i] = (float)((1.0 - e->fade) * dry + e->fade * yr);
    }

    if (e->port[PORT_LFO])
        *(float*)e->port[PORT_LFO] = (float)clampd((e->lfo_v + LFO_CLIP) / (2.0 * LFO_CLIP), 0.0, 1.0);

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

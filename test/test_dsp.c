/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* Checks for the Jade Choir. Usage: test_dsp <bundle folder>
 * Part 1 drives the circuit code directly: each stage's time-domain solver against an
 * independent AC (complex phasor) analysis of the same netlist, the stages against the signal
 * levels printed on the schematic, and the LFO and BBD against the schematic's figures.
 * Part 2 loads the built plugin library the way a host does. */
#ifndef _CRT_SECURE_NO_WARNINGS
#    define _CRT_SECURE_NO_WARNINGS
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../src/amp.h"
#include "../src/fx.h"
#include "../src/lv2_abi.h"
#include "../src/ports.h"

#ifdef _WIN32
#    include <windows.h>
#    define LIB_NAME "jade_choir.dll"
#    define LIB_OPEN(p) ((void*)LoadLibraryA(p))
#    define LIB_SYM(l, s) ((void*)GetProcAddress((HMODULE)(l), s))
#else
#    include <dlfcn.h>
#    ifdef __APPLE__
#        define LIB_NAME "jade_choir.dylib"
#    else
#        define LIB_NAME "jade_choir.so"
#    endif
#    define LIB_OPEN(p) dlopen(p, RTLD_NOW)
#    define LIB_SYM(l, s) dlsym(l, s)
#endif

#define PI 3.14159265358979323846

static int failures;

static void check(const char* what, int ok)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        ++failures;
}

/* ---- AC analysis: the same netlist as complex admittances, solved directly ---- */

typedef struct { double re, im; } Cx;

static Cx cx(double r, double i) { Cx c; c.re = r; c.im = i; return c; }
static Cx cadd(Cx a, Cx b) { return cx(a.re + b.re, a.im + b.im); }
static Cx csub(Cx a, Cx b) { return cx(a.re - b.re, a.im - b.im); }
static Cx cmul(Cx a, Cx b) { return cx(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re); }
static Cx cdiv(Cx a, Cx b)
{
    const double d = b.re * b.re + b.im * b.im;
    return cx((a.re * b.re + a.im * b.im) / d, (a.im * b.re - a.re * b.im) / d);
}
static double cmag(Cx a) { return sqrt(a.re * a.re + a.im * a.im); }

static void ac_g(Cx A[CK_MAX_DIM][CK_MAX_DIM], int a, int b, Cx y)
{
    if (a) A[a - 1][a - 1] = cadd(A[a - 1][a - 1], y);
    if (b) A[b - 1][b - 1] = cadd(A[b - 1][b - 1], y);
    if (a && b) {
        A[a - 1][b - 1] = csub(A[a - 1][b - 1], y);
        A[b - 1][a - 1] = csub(A[b - 1][a - 1], y);
    }
}

/* Node voltage at frequency f with source `which` at 1 V and the others at 0. Diodes are left
 * out (at the millivolt levels used here they pass well under a nanoamp). */
static Cx ac_node(const Circuit* k, double f, int which, int node)
{
    static Cx A[CK_MAX_DIM][CK_MAX_DIM];
    Cx b[CK_MAX_DIM];
    const double w = 2.0 * PI * f;
    const int n = k->dim;
    int i, j, c, row;
    memset(A, 0, sizeof A);
    memset(b, 0, sizeof b);
    for (i = 0; i < k->nr; ++i)
        ac_g(A, k->r[i].a, k->r[i].b, cx(1.0 / k->r[i].r, 0.0));
    for (i = 0; i < k->nc; ++i)
        ac_g(A, k->c[i].a, k->c[i].b, cx(0.0, w * k->c[i].c));
    for (i = 0; i < k->nsrc; ++i) {
        row = CK_SRC_ROW(k, i);
        A[row][k->src[i] - 1] = cx(1, 0);
        A[k->src[i] - 1][row] = cx(1, 0);
        b[row] = cx(i == which ? 1.0 : 0.0, 0.0);
    }
    if (k->op_o) {
        row = CK_OP_ROW(k);
        if (k->op_p) A[row][k->op_p - 1] = cx(1, 0);
        if (k->op_m) A[row][k->op_m - 1] = cx(-1, 0);
        A[k->op_o - 1][row] = cx(1, 0);
    }
    if (k->op2_o) {
        row = CK_OP2_ROW(k);
        if (k->op2_p) A[row][k->op2_p - 1] = cx(1, 0);
        if (k->op2_m) A[row][k->op2_m - 1] = cx(-1, 0);
        A[k->op2_o - 1][row] = cx(1, 0);
    }
    if (k->buf_out) {
        row = CK_BUF_ROW(k);
        A[row][k->buf_out - 1] = cx(1, 0);
        A[row][k->buf_in - 1] = cx(-1, 0);
        A[k->buf_out - 1][row] = cx(1, 0);
    }
    for (c = 0; c < n; ++c) {
        int p = c;
        for (i = c + 1; i < n; ++i)
            if (cmag(A[i][c]) > cmag(A[p][c]))
                p = i;
        if (p != c) {
            for (j = 0; j < n; ++j) {
                Cx t = A[c][j];
                A[c][j] = A[p][j];
                A[p][j] = t;
            }
            {
                Cx t = b[c];
                b[c] = b[p];
                b[p] = t;
            }
        }
        for (i = c + 1; i < n; ++i) {
            const Cx m = cdiv(A[i][c], A[c][c]);
            for (j = c; j < n; ++j)
                A[i][j] = csub(A[i][j], cmul(m, A[c][j]));
            b[i] = csub(b[i], cmul(m, b[c]));
        }
    }
    for (i = n - 1; i >= 0; --i) {
        Cx s = b[i];
        for (j = i + 1; j < n; ++j)
            s = csub(s, cmul(A[i][j], b[j]));
        b[i] = cdiv(s, A[i][i]);
    }
    return b[node - 1];
}

/* ---- Time-domain measurement: a sine through the solver, amplitude and phase of a node ---- */

static Cx td_node(Circuit* k, double fs, double f, double amp, int which, int node)
{
    const int settle = (int)(fs * 4.0), len = (int)(fs / f * 20.0 + 0.5); /* C18: 10u into up to 51k */
    double u[CK_MAX_SRC] = { 0 }, sr = 0.0, si = 0.0;
    int n;
    ck_reset(k);
    for (n = 0; n < settle + len; ++n) {
        const double ph = 2.0 * PI * f * n / fs;
        u[which] = amp * sin(ph);
        ck_step(k, u);
        if (n >= settle) {
            const double v = ck_node(k, node);
            sr += v * sin(ph);
            si += v * cos(ph);
        }
    }
    return cx(2.0 * sr / len / amp, 2.0 * si / len / amp);
}

static double td_rms(Circuit* k, double fs, double f, double rms_in, int which, int node)
{
    return cmag(td_node(k, fs, f, rms_in * sqrt(2.0), which, node)) * rms_in;
}

static int compare_ac(Circuit* k, double fs, int which, int node, const char* name, double* worst_db)
{
    static const double freqs[] = { 40.0, 100.0, 300.0, 1000.0, 3000.0, 6000.0 };
    int i, ok = 1;
    for (i = 0; i < (int)(sizeof freqs / sizeof freqs[0]); ++i) {
        /* The trapezoidal rule is the bilinear transform: the solver at f matches the circuit at
         * fs / pi * tan(pi f / fs). */
        const Cx a = ac_node(k, fs / PI * tan(PI * freqs[i] / fs), which, node);
        const Cx t = td_node(k, fs, freqs[i], 1e-3, which, node);
        const double ma = cmag(a), mt = cmag(t);
        const double db = 20.0 * log10((mt + 1e-30) / (ma + 1e-30));
        if (ma > 1e-4 && fabs(db) > *worst_db)
            *worst_db = fabs(db);
        if (ma > 1e-4 && fabs(db) > 0.01) {
            printf("    %s at %.0f Hz: solver %.4f, AC %.4f (%.3f dB)\n", name, freqs[i], mt, ma, db);
            ok = 0;
        }
    }
    return ok;
}

static void test_against_ac(void)
{
    const double fs = 96000.0;
    static Circuit k;
    char label[200];
    PreampSet p;
    MixSet m;
    double worst = 0.0;
    int ok = 1, i;
    static const double pots[][6] = {
        /* volume, treble, middle, bass, hi-treble, distortion */
        { 1.0, 1.0, 1.0, 1.0, 1.0, 0.0 },
        { 0.5, 0.5, 0.5, 0.5, 0.0, 0.0 },
        { 0.3, 0.0, 1.0, 0.2, 0.7, 0.0 },
        { 0.8, 0.9, 0.1, 1.0, 0.3, 0.6 },
        { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 },
    };

    memset(&p, 0, sizeof p);
    p.dist_fs = 1;
    build_input(&k, &p, fs);
    ok &= compare_ac(&k, fs, 0, IN_O, "input, HIGH", &worst);
    p.low_input = 1;
    build_input(&k, &p, fs);
    ok &= compare_ac(&k, fs, 0, IN_O, "input, LOW", &worst);
    for (i = 0; i < (int)(sizeof pots / sizeof pots[0]); ++i) {
        p.volume = pots[i][0];
        p.treble = pots[i][1];
        p.middle = pots[i][2];
        p.bass = pots[i][3];
        p.hitreble = pots[i][4];
        p.distortion = pots[i][5];
        p.dist_switch = p.distortion > 0.0;
        p.dist_fs = i != 2;
        build_drive(&k, &p, fs);
        /* The AC analysis is of the linear network, so the clipping diodes come out here (at a
         * few millivolts they still conduct enough to move the 40 Hz response by 0.04 dB). */
        k.nd = 0;
        ck_build(&k, fs);
        ok &= compare_ac(&k, fs, 0, DR_OUT, "IC2a, clippers, tone stack, volume", &worst);
    }
    build_ic3a(&k, fs);
    ok &= compare_ac(&k, fs, 0, A3_O, "IC3a", &worst);
    build_ic3b(&k, fs);
    ok &= compare_ac(&k, fs, 0, B3_O, "IC3b", &worst);
    build_chorus_pre(&k, 48000.0);
    ok &= compare_ac(&k, 48000.0, 0, CP_O, "chorus input filter", &worst);
    build_chorus_post(&k, 48000.0);
    ok &= compare_ac(&k, 48000.0, 0, CO_O, "chorus output filter", &worst);
    build_return(&k, 48000.0);
    ok &= compare_ac(&k, 48000.0, 0, RR_O, "reverb recovery", &worst);
    memset(&m, 0, sizeof m);
    m.reverb = 0.6;
    m.reverb_fs = 1;
    m.s_dry = 0.0;
    m.s_wet = 1.0;
    build_mixer(&k, &m, 48000.0);
    for (i = 0; i < 3; ++i) {
        ok &= compare_ac(&k, 48000.0, i, MX_OL, "left mixer", &worst);
        ok &= compare_ac(&k, 48000.0, i, MX_OR, "right mixer", &worst);
    }
    build_lineout(&k, 48000.0);
    ok &= compare_ac(&k, 48000.0, 0, LO_A, "line out", &worst);
    snprintf(label, sizeof label, "every stage matches its AC analysis, 40 Hz-6 kHz (worst %.4f dB)", worst);
    check(label, ok);
}

/* ---- The schematic's printed signal levels (1 kHz, rms) ---- */

static void level_check(const char* what, double got_mv, double want_mv, double tol)
{
    char label[200];
    snprintf(label, sizeof label, "%s: %.0f mV (schematic %.0f mV)", what, got_mv, want_mv);
    check(label, fabs(got_mv / want_mv - 1.0) <= tol);
}

static void test_levels(void)
{
    const double fs = 96000.0;
    static Circuit k;
    PreampSet p;
    double v;


    memset(&p, 0, sizeof p);
    p.dist_fs = 1;
    build_input(&k, &p, fs);
    level_check("HIGH input 21 mV, IC2b output", 1e3 * td_rms(&k, fs, 1000.0, 0.021, 0, IN_O), 72.0, 0.05);
    p.low_input = 1;
    build_input(&k, &p, fs);
    level_check("LOW input 85 mV, IC2b output", 1e3 * td_rms(&k, fs, 1000.0, 0.085, 0, IN_O), 72.0, 0.05);

    /* Distortion at OFF (fully anticlockwise), equalizer and hi-treble fully clockwise. */
    p.low_input = 0;
    p.treble = p.middle = p.bass = p.hitreble = 1.0;
    p.volume = 1.0;
    build_drive(&k, &p, fs);
    level_check("IC2a output after C30, distortion OFF", 1e3 * td_rms(&k, fs, 1000.0, 0.072, 0, DR_X), 130.0, 0.08);

    /* The rest of the test conditions (VOLUME among them) are cut off the scan. With VOLUME
     * fully clockwise too, the chain from IC2b to IC3b gives: */
    {
        static Circuit a, b;
        p.volume = 1.0;
        build_drive(&k, &p, fs);
        build_ic3a(&a, fs);
        build_ic3b(&b, fs);
        v = cmag(ac_node(&k, 1000.0, 0, DR_OUT)) * cmag(ac_node(&a, 1000.0, 0, A3_O))
            * cmag(ac_node(&b, 1000.0, 0, B3_O)) * 0.072;
        level_check("IC3b output, VOLUME also fully clockwise", 1e3 * v, 290.0, 0.10);
    }

    build_chorus_pre(&k, 48000.0);
    level_check("IC3b 290 mV, IC5a output (into the BBD)", 1e3 * td_rms(&k, 48000.0, 1000.0, 0.290, 0, CP_O), 760.0, 0.10);
    build_chorus_post(&k, 48000.0);
    printf("  (output filter passes %.4f at 1 kHz, C79 to Q27's emitter)\n", cmag(ac_node(&k, 1000.0, 0, CO_E)));
    level_check("BBD 760 mV x gain, Q27 emitter", 1e3 * td_rms(&k, 48000.0, 1000.0, 0.760 * BBD_GAIN, 0, CO_E), 850.0, 0.05);
    level_check("BBD 760 mV x gain, IC5b output", 1e3 * td_rms(&k, 48000.0, 1000.0, 0.760 * BBD_GAIN, 0, CO_O), 300.0, 0.10);
}

/* ---- LFO and BBD ---- */

static double measure_lfo_hz(double rate, int fixed, double fs)
{
    Lfo l;
    int n, flips = 0, first = -1, last = -1, prev;
    lfo_init(&l, fs);
    prev = l.high;
    for (n = 0; n < (int)(fs * 30.0); ++n) {
        lfo_tick(&l, rate, fixed);
        if (l.high && !prev) {
            if (first < 0)
                first = n;
            last = n;
            ++flips;
        }
        prev = l.high;
    }
    return flips > 1 ? (flips - 1) * fs / (last - first) : 0.0;
}

static void test_lfo(void)
{
    char label[200];
    const double chorus = measure_lfo_hz(0.0, 1, 48000.0);
    const double vib_lo = measure_lfo_hz(0.0, 0, 48000.0), vib_hi = measure_lfo_hz(1.0, 0, 48000.0);
    snprintf(label, sizeof label, "chorus LFO %.3f Hz, rate formula %.3f Hz", chorus, lfo_rate_hz(0.0, 1));
    check(label, fabs(chorus / lfo_rate_hz(0.0, 1) - 1.0) < 0.01);
    snprintf(label, sizeof label, "vibrato SPEED range %.3f-%.3f Hz (period %.0f-%.0f ms)", vib_lo, vib_hi,
             1e3 / vib_hi, 1e3 / vib_lo);
    check(label, fabs(vib_hi / lfo_rate_hz(1.0, 0) - 1.0) < 0.01 && fabs(vib_lo / lfo_rate_hz(0.0, 0) - 1.0) < 0.01);
    snprintf(label, sizeof label, "fastest vibrato period %.0f ms, schematic 240 ms (within 10%%)", 1e3 / vib_hi);
    check(label, fabs(1e3 / vib_hi / 240.0 - 1.0) < 0.10);
    printf("  (slowest vibrato period %.0f ms; the schematic's waveform note says 1 s, see README)\n", 1e3 / vib_lo);
    printf("  (triangle %.2f-%.2f V)\n", LFO_TRI_LO, LFO_TRI_HI);
}

static void test_bbd(void)
{
    char label[200];
    const double fs = 48000.0;
    Bbd b;
    int n, at = -1, k;
    double vs[2] = { LFO_TRI_LO, LFO_TRI_HI };
    double want[2] = { BBD_DELAY_MAX, BBD_DELAY_MIN };
    for (k = 0; k < 2; ++k) {
        bbd_init(&b, fs);
        b.cv = vs[k];
        at = -1;
        for (n = 0; n < 1000; ++n) {
            const double y = bbd_tick(&b, n == 10 ? 1.0 : 0.0, vs[k]);
            if (at < 0 && fabs(y) > 0.5)
                at = n - 10;
        }
        free(b.buf);
        snprintf(label, sizeof label, "BBD delay at the %s of the sweep: %.2f ms (512 / %.0f kHz = %.2f ms)",
                 k ? "top" : "bottom", 1e3 * at / fs, (k ? BBD_CLK_MAX : BBD_CLK_MIN) / 1e3, 1e3 * want[k]);
        check(label, fabs(at - want[k] * fs) <= 1.0);
    }
}

/* ---- The built plugin ---- */

typedef struct {
    const LV2_Descriptor* d;
    LV2_Handle h;
    float ctl[PORT_COUNT];
} Inst;

static const float DEFAULTS[PORT_COUNT] = {
    0, 0, 0,          /* audio */
    0,                /* input jack: High */
    0.4f, 0.5f, 0.5f, 0.5f, 0.0f, /* volume, treble, middle, bass, hi-treble */
    0.0f, 0.3f,       /* distortion, reverb */
    0.5f, 0.5f, 2,    /* speed, depth, mode: chorus */
    1, 1, 1, 0,       /* footswitches, line out stereo */
    1.0f, 0.0f, 1,    /* volts, output, enabled */
    0                 /* LFO out */
};

static void inst_open(Inst* x, const LV2_Descriptor* d, double fs)
{
    int i;
    x->d = d;
    memcpy(x->ctl, DEFAULTS, sizeof x->ctl);
    x->h = d->instantiate(d, fs, ".", NULL);
    for (i = JD_FIRST_CONTROL; i < PORT_COUNT; ++i)
        d->connect_port(x->h, (uint32_t)i, &x->ctl[i]);
}

static void inst_render(Inst* x, const float* in, float* l, float* r, long n, int block)
{
    long i;
    for (i = 0; i < n; i += block) {
        const uint32_t m = (uint32_t)(n - i < block ? n - i : block);
        x->d->connect_port(x->h, PORT_IN, (void*)(in + i));
        x->d->connect_port(x->h, PORT_OUT_L, l + i);
        x->d->connect_port(x->h, PORT_OUT_R, r + i);
        x->d->run(x->h, m);
    }
}

static void fill_sine(float* b, long n, double f, double fs, double amp)
{
    long i;
    for (i = 0; i < n; ++i)
        b[i] = (float)(amp * sin(2.0 * PI * f * i / fs));
}

static int all_finite(const float* b, long n, double limit, double* peak)
{
    long i;
    *peak = 0.0;
    for (i = 0; i < n; ++i) {
        if (!isfinite(b[i]) || fabs(b[i]) > limit)
            return 0;
        if (fabs(b[i]) > *peak)
            *peak = fabs(b[i]);
    }
    return 1;
}

static double rms(const float* a, long from, long n)
{
    double s = 0.0;
    long i;
    for (i = from; i < from + n; ++i)
        s += (double)a[i] * a[i];
    return sqrt(s / n);
}

static double rms_diff(const float* a, const float* b, long from, long n)
{
    double s = 0.0;
    long i;
    for (i = from; i < from + n; ++i)
        s += ((double)a[i] - b[i]) * ((double)a[i] - b[i]);
    return sqrt(s / n);
}

/* Total harmonic distortion of a sine at f: what's left after fitting the fundamental. */
static double thd(const float* a, long from, long n, double f, double fs)
{
    double sr = 0.0, si = 0.0, tot = 0.0, fund;
    long i;
    for (i = from; i < from + n; ++i) {
        const double ph = 2.0 * PI * f * i / fs;
        sr += a[i] * sin(ph);
        si += a[i] * cos(ph);
        tot += (double)a[i] * a[i];
    }
    sr *= 2.0 / n;
    si *= 2.0 / n;
    fund = 0.5 * (sr * sr + si * si);
    tot /= n;
    return sqrt(fmax(tot - fund, 0.0) / fund);
}

static void test_plugin(const char* bundle)
{
    char path[1024], label[200];
    const double fs = 48000.0;
    const long n = (long)(4.0 * fs);
    float* in = (float*)malloc(sizeof(float) * n);
    float* l = (float*)malloc(sizeof(float) * n);
    float* r = (float*)malloc(sizeof(float) * n);
    const LV2_Descriptor* (*get)(uint32_t);
    const LV2_Descriptor* d;
    void* lib;
    Inst x;
    double peak, t0, t1;
    static const double rates[] = { 44100.0, 48000.0, 88200.0, 96000.0, 192000.0 };
    int i;

    snprintf(path, sizeof path, "%s/%s", bundle, LIB_NAME);
    lib = LIB_OPEN(path);
    check("library loads", lib != NULL);
    if (!lib)
        return;
    *(void**)&get = LIB_SYM(lib, "lv2_descriptor");
    check("exports lv2_descriptor", get != NULL);
    if (!get)
        return;
    d = get(0);
    check("descriptor has the plugin's URI", d && strcmp(d->URI, "urn:davidian:jade-choir") == 0);
    check("only one plugin in the library", get(1) == NULL);

    fill_sine(in, n, 440.0, fs, 0.1);

    /* Chorus off: both sides get the dry signal (plus the same reverb). */
    inst_open(&x, d, fs);
    x.ctl[PORT_MODE] = MODE_OFF;
    d->activate(x.h);
    inst_render(&x, in, l, r, n, 256);
    snprintf(label, sizeof label, "chorus off: left and right match (difference %.1f dB below the signal)",
             20.0 * log10(rms_diff(l, r, n / 2, n / 2) / rms(l, n / 2, n / 2) + 1e-12));
    check(label, rms_diff(l, r, n / 2, n / 2) < 0.01 * rms(l, n / 2, n / 2));
    check("output is finite and bounded", all_finite(l, n, 30.0, &peak) && all_finite(r, n, 30.0, &peak));
    snprintf(label, sizeof label, "line out at volume 4: %.2f V rms for a 0.1 V peak input", rms(l, n / 2, n / 2));
    check(label, rms(l, n / 2, n / 2) > 0.05);
    d->cleanup(x.h);

    /* Chorus on: the right side carries the delayed signal only. */
    inst_open(&x, d, fs);
    x.ctl[PORT_REVERB] = 0.0f;
    d->activate(x.h);
    inst_render(&x, in, l, r, n, 256);
    snprintf(label, sizeof label, "chorus: right differs from left (difference %.1f dB)",
             20.0 * log10(rms_diff(l, r, n / 2, n / 2) / rms(l, n / 2, n / 2)));
    check(label, rms_diff(l, r, n / 2, n / 2) > 0.1 * rms(l, n / 2, n / 2));
    snprintf(label, sizeof label, "chorus: right level %.1f dB from left", 20.0 * log10(rms(r, n / 2, n / 2) / rms(l, n / 2, n / 2)));
    check(label, fabs(20.0 * log10(rms(r, n / 2, n / 2) / rms(l, n / 2, n / 2))) < 6.0);
    d->cleanup(x.h);

    /* Distortion adds harmonics. */
    {
        double clean, dirty;
        inst_open(&x, d, fs);
        x.ctl[PORT_MODE] = MODE_OFF;
        x.ctl[PORT_REVERB] = 0.0f;
        d->activate(x.h);
        inst_render(&x, in, l, r, n, 256);
        clean = thd(l, n / 2, n / 2, 440.0, fs);
        x.ctl[PORT_DISTORTION] = 0.7f;
        inst_render(&x, in, l, r, n, 256);
        dirty = thd(l, n / 2, n / 2, 440.0, fs);
        snprintf(label, sizeof label, "distortion at 7: THD %.1f%% (at OFF %.3f%%)", 100.0 * dirty, 100.0 * clean);
        check(label, clean < 0.01 && dirty > 0.1);
        x.ctl[PORT_FS_DIST] = 0.0f;
        inst_render(&x, in, l, r, n, 256);
        snprintf(label, sizeof label, "distortion footswitch off: THD %.3f%%", 100.0 * thd(l, n / 2, n / 2, 440.0, fs));
        check(label, thd(l, n / 2, n / 2, 440.0, fs) < 0.01);
        d->cleanup(x.h);
    }

    /* Bypass: after the fade, both outputs are the input. */
    inst_open(&x, d, fs);
    d->activate(x.h);
    x.ctl[PORT_ENABLED] = 0.0f;
    inst_render(&x, in, l, r, n, 256);
    check("bypass passes the input to both outputs", rms_diff(l, in, (long)(fs / 10), n - (long)(fs / 10)) < 1e-6
                                                         && rms_diff(r, in, (long)(fs / 10), n - (long)(fs / 10)) < 1e-6);
    d->cleanup(x.h);

    /* The LFO output for the GUI lamp moves. */
    {
        float lo = 2.0f, hi = -2.0f;
        inst_open(&x, d, fs);
        d->activate(x.h);
        for (i = 0; i < 400; ++i) {
            inst_render(&x, in, l, r, 256, 256);
            lo = fminf(lo, x.ctl[PORT_LFO]);
            hi = fmaxf(hi, x.ctl[PORT_LFO]);
        }
        snprintf(label, sizeof label, "LFO lamp output sweeps %.2f-%.2f", lo, hi);
        check(label, lo < 0.1f && hi > 0.9f);
        d->cleanup(x.h);
    }

    /* Every rate, hot input and every effect at full: finite, and CPU time. */
    for (i = 0; i < (int)(sizeof rates / sizeof rates[0]); ++i) {
        const long m = (long)(2.0 * rates[i]);
        float* bi = (float*)malloc(sizeof(float) * m);
        float* bl = (float*)malloc(sizeof(float) * m);
        float* br = (float*)malloc(sizeof(float) * m);
        fill_sine(bi, m, 220.0, rates[i], 1.0);
        inst_open(&x, d, rates[i]);
        x.ctl[PORT_VOLUME] = 1.0f;
        x.ctl[PORT_DISTORTION] = 1.0f;
        x.ctl[PORT_REVERB] = 1.0f;
        x.ctl[PORT_MODE] = MODE_VIB;
        x.ctl[PORT_DEPTH] = 1.0f;
        x.ctl[PORT_SPEED] = 1.0f;
        x.ctl[PORT_TREBLE] = x.ctl[PORT_BASS] = x.ctl[PORT_MIDDLE] = x.ctl[PORT_HI_TREBLE] = 1.0f;
        d->activate(x.h);
        t0 = (double)clock() / CLOCKS_PER_SEC;
        inst_render(&x, bi, bl, br, m, 512);
        t1 = (double)clock() / CLOCKS_PER_SEC;
        snprintf(label, sizeof label, "%.1f kHz, everything at full, 1 V peak in: finite, peak below 20 V (%.1f%% of one core)",
                 rates[i] / 1e3, 100.0 * (t1 - t0) / 2.0);
        check(label, all_finite(bl, m, 20.0, &peak) && all_finite(br, m, 20.0, &peak));
        d->cleanup(x.h);
        free(bi);
        free(bl);
        free(br);
    }
    free(in);
    free(l);
    free(r);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        printf("usage: test_dsp <bundle folder>\n");
        return 2;
    }
    printf("Stages against an AC analysis of the same netlists\n");
    test_against_ac();
    printf("Signal levels printed on the schematic\n");
    test_levels();
    printf("Chorus/vibrato LFO\n");
    test_lfo();
    printf("BBD delay\n");
    test_bbd();
    printf("Built plugin\n");
    test_plugin(argv[1]);
    printf(failures ? "\n%d FAILED\n" : "\nAll passed\n", failures);
    return failures ? 1 : 0;
}

/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* Checks for the Jade Choir. Usage: test_dsp <bundle folder>
 * Part 1 drives the circuit code directly: the linear stages' time-domain solver against an
 * independent AC (complex phasor) analysis of the same netlist; the stages against the DC and
 * signal voltages printed on the schematic; the LFO and BBD against the schematic's figures.
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

/* ---- AC analysis of a linear netlist: complex admittances, solved directly ---- */

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

/* Node voltage at frequency f with caller source `which` at 1 V and every other source (rails
 * included) at 0. Nonlinear devices are left out, so this is for linear stages. */
static Cx ac_node(const Circuit* k, double f, int which, int node)
{
    static Cx A[CK_MAX_DIM][CK_MAX_DIM];
    Cx b[CK_MAX_DIM];
    const double w = 2.0 * PI * f;
    const int n = k->dim;
    int i, j, c, row, drv = 0;
    memset(A, 0, sizeof A);
    memset(b, 0, sizeof b);
    for (i = 0; i < k->nr; ++i)
        ac_g(A, k->r[i].a, k->r[i].b, cx(1.0 / k->r[i].r, 0.0));
    for (i = 0; i < k->nx; ++i)
        ac_g(A, k->x[i].a, k->x[i].b, k->x[i].ind ? cx(0.0, -1.0 / (w * k->x[i].v)) : cx(0.0, w * k->x[i].v));
    for (i = 0; i < k->nsrc; ++i) {
        row = CK_SRC_ROW(k, i);
        A[row][k->src[i].node - 1] = cx(1, 0);
        A[k->src[i].node - 1][row] = cx(1, 0);
        if (!k->src[i].is_dc)
            b[row] = cx(drv++ == which ? 1.0 : 0.0, 0.0);
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
        A[row][k->buf_in - 1] = cx(-k->buf_gain, 0);
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

/* ---- Time-domain measurement: a sine through the solver, amplitude and phase of a node.
 * Stages with JFETs keep their DC operating point (settled beforehand); the DC is removed. */
static Cx td_node(Circuit* k, double fs, double f, double amp, int which, int node, const double* dc_in)
{
    const int settle = (int)(fs * 1.5), len = (int)(fs / f * 20.0 + 0.5);
    double u[CK_MAX_SRC] = { 0 }, sr = 0.0, si = 0.0, mean = 0.0;
    int n, i;
    for (n = 0; n < settle + len; ++n) {
        const double ph = 2.0 * PI * f * n / fs;
        for (i = 0; i < CK_MAX_SRC; ++i)
            u[i] = dc_in ? dc_in[i] : 0.0;
        u[which] += amp * sin(ph);
        ck_step(k, u);
        if (n >= settle) {
            const double v = ck_node(k, node);
            sr += v * sin(ph);
            si += v * cos(ph);
            mean += v;
        }
    }
    (void)mean;
    return cx(2.0 * sr / len / amp, 2.0 * si / len / amp);
}

static double td_rms(Circuit* k, double fs, double f, double rms_in, int which, int node, const double* dc_in)
{
    return cmag(td_node(k, fs, f, rms_in * sqrt(2.0), which, node, dc_in)) * rms_in;
}

static int compare_ac(Circuit* k, double fs, int which, int node, const char* name, double* worst_db)
{
    static const double freqs[] = { 40.0, 100.0, 300.0, 1000.0, 3000.0, 6000.0 };
    int i, ok = 1;
    for (i = 0; i < (int)(sizeof freqs / sizeof freqs[0]); ++i) {
        /* The trapezoidal rule is the bilinear transform: the solver at f matches the circuit at
         * fs / pi * tan(pi f / fs). */
        const Cx a = ac_node(k, fs / PI * tan(PI * freqs[i] / fs), which, node);
        Cx t;
        double ma, mt, db;
        ck_reset(k);
        t = td_node(k, fs, freqs[i], 1e-3, which, node, NULL);
        ma = cmag(a);
        mt = cmag(t);
        db = 20.0 * log10((mt + 1e-30) / (ma + 1e-30));
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
    MixSet m;
    double worst = 0.0;
    int ok = 1, i;

    build_ch2_out(&k, fs);
    ok &= compare_ac(&k, fs, 0, OA_O, "CH-2 output amplifier", &worst);
    build_reverb_drive(&k, 48000.0);
    ok &= compare_ac(&k, 48000.0, 0, RD_SN, "reverb driver (coil current)", &worst);
    build_reverb_return(&k, 48000.0);
    ok &= compare_ac(&k, 48000.0, 0, RR_O, "reverb recovery", &worst);
    build_chorus_pre(&k, 48000.0);
    ok &= compare_ac(&k, 48000.0, 0, CP_O, "chorus input filter", &worst);
    build_chorus_post(&k, 48000.0);
    ok &= compare_ac(&k, 48000.0, 0, CO_O, "chorus output filter", &worst);
    memset(&m, 0, sizeof m);
    m.reverb = 0.6;
    m.s_dry = 0.3;
    m.s_wet = 1.0;
    build_mixer(&k, &m, 48000.0);
    for (i = 0; i < 4; ++i) {
        ok &= compare_ac(&k, 48000.0, i, MX_GL, "mixer, left", &worst);
        ok &= compare_ac(&k, 48000.0, i, MX_GR, "mixer, right", &worst);
    }
    build_power(&k, 48000.0);
    ok &= compare_ac(&k, 48000.0, 0, PA_O, "power amp", &worst);
    snprintf(label, sizeof label, "linear stages match their AC analysis, 40 Hz-6 kHz (worst %.4f dB)", worst);
    check(label, ok);
}

/* ---- The schematic's printed voltages ---- */

static void level_check(const char* what, double got, double want, double tol_db)
{
    char label[220];
    const double db = 20.0 * log10(got / want);
    snprintf(label, sizeof label, "%s: %.0f mV (schematic %.0f mV, %+.1f dB)", what, 1e3 * got, 1e3 * want, db);
    check(label, fabs(db) <= tol_db);
}

static void dc_check(const char* what, double got, double want, double tol)
{
    char label[220];
    snprintf(label, sizeof label, "%s: %+.2f V DC (schematic %+.2f V)", what, got, want);
    check(label, fabs(got - want) <= tol);
}

static void test_levels(void)
{
    const double fs = 96000.0;
    static Circuit k, k2;
    ChanSet c;
    DriveSet d;
    double u[CK_MAX_SRC] = { 0 }, g;

    /* The schematic's test conditions: 1 kHz, BASS and TREBLE fully clockwise, MIDDLE fully
     * anticlockwise. */
    memset(&c, 0, sizeof c);
    c.volume = 1.0;
    c.treble = 1.0;
    c.middle = 0.0;
    c.bass = 1.0;

    printf("  CH-1\n");
    build_chan_in(&k, &c, 0, fs, 1);
    dc_check("Q1 drain (sets the JFET parameters)", ck_node(&k, IN_D), 12.6, 0.05);
    level_check("Q1 drain, 19 mV at HIGH", td_rms(&k, fs, 1000.0, 0.019, 0, IN_D, u), 0.180, 1.0);
    c.low_input = 1;
    build_chan_in(&k, &c, 0, fs, 1);
    level_check("Q1 drain, 59 mV at LOW", td_rms(&k, fs, 1000.0, 0.059, 0, IN_D, u), 0.180, 1.0);
    c.low_input = 0;

    printf("  CH-2\n");
    build_chan_in(&k, &c, 1, fs, 1);
    dc_check("Q1 drain", ck_node(&k, IN_D), 12.8, 0.05);
    level_check("Q1 drain, 19 mV at HIGH", td_rms(&k, fs, 1000.0, 0.019, 0, IN_D, u), 0.180, 1.0);
    level_check("Q2 gate, after the tone stack", td_rms(&k, fs, 1000.0, 0.019, 0, IN_OUT, u), 0.024, 1.5);
    g = ck_node(&k, IN_OUT);
    memset(&d, 0, sizeof d);
    d.volume = 1.0;
    build_ch2_drive(&k2, &d, fs, g, 1);
    dc_check("Q2 source", ck_node(&k2, DR_S2), 13.3, 0.05);
    dc_check("Q3 drain (distortion stage)", ck_node(&k2, DR_D3), 10.4, 0.05);
    u[0] = g;
    level_check("Q2 source, 24 mV at its gate", td_rms(&k2, fs, 1000.0, 0.024, 0, DR_S2, u), 0.027, 1.5);
    g = ck_node(&k2, DR_OUT);
    build_ch2_q5(&k, 1, fs, g, 1);
    dc_check("Q5 drain", ck_node(&k, Q5_D), 5.17, 0.05);
    u[0] = g;
    level_check("Q5 drain, 20 mV at its gate", td_rms(&k, fs, 1000.0, 0.020, 0, Q5_D, u), 0.200, 1.5);
    level_check("reverb send, 20 mV at Q5's gate", td_rms(&k, fs, 1000.0, 0.020, 0, Q5_REV, u), 0.100, 1.5);
    level_check("chorus send (Q19 source), 20 mV at Q5's gate", td_rms(&k, fs, 1000.0, 0.020, 0, Q5_S19, u), 0.185, 1.5);
    {
        /* Q16/Q15 from Q5's 200 mV, through C52/R96. */
        const double g16 = td_rms(&k, fs, 1000.0, 0.020, 0, Q5_G16, u);
        build_ch2_out(&k2, fs);
        level_check("Q15 output (2.1 V), 200 mV at Q5's drain", g16 * cmag(ac_node(&k2, 1000.0, 0, OA_O)), 2.1, 2.5);
    }
    build_chorus_pre(&k, 48000.0);
    level_check("IC2b output (into the BBD), 185 mV in", 0.185 * cmag(ac_node(&k, 1000.0, 0, CP_O)), 0.940, 0.5);
    build_chorus_post(&k, 48000.0);
    printf("  (BBD output filter and de-emphasis pass %.4f at 1 kHz; fx.h has %.4f)\n",
           cmag(ac_node(&k, 1000.0, 0, CO_O)), BBD_POST_1K);
    level_check("IC2a output, 940 mV into the BBD", 0.940 * BBD_GAIN * cmag(ac_node(&k, 1000.0, 0, CO_O)), 2.2, 0.1);
    build_power(&k, 48000.0);
    level_check("speaker, 800 mV at MAIN IN", 0.8 * cmag(ac_node(&k, 1000.0, 0, PA_O)), 21.5, 0.5);
}

/* ---- LFO and BBD ---- */

static double measure_period(double speed, int vib, double fs)
{
    Lfo l;
    double prev = 0.0, first = -1.0, last = -1.0;
    int n, ups = 0;
    lfo_init(&l, fs);
    for (n = 0; n < (int)(fs * 12.0); ++n) {
        const double v = lfo_tick(&l, speed, 1.0, vib);
        if (n && prev < 0.0 && v >= 0.0) {
            if (first < 0)
                first = n;
            last = n;
            ++ups;
        }
        prev = v;
    }
    return ups > 1 ? (last - first) / fs / (ups - 1) : 0.0;
}

static void test_lfo(void)
{
    char label[200];
    const double c = measure_period(0.0, 0, 48000.0);
    const double slow = measure_period(0.0, 1, 48000.0), fast = measure_period(1.0, 1, 48000.0);
    snprintf(label, sizeof label, "chorus LFO period %.3f s (schematic 1.2 s)", c);
    check(label, fabs(c - 1.2) < 0.01);
    snprintf(label, sizeof label, "vibrato SPEED %.0f-%.0f ms (schematic 500 and 120 ms)", 1e3 * slow, 1e3 * fast);
    check(label, fabs(slow - 0.5) < 0.005 && fabs(fast - 0.12) < 0.002);
}

static void test_bbd(void)
{
    char label[200];
    const double fs = 48000.0;
    Bbd b;
    int n, at, k;
    const double vs[2] = { -0.5 * LFO_CHORUS_PP, 0.5 * LFO_CHORUS_PP };
    const double want[2] = { 512 * BBD_T_MIN, 512 * BBD_T_MAX };
    for (k = 0; k < 2; ++k) {
        bbd_init(&b, fs);
        b.cv = vs[k];
        at = -1;
        for (n = 0; n < 2000; ++n) {
            const double y = bbd_tick(&b, n == 10 ? 1.0 : 0.0, vs[k]);
            if (at < 0 && fabs(y) > 0.3)
                at = n - 10;
        }
        free(b.buf);
        snprintf(label, sizeof label, "BBD delay at the %s of the chorus sweep: %.2f ms (512 x %.1f us = %.2f ms)",
                 k ? "top" : "bottom", 1e3 * at / fs, 1e6 * (k ? BBD_T_MAX : BBD_T_MIN), 1e3 * want[k]);
        check(label, fabs(at - want[k] * fs) <= 1.0);
    }
}

/* ---- The built plugin ---- */

typedef struct {
    const LV2_Descriptor* d;
    LV2_Handle h;
    float ctl[PORT_COUNT];
} Inst;

static void inst_open(Inst* x, const LV2_Descriptor* d, double fs)
{
    int i;
    x->d = d;
    memset(x->ctl, 0, sizeof x->ctl);
    x->ctl[PORT_CHANNEL] = CHAN_2;
    x->ctl[PORT_VOLUME1] = x->ctl[PORT_VOLUME2] = 0.4f;
    x->ctl[PORT_TREBLE1] = x->ctl[PORT_MIDDLE1] = x->ctl[PORT_BASS1] = 0.5f;
    x->ctl[PORT_TREBLE2] = x->ctl[PORT_MIDDLE2] = x->ctl[PORT_BASS2] = 0.5f;
    x->ctl[PORT_REVERB] = 0.3f;
    x->ctl[PORT_SPEED] = x->ctl[PORT_DEPTH] = 0.5f;
    x->ctl[PORT_MODE] = MODE_CHORUS;
    x->ctl[PORT_FS_CHORUS] = x->ctl[PORT_FS_REVERB] = x->ctl[PORT_FS_DIST] = 1.0f;
    x->ctl[PORT_VOLTS] = 1.0f;
    x->ctl[PORT_ENABLED] = 1.0f;
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
    double sr = 0.0, si = 0.0, tot = 0.0, fund, mean = 0.0;
    long i;
    for (i = from; i < from + n; ++i)
        mean += a[i];
    mean /= n;
    for (i = from; i < from + n; ++i) {
        const double ph = 2.0 * PI * f * i / fs, v = a[i] - mean;
        sr += v * sin(ph);
        si += v * cos(ph);
        tot += v * v;
    }
    sr *= 2.0 / n;
    si *= 2.0 / n;
    fund = 0.5 * (sr * sr + si * si);
    tot /= n;
    return sqrt(fmax(tot - fund, 0.0) / fund);
}

static void test_plugin(const char* bundle)
{
    char path[1024], label[220];
    const double fs = 48000.0;
    const long n = (long)(4.0 * fs), half = n / 2;
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

    fill_sine(in, n, 440.0, fs, 0.05);

    /* Starts quietly: no thump from the DC operating points. */
    inst_open(&x, d, fs);
    d->activate(x.h);
    {
        float* z = (float*)calloc((size_t)fs, sizeof(float));
        inst_render(&x, z, l, r, (long)fs, 256);
        {
            double pl, pr;
            long j, at = 0;
            all_finite(l, (long)fs, 1e9, &pl);
            all_finite(r, (long)fs, 1e9, &pr);
            for (j = 0; j < (long)fs; ++j)
                if (fabs(l[j]) == pl || fabs(r[j]) == pr)
                    at = j;
            snprintf(label, sizeof label, "silence in, silence out (peak %.1e at %.0f ms)", fmax(pl, pr), 1e3 * at / fs);
            check(label, fmax(pl, pr) < 1e-3);
        }
        free(z);
    }
    d->cleanup(x.h);

    /* CH-2, effect off: its dry signal reaches both speakers. */
    inst_open(&x, d, fs);
    x.ctl[PORT_MODE] = MODE_OFF;
    x.ctl[PORT_REVERB] = 0.0f;
    d->activate(x.h);
    inst_render(&x, in, l, r, n, 256);
    snprintf(label, sizeof label, "CH-2, effect off: dry on both sides (right %+.1f dB from left)",
             20.0 * log10(rms(r, half, half) / rms(l, half, half)));
    check(label, rms(l, half, half) > 1e-3 && fabs(20.0 * log10(rms(r, half, half) / rms(l, half, half))) < 3.0);
    check("output is finite and bounded", all_finite(l, n, 1.5, &peak) && all_finite(r, n, 1.5, &peak));
    d->cleanup(x.h);

    /* Chorus on: the right side carries the delayed signal instead of the dry one. */
    inst_open(&x, d, fs);
    x.ctl[PORT_REVERB] = 0.0f;
    d->activate(x.h);
    inst_render(&x, in, l, r, n, 256);
    snprintf(label, sizeof label, "chorus: right differs from left (difference %.1f dB below left)",
             20.0 * log10(rms_diff(l, r, half, half) / rms(l, half, half)));
    check(label, rms_diff(l, r, half, half) > 0.1 * rms(l, half, half));
    snprintf(label, sizeof label, "chorus: right %+.1f dB from left", 20.0 * log10(rms(r, half, half) / rms(l, half, half)));
    check(label, fabs(20.0 * log10(rms(r, half, half) / rms(l, half, half))) < 6.0);
    d->cleanup(x.h);

    /* CH-1 feeds both power amps. */
    inst_open(&x, d, fs);
    x.ctl[PORT_CHANNEL] = CHAN_1;
    x.ctl[PORT_REVERB] = 0.0f;
    d->activate(x.h);
    inst_render(&x, in, l, r, n, 256);
    snprintf(label, sizeof label, "CH-1 reaches both speakers (right %+.1f dB from left)",
             20.0 * log10(rms(r, half, half) / rms(l, half, half)));
    check(label, rms(l, half, half) > 1e-3 && fabs(20.0 * log10(rms(r, half, half) / rms(l, half, half))) < 3.0);
    d->cleanup(x.h);

    /* Distortion adds harmonics; its footswitch takes them away. */
    {
        double clean, dirty;
        inst_open(&x, d, fs);
        x.ctl[PORT_MODE] = MODE_OFF;
        x.ctl[PORT_REVERB] = 0.0f;
        d->activate(x.h);
        inst_render(&x, in, l, r, n, 256);
        clean = thd(l, half, half, 440.0, fs);
        x.ctl[PORT_DISTORTION] = 0.8f;
        inst_render(&x, in, l, r, n, 256);
        dirty = thd(l, half, half, 440.0, fs);
        snprintf(label, sizeof label, "distortion at 8: THD %.1f%% (at OFF %.2f%%)", 100.0 * dirty, 100.0 * clean);
        check(label, clean < 0.02 && dirty > 0.05);
        x.ctl[PORT_FS_DIST] = 0.0f;
        inst_render(&x, in, l, r, n, 256);
        snprintf(label, sizeof label, "distortion footswitch off: THD %.2f%%", 100.0 * thd(l, half, half, 440.0, fs));
        check(label, thd(l, half, half, 440.0, fs) < 0.02);
        d->cleanup(x.h);
    }

    /* Reverb adds a tail. */
    {
        float* burst = (float*)calloc((size_t)n, sizeof(float));
        double tail;
        for (i = 0; i < (int)(0.2 * fs); ++i)
            burst[i] = in[i];
        inst_open(&x, d, fs);
        x.ctl[PORT_MODE] = MODE_OFF;
        x.ctl[PORT_REVERB] = 1.0f;
        d->activate(x.h);
        inst_render(&x, burst, l, r, n, 256);
        tail = rms(l, (long)(0.5 * fs), (long)(0.3 * fs)) / rms(l, (long)(0.05 * fs), (long)(0.1 * fs));
        snprintf(label, sizeof label, "reverb tail 0.3 s after the note: %.1f dB below it", 20.0 * log10(tail));
        check(label, tail > 0.01 && tail < 1.0);
        d->cleanup(x.h);
        free(burst);
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
        check(label, hi - lo > 0.5f);
        d->cleanup(x.h);
    }

    /* CPU time at the defaults (CH 2, chorus, reverb). */
    {
        const long m = (long)(4.0 * fs);
        float* bi = (float*)malloc(sizeof(float) * m);
        float* bl = (float*)malloc(sizeof(float) * m);
        float* br = (float*)malloc(sizeof(float) * m);
        fill_sine(bi, m, 220.0, fs, 0.1);
        inst_open(&x, d, fs);
        d->activate(x.h);
        t0 = (double)clock() / CLOCKS_PER_SEC;
        inst_render(&x, bi, bl, br, m, 512);
        t1 = (double)clock() / CLOCKS_PER_SEC;
        printf("  (defaults at 48 kHz: %.1f%% of one core)\n", 100.0 * (t1 - t0) / 4.0);
        d->cleanup(x.h);
        free(bi);
        free(bl);
        free(br);
    }

    /* Every rate, hot input and everything at full: finite, and CPU time. */
    for (i = 0; i < (int)(sizeof rates / sizeof rates[0]); ++i) {
        const long m = (long)(2.0 * rates[i]);
        float* bi = (float*)malloc(sizeof(float) * m);
        float* bl = (float*)malloc(sizeof(float) * m);
        float* br = (float*)malloc(sizeof(float) * m);
        fill_sine(bi, m, 220.0, rates[i], 1.0);
        inst_open(&x, d, rates[i]);
        x.ctl[PORT_CHANNEL] = CHAN_BOTH;
        x.ctl[PORT_VOLUME1] = x.ctl[PORT_VOLUME2] = 1.0f;
        x.ctl[PORT_DISTORTION] = 1.0f;
        x.ctl[PORT_REVERB] = 1.0f;
        x.ctl[PORT_MODE] = MODE_VIB;
        x.ctl[PORT_DEPTH] = x.ctl[PORT_SPEED] = 1.0f;
        x.ctl[PORT_BRIGHT1] = x.ctl[PORT_BRIGHT2] = 1.0f;
        d->activate(x.h);
        t0 = (double)clock() / CLOCKS_PER_SEC;
        inst_render(&x, bi, bl, br, m, 512);
        t1 = (double)clock() / CLOCKS_PER_SEC;
        snprintf(label, sizeof label, "%.1f kHz, everything at full, 1 V peak in: finite, within 1.5 x full scale (%.1f%% of one core)",
                 rates[i] / 1e3, 100.0 * (t1 - t0) / 2.0);
        check(label, all_finite(bl, m, 1.5, &peak) && all_finite(br, m, 1.5, &peak));
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
    printf("Linear stages against an AC analysis of the same netlists\n");
    test_against_ac();
    printf("Voltages printed on the schematic\n");
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

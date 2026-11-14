/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* A small circuit simulator for the stages of the amplifier: modified nodal analysis with
 * trapezoidal capacitors, ideal op amps that clip at their output swing, unity-gain buffers
 * (emitter followers) and antiparallel diode pairs solved by Newton's method (the "DK"
 * method: the linear part is inverted once per control setting, the diodes are solved in a
 * small system of their own every sample).
 *
 * Each stage is a netlist built by a function in amp.h from the schematic's part values.
 * Node 0 is ground. Stages are joined by op amp outputs, which are ideal voltage sources, so
 * every stage can be solved on its own.
 *
 * Per sample, the only inputs are the capacitor history currents, the source voltages and the
 * op amp's clip level; everything else is a fixed linear map of those, worked out when the
 * stage is built. So a step costs (capacitors + diodes) x (capacitors + sources) multiplies,
 * and any node voltage can be read afterwards. */
#ifndef JD_CIRCUIT_H
#define JD_CIRCUIT_H

#include <math.h>
#include <string.h>

#define CK_MAX_NODES 26
#define CK_MAX_R 40
#define CK_MAX_C 16
#define CK_MAX_SRC 3
#define CK_MAX_D 2
#define CK_MAX_DIM (CK_MAX_NODES + CK_MAX_SRC + 2 + 1)
#define CK_MAX_Z (CK_MAX_C + CK_MAX_SRC + 1)

/* Op amp output swing: NJM4558 (a listed substitute for the M5218 on the parts list),
 * datasheet typical +/-14 V into 10k on +/-15 V supplies. */
#define OPAMP_SWING 14.0

/* 1S2473 small-signal silicon diode, the Japanese counterpart of the 1N4148: the PSpice
 * library D1N4148 model, IS = 2.682 nA, N = 1.836, at 25 C (thermal voltage 25.85 mV). */
#define DIODE_IS 2.682e-9
#define DIODE_NVT (1.836 * 0.025852)

static inline double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

typedef struct { int a, b; double r; } CkR;
typedef struct { int a, b; double c; } CkC;
typedef struct { int a, b; } CkD;

typedef struct {
    /* Netlist */
    int nodes;
    int nr, nc, nsrc, nd;
    CkR r[CK_MAX_R];
    CkC c[CK_MAX_C];
    CkD d[CK_MAX_D];
    int src[CK_MAX_SRC];      /* node driven by input u[k] (to ground) */
    int op_p, op_m, op_o;     /* one op amp; op_o = 0 if none */
    int op2_p, op2_m, op2_o;  /* a second op amp, never clipped here (see ck_step) */
    int buf_in, buf_out;      /* a unity-gain buffer; buf_out = 0 if none */
    int noclip;               /* the caller clips the op amp outputs instead */

    /* Solver. z = [capacitor histories, source voltages, op amp clip level]. */
    int dim, nz;
    double T;                             /* sample period */
    double W[3][CK_MAX_DIM][CK_MAX_Z];    /* every unknown from z: op amp linear, at +, at - */
    double SN[3][CK_MAX_DIM][CK_MAX_D];   /* every unknown from the diode currents */
    double K[3][CK_MAX_D][CK_MAX_D];      /* diode voltages from the diode currents */
    double Wc[3][CK_MAX_C][CK_MAX_Z];     /* capacitor voltages from z */
    double SNc[3][CK_MAX_C][CK_MAX_D];    /* capacitor voltages from the diode currents */
    double Wd[3][CK_MAX_D][CK_MAX_Z];     /* diode voltages from z, without their currents */
    int ok[3];

    /* State */
    double h[CK_MAX_C];       /* capacitor history currents */
    double vd[CK_MAX_D];      /* diode voltages (Newton's starting point next sample) */
    double z[CK_MAX_Z];       /* last sample's inputs */
    double id[CK_MAX_D];      /* last sample's diode currents */
    int mode;                 /* last sample's op amp state */
} Circuit;

/* Starts a netlist. The capacitor histories and the last solution are kept, so a stage can be
 * rebuilt with new control settings while audio runs; the builders always add the capacitors
 * in the same order, whatever the switch settings. */
static void ck_begin(Circuit* k, int nodes)
{
    k->nodes = nodes;
    k->nr = k->nc = k->nsrc = k->nd = 0;
    k->op_p = k->op_m = k->op_o = 0;
    k->op2_p = k->op2_m = k->op2_o = 0;
    k->buf_in = k->buf_out = 0;
    k->noclip = 0;
}

static void ck_r(Circuit* k, int a, int b, double r)
{
    if (k->nr < CK_MAX_R && r > 0.0 && a != b) {
        k->r[k->nr].a = a;
        k->r[k->nr].b = b;
        k->r[k->nr].r = r;
        ++k->nr;
    }
}

static void ck_c(Circuit* k, int a, int b, double c)
{
    if (k->nc < CK_MAX_C) {
        k->c[k->nc].a = a;
        k->c[k->nc].b = b;
        k->c[k->nc].c = c;
        ++k->nc;
    }
}

static void ck_diodes(Circuit* k, int a, int b)
{
    if (k->nd < CK_MAX_D) {
        k->d[k->nd].a = a;
        k->d[k->nd].b = b;
        ++k->nd;
    }
}

static void ck_src(Circuit* k, int node)
{
    if (k->nsrc < CK_MAX_SRC)
        k->src[k->nsrc++] = node;
}

static void ck_opamp(Circuit* k, int p, int m, int o)
{
    if (!k->op_o) {
        k->op_p = p;
        k->op_m = m;
        k->op_o = o;
    } else {
        k->op2_p = p;
        k->op2_m = m;
        k->op2_o = o;
    }
}

static void ck_buffer(Circuit* k, int in, int out)
{
    k->buf_in = in;
    k->buf_out = out;
}

/* Gauss-Jordan inversion with partial pivoting. Returns 0 if singular. */
static int ck_invert(int n, double a[CK_MAX_DIM][CK_MAX_DIM], double inv[CK_MAX_DIM][CK_MAX_DIM])
{
    int i, j, col;
    for (i = 0; i < n; ++i)
        for (j = 0; j < n; ++j)
            inv[i][j] = i == j ? 1.0 : 0.0;
    for (col = 0; col < n; ++col) {
        int piv = col;
        double best = fabs(a[col][col]), d;
        for (i = col + 1; i < n; ++i)
            if (fabs(a[i][col]) > best) {
                best = fabs(a[i][col]);
                piv = i;
            }
        if (best < 1e-300)
            return 0;
        if (piv != col)
            for (j = 0; j < n; ++j) {
                double t = a[col][j];
                a[col][j] = a[piv][j];
                a[piv][j] = t;
                t = inv[col][j];
                inv[col][j] = inv[piv][j];
                inv[piv][j] = t;
            }
        d = 1.0 / a[col][col];
        for (j = 0; j < n; ++j) {
            a[col][j] *= d;
            inv[col][j] *= d;
        }
        for (i = 0; i < n; ++i) {
            const double f = a[i][col];
            if (i == col || f == 0.0)
                continue;
            for (j = 0; j < n; ++j) {
                a[i][j] -= f * a[col][j];
                inv[i][j] -= f * inv[col][j];
            }
        }
    }
    return 1;
}

/* Row/column of each unknown. Nodes 1..nodes are 0..nodes-1. */
#define CK_SRC_ROW(k, i) ((k)->nodes + (i))
#define CK_OP_ROW(k) ((k)->nodes + (k)->nsrc)
#define CK_OP2_ROW(k) (CK_OP_ROW(k) + ((k)->op_o ? 1 : 0))
#define CK_BUF_ROW(k) (CK_OP2_ROW(k) + ((k)->op2_o ? 1 : 0))
/* z index of the clip level */
#define CK_Z_CLIP(k) ((k)->nc + (k)->nsrc)

static void ck_stamp_g(double A[CK_MAX_DIM][CK_MAX_DIM], int a, int b, double g)
{
    if (a) A[a - 1][a - 1] += g;
    if (b) A[b - 1][b - 1] += g;
    if (a && b) {
        A[a - 1][b - 1] -= g;
        A[b - 1][a - 1] -= g;
    }
}

/* Builds the maps for one op amp state (0 linear, 1 at +swing, 2 at -swing). */
static void ck_build_mode(Circuit* k, int mode)
{
    double A[CK_MAX_DIM][CK_MAX_DIM], S[CK_MAX_DIM][CK_MAX_DIM]; /* stack: builds may run on several threads */
    int i, j, row;
    memset(A, 0, sizeof A);
    for (i = 0; i < k->nr; ++i)
        ck_stamp_g(A, k->r[i].a, k->r[i].b, 1.0 / k->r[i].r);
    for (i = 0; i < k->nc; ++i)
        ck_stamp_g(A, k->c[i].a, k->c[i].b, 2.0 * k->c[i].c / k->T);
    for (i = 0; i < k->nsrc; ++i) {
        row = CK_SRC_ROW(k, i);
        A[row][k->src[i] - 1] = 1.0;
        A[k->src[i] - 1][row] = 1.0;
    }
    if (k->op_o) {
        row = CK_OP_ROW(k);
        if (mode == 0) {
            if (k->op_p) A[row][k->op_p - 1] = 1.0;
            if (k->op_m) A[row][k->op_m - 1] = -1.0;
        } else {
            A[row][k->op_o - 1] = 1.0; /* output pinned at the swing */
        }
        A[k->op_o - 1][row] = 1.0;
    }
    if (k->op2_o) {
        row = CK_OP2_ROW(k);
        if (k->op2_p) A[row][k->op2_p - 1] = 1.0;
        if (k->op2_m) A[row][k->op2_m - 1] = -1.0;
        A[k->op2_o - 1][row] = 1.0;
    }
    if (k->buf_out) {
        row = CK_BUF_ROW(k);
        A[row][k->buf_out - 1] = 1.0;
        A[row][k->buf_in - 1] = -1.0;
        A[k->buf_out - 1][row] = 1.0;
    }
    k->ok[mode] = ck_invert(k->dim, A, S);

    /* The right-hand side is sum h_i (e_a - e_b) + sum u_k e_src + clip e_op, so each unknown
     * is a row of S read at those places. */
    for (i = 0; i < k->dim; ++i) {
        for (j = 0; j < k->nc; ++j)
            k->W[mode][i][j] = (k->c[j].a ? S[i][k->c[j].a - 1] : 0.0) - (k->c[j].b ? S[i][k->c[j].b - 1] : 0.0);
        for (j = 0; j < k->nsrc; ++j)
            k->W[mode][i][k->nc + j] = S[i][CK_SRC_ROW(k, j)];
        k->W[mode][i][CK_Z_CLIP(k)] = k->op_o ? S[i][CK_OP_ROW(k)] : 0.0;
        /* Diode current i_j leaves node a and enters node b. */
        for (j = 0; j < k->nd; ++j)
            k->SN[mode][i][j] = (k->d[j].b ? S[i][k->d[j].b - 1] : 0.0) - (k->d[j].a ? S[i][k->d[j].a - 1] : 0.0);
    }
    for (i = 0; i < k->nd; ++i)
        for (j = 0; j < k->nd; ++j)
            k->K[mode][i][j] = (k->d[i].a ? k->SN[mode][k->d[i].a - 1][j] : 0.0)
                               - (k->d[i].b ? k->SN[mode][k->d[i].b - 1][j] : 0.0);
    /* Differences of rows, for the capacitors and diodes. */
    for (i = 0; i < k->nc; ++i) {
        const int a = k->c[i].a, b = k->c[i].b;
        for (j = 0; j < k->nz; ++j)
            k->Wc[mode][i][j] = (a ? k->W[mode][a - 1][j] : 0.0) - (b ? k->W[mode][b - 1][j] : 0.0);
        for (j = 0; j < k->nd; ++j)
            k->SNc[mode][i][j] = (a ? k->SN[mode][a - 1][j] : 0.0) - (b ? k->SN[mode][b - 1][j] : 0.0);
    }
    for (i = 0; i < k->nd; ++i) {
        const int a = k->d[i].a, b = k->d[i].b;
        for (j = 0; j < k->nz; ++j)
            k->Wd[mode][i][j] = (a ? k->W[mode][a - 1][j] : 0.0) - (b ? k->W[mode][b - 1][j] : 0.0);
    }
}

/* (Re)builds the solver after the netlist or the sample rate changed. */
static void ck_build(Circuit* k, double sample_rate)
{
    k->T = 1.0 / sample_rate;
    k->dim = k->nodes + k->nsrc + (k->op_o ? 1 : 0) + (k->op2_o ? 1 : 0) + (k->buf_out ? 1 : 0);
    k->nz = k->nc + k->nsrc + 1;
    ck_build_mode(k, 0);
    if (k->op_o) {
        ck_build_mode(k, 1);
        ck_build_mode(k, 2);
    }
}

static void ck_reset(Circuit* k)
{
    memset(k->h, 0, sizeof k->h);
    memset(k->vd, 0, sizeof k->vd);
    memset(k->z, 0, sizeof k->z);
    memset(k->id, 0, sizeof k->id);
    k->mode = 0;
}

/* Any unknown (a node voltage for n = node - 1) after the last ck_step. */
static inline double ck_unknown(const Circuit* k, int i)
{
    const double* w = k->W[k->mode][i];
    double s = 0.0;
    int j;
    for (j = 0; j < k->nz; ++j)
        s += w[j] * k->z[j];
    for (j = 0; j < k->nd; ++j)
        s += k->SN[k->mode][i][j] * k->id[j];
    return s;
}

static inline double ck_node(const Circuit* k, int n) { return n ? ck_unknown(k, n - 1) : 0.0; }

/* Antiparallel diode pair current and slope. */
static inline double ck_diode_i(double v, double* di)
{
    const double x = clampd(v / DIODE_NVT, -80.0, 80.0);
    const double ep = exp(x), em = 1.0 / ep;
    *di = DIODE_IS / DIODE_NVT * (ep + em);
    return DIODE_IS * (ep - em);
}

static inline double ck_dot(const double* a, const double* b, int n)
{
    double s = 0.0;
    int i;
    for (i = 0; i < n; ++i)
        s += a[i] * b[i];
    return s;
}

/* Solves the diode currents for k->z in this mode. */
static void ck_diodes_solve(Circuit* k, int mode)
{
    double p[CK_MAX_D], vd[CK_MAX_D];
    int i, j, it;
    /* Start from last sample's currents: exact when they haven't changed. */
    for (i = 0; i < k->nd; ++i)
        p[i] = ck_dot(k->Wd[mode][i], k->z, k->nz);
    for (i = 0; i < k->nd; ++i) {
        vd[i] = p[i];
        for (j = 0; j < k->nd; ++j)
            vd[i] += k->K[mode][i][j] * k->id[j];
        /* ...unless that guess lands far from where the diode was. */
        if (fabs(vd[i] - k->vd[i]) > 0.05)
            vd[i] = k->vd[i];
    }
    /* Newton on the diode voltages: vd = p + K * i(vd), with limited steps so the exponential
     * can't run away. */
    for (it = 0; it < 60; ++it) {
        double f[CK_MAX_D], J[CK_MAX_D][CK_MAX_D], gd[CK_MAX_D], id[CK_MAX_D], step[CK_MAX_D], worst = 0.0;
        for (i = 0; i < k->nd; ++i)
            id[i] = ck_diode_i(vd[i], &gd[i]);
        for (i = 0; i < k->nd; ++i) {
            f[i] = vd[i] - p[i];
            for (j = 0; j < k->nd; ++j) {
                f[i] -= k->K[mode][i][j] * id[j];
                J[i][j] = (i == j ? 1.0 : 0.0) - k->K[mode][i][j] * gd[j];
            }
        }
        if (k->nd == 1) {
            step[0] = f[0] / J[0][0];
        } else {
            const double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
            step[0] = (J[1][1] * f[0] - J[0][1] * f[1]) / det;
            step[1] = (J[0][0] * f[1] - J[1][0] * f[0]) / det;
        }
        for (i = 0; i < k->nd; ++i) {
            step[i] = clampd(step[i], -0.1, 0.1);
            vd[i] -= step[i];
            if (fabs(step[i]) > worst)
                worst = fabs(step[i]);
        }
        if (worst < 1e-9)
            break;
    }
    for (i = 0; i < k->nd; ++i) {
        double g;
        k->id[i] = ck_diode_i(vd[i], &g);
        k->vd[i] = vd[i];
    }
}

/* One sample. u[] are the source voltages. The op amp clips at OPAMP_SWING: if the linear
 * solution goes past it, the output is pinned there instead (the inverting input then no
 * longer follows). A second op amp, used only in the output mixer, is clipped by the caller. */
static void ck_step(Circuit* k, const double* u)
{
    int i, mode = 0;
    for (i = 0; i < k->nc; ++i)
        k->z[i] = k->h[i];
    for (i = 0; i < k->nsrc; ++i)
        k->z[k->nc + i] = u[i];
    k->z[CK_Z_CLIP(k)] = 0.0;
    if (!k->ok[0])
        return;
    ck_diodes_solve(k, 0);
    k->mode = 0;
    if (k->op_o && !k->noclip) {
        const double vo = ck_unknown(k, k->op_o - 1);
        if (vo > OPAMP_SWING || vo < -OPAMP_SWING) {
            mode = vo > 0.0 ? 1 : 2;
            k->z[CK_Z_CLIP(k)] = mode == 1 ? OPAMP_SWING : -OPAMP_SWING;
            ck_diodes_solve(k, mode);
            k->mode = mode;
        }
    }
    for (i = 0; i < k->nc; ++i) {
        const double g = 2.0 * k->c[i].c / k->T;
        const double vc = ck_dot(k->Wc[k->mode][i], k->z, k->nz) + ck_dot(k->SNc[k->mode][i], k->id, k->nd);
        /* i = g v - h, and the next history is g v + i. (z keeps this sample's h.) */
        k->h[i] = 2.0 * g * vc - k->h[i];
    }
}

#endif

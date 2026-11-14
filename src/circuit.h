/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* A small circuit simulator for the stages of the amplifier: modified nodal analysis with
 * trapezoidal capacitors and inductors, ideal op amps that clip at their output limits,
 * unity-gain buffers (emitter and source followers), DC supply rails, and two kinds of
 * nonlinear device solved by Newton's method: antiparallel diode pairs and n-channel JFETs
 * (square law, with the triode region). This is the "DK" method: the linear part is inverted
 * once per control setting, and only the nonlinear devices are solved every sample.
 *
 * Each stage is a netlist built by a function in amp.h from the schematic's part values.
 * Node 0 is ground. Stages are joined by low-impedance outputs (op amps, followers), which are
 * ideal voltage sources, so every stage can be solved on its own.
 *
 * Per sample, the only inputs are the reactive parts' history currents, the source voltages
 * and the op amp's clip level; everything else is a fixed linear map of those, worked out
 * when the stage is built. */
#ifndef JD_CIRCUIT_H
#define JD_CIRCUIT_H

#include <math.h>
#include <string.h>

#define CK_MAX_NODES 26
#define CK_MAX_R 40
#define CK_MAX_C 18
#define CK_MAX_SRC 6
#define CK_MAX_NL 3
#define CK_MAX_CTL (2 * CK_MAX_NL)
#define CK_MAX_DIM (CK_MAX_NODES + CK_MAX_SRC + 3)
#define CK_MAX_Z (CK_MAX_C + CK_MAX_SRC + 1)

/* 1S2473 small-signal silicon diode, the Japanese counterpart of the 1N4148: the PSpice
 * library D1N4148 model, IS = 2.682 nA, N = 1.836, at 25 C (thermal voltage 25.85 mV). */
#define DIODE_IS 2.682e-9
#define DIODE_NVT (1.836 * 0.025852)

static inline double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

typedef struct { int a, b; double r; } CkR;
typedef struct { int a, b; double v; int ind; } CkX;   /* capacitor (v = C) or inductor (v = L) */
typedef struct { int node; double dc; int is_dc; } CkS;
enum { NL_DIODES, NL_JFET };
typedef struct {
    int type;
    int a, b, c;       /* diodes: a, b. JFET: drain a, source b, gate c */
    double beta, vp;   /* JFET: Idss / Vp^2 and the pinch-off voltage (negative) */
} CkN;

typedef struct {
    /* Netlist */
    int nodes;
    int nr, nx, nsrc, nnl;
    CkR r[CK_MAX_R];
    CkX x[CK_MAX_C];
    CkS src[CK_MAX_SRC];
    CkN nl[CK_MAX_NL];
    int op_p, op_m, op_o;     /* one op amp; op_o = 0 if none */
    double op_lo, op_hi;      /* its output limits */
    int op2_p, op2_m, op2_o;  /* a second op amp, never clipped here (see ck_step) */
    int buf_in, buf_out;      /* a unity-gain buffer; buf_out = 0 if none */
    double buf_gain;
    int noclip;               /* the caller clips the op amp outputs instead */

    /* Solver. z = [reactive histories, source voltages, op amp clip level]. */
    int dim, nz, nctl;
    double T;
    double W[3][CK_MAX_DIM][CK_MAX_Z];       /* every unknown from z: op amp linear, at hi, at lo */
    double SN[3][CK_MAX_DIM][CK_MAX_NL];     /* every unknown from the nonlinear currents */
    double Wx[3][CK_MAX_C][CK_MAX_Z];        /* reactive part voltages from z */
    double SNx[3][CK_MAX_C][CK_MAX_NL];
    double Wv[3][CK_MAX_CTL][CK_MAX_Z];      /* nonlinear control voltages from z */
    double Kv[3][CK_MAX_CTL][CK_MAX_NL];     /* ... and from the nonlinear currents */
    int ctl_a[CK_MAX_CTL], ctl_b[CK_MAX_CTL];
    int ok[3];

    /* State */
    double q[CK_MAX_C];       /* reactive history: current a->b is g v + q */
    double v[CK_MAX_CTL];     /* nonlinear control voltages (Newton's start next sample) */
    double z[CK_MAX_Z];
    double inl[CK_MAX_NL];
    int mode;
} Circuit;

/* Starts a netlist. Histories and the last solution are kept, so a stage can be rebuilt with
 * new control settings while audio runs; the builders always add the reactive parts in the
 * same order, whatever the switch settings. */
static void ck_begin(Circuit* k, int nodes)
{
    k->nodes = nodes;
    k->nr = k->nx = k->nsrc = k->nnl = 0;
    k->op_p = k->op_m = k->op_o = 0;
    k->op_lo = -14.0;
    k->op_hi = 14.0;
    k->op2_p = k->op2_m = k->op2_o = 0;
    k->buf_in = k->buf_out = 0;
    k->buf_gain = 1.0;
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

static void ck_x(Circuit* k, int a, int b, double v, int ind)
{
    if (k->nx < CK_MAX_C) {
        k->x[k->nx].a = a;
        k->x[k->nx].b = b;
        k->x[k->nx].v = v;
        k->x[k->nx].ind = ind;
        ++k->nx;
    }
}

static void ck_c(Circuit* k, int a, int b, double c) { ck_x(k, a, b, c, 0); }
static void ck_l(Circuit* k, int a, int b, double l) { ck_x(k, a, b, l, 1); }

static void ck_diodes(Circuit* k, int a, int b)
{
    if (k->nnl < CK_MAX_NL) {
        CkN* n = &k->nl[k->nnl++];
        memset(n, 0, sizeof *n);
        n->type = NL_DIODES;
        n->a = a;
        n->b = b;
    }
}

/* n-channel JFET: drain d, gate g, source s; Idss in amps, pinch-off vp in volts (negative). */
static void ck_jfet(Circuit* k, int d, int g, int s, double idss, double vp)
{
    if (k->nnl < CK_MAX_NL) {
        CkN* n = &k->nl[k->nnl++];
        n->type = NL_JFET;
        n->a = d;
        n->b = s;
        n->c = g;
        n->vp = vp;
        n->beta = idss / (vp * vp);
    }
}

/* A source driven by the caller (u[] in ck_step, in the order added). */
static void ck_src(Circuit* k, int node)
{
    if (k->nsrc < CK_MAX_SRC) {
        k->src[k->nsrc].node = node;
        k->src[k->nsrc].is_dc = 0;
        k->src[k->nsrc].dc = 0.0;
        ++k->nsrc;
    }
}

/* A supply rail. */
static void ck_rail(Circuit* k, int node, double volts)
{
    if (k->nsrc < CK_MAX_SRC) {
        k->src[k->nsrc].node = node;
        k->src[k->nsrc].is_dc = 1;
        k->src[k->nsrc].dc = volts;
        ++k->nsrc;
    }
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

static void ck_buffer(Circuit* k, int in, int out, double gain)
{
    k->buf_in = in;
    k->buf_out = out;
    k->buf_gain = gain;
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

#define CK_SRC_ROW(k, i) ((k)->nodes + (i))
#define CK_OP_ROW(k) ((k)->nodes + (k)->nsrc)
#define CK_OP2_ROW(k) (CK_OP_ROW(k) + ((k)->op_o ? 1 : 0))
#define CK_BUF_ROW(k) (CK_OP2_ROW(k) + ((k)->op2_o ? 1 : 0))
#define CK_Z_CLIP(k) ((k)->nx + (k)->nsrc)

static void ck_stamp_g(double A[CK_MAX_DIM][CK_MAX_DIM], int a, int b, double g)
{
    if (a) A[a - 1][a - 1] += g;
    if (b) A[b - 1][b - 1] += g;
    if (a && b) {
        A[a - 1][b - 1] -= g;
        A[b - 1][a - 1] -= g;
    }
}

static inline double ck_xg(const Circuit* k, int i)
{
    return k->x[i].ind ? k->T / (2.0 * k->x[i].v) : 2.0 * k->x[i].v / k->T;
}


/* Builds the maps for one op amp state (0 linear, 1 at op_hi, 2 at op_lo). */
static void ck_build_mode(Circuit* k, int mode)
{
    double A[CK_MAX_DIM][CK_MAX_DIM], S[CK_MAX_DIM][CK_MAX_DIM]; /* stack: builds may run on several threads */
    int i, j, row;
    memset(A, 0, sizeof A);
    for (i = 0; i < k->nr; ++i)
        ck_stamp_g(A, k->r[i].a, k->r[i].b, 1.0 / k->r[i].r);
    for (i = 0; i < k->nx; ++i)
        ck_stamp_g(A, k->x[i].a, k->x[i].b, ck_xg(k, i));
    for (i = 0; i < k->nsrc; ++i) {
        row = CK_SRC_ROW(k, i);
        A[row][k->src[i].node - 1] = 1.0;
        A[k->src[i].node - 1][row] = 1.0;
    }
    if (k->op_o) {
        row = CK_OP_ROW(k);
        if (mode == 0) {
            if (k->op_p) A[row][k->op_p - 1] = 1.0;
            if (k->op_m) A[row][k->op_m - 1] = -1.0;
        } else {
            A[row][k->op_o - 1] = 1.0; /* output pinned at its limit */
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
        A[row][k->buf_in - 1] = -k->buf_gain;
        A[k->buf_out - 1][row] = 1.0;
    }
    k->ok[mode] = ck_invert(k->dim, A, S);

    /* The right-hand side: reactive part i carries g v + q from a to b, so -q leaves at a and
     * +q at b; the sources and the clip level sit in their own rows. Nonlinear current j
     * leaves its node a and enters node b. */
    for (i = 0; i < k->dim; ++i) {
        for (j = 0; j < k->nx; ++j)
            k->W[mode][i][j] = (k->x[j].b ? S[i][k->x[j].b - 1] : 0.0) - (k->x[j].a ? S[i][k->x[j].a - 1] : 0.0);
        for (j = 0; j < k->nsrc; ++j)
            k->W[mode][i][k->nx + j] = S[i][CK_SRC_ROW(k, j)];
        k->W[mode][i][CK_Z_CLIP(k)] = k->op_o ? S[i][CK_OP_ROW(k)] : 0.0;
        for (j = 0; j < k->nnl; ++j)
            k->SN[mode][i][j] = (k->nl[j].b ? S[i][k->nl[j].b - 1] : 0.0) - (k->nl[j].a ? S[i][k->nl[j].a - 1] : 0.0);
    }
    for (i = 0; i < k->nx; ++i) {
        const int a = k->x[i].a, b = k->x[i].b;
        for (j = 0; j < k->nz; ++j)
            k->Wx[mode][i][j] = (a ? k->W[mode][a - 1][j] : 0.0) - (b ? k->W[mode][b - 1][j] : 0.0);
        for (j = 0; j < k->nnl; ++j)
            k->SNx[mode][i][j] = (a ? k->SN[mode][a - 1][j] : 0.0) - (b ? k->SN[mode][b - 1][j] : 0.0);
    }
    for (i = 0; i < k->nctl; ++i) {
        const int a = k->ctl_a[i], b = k->ctl_b[i];
        for (j = 0; j < k->nz; ++j)
            k->Wv[mode][i][j] = (a ? k->W[mode][a - 1][j] : 0.0) - (b ? k->W[mode][b - 1][j] : 0.0);
        for (j = 0; j < k->nnl; ++j)
            k->Kv[mode][i][j] = (a ? k->SN[mode][a - 1][j] : 0.0) - (b ? k->SN[mode][b - 1][j] : 0.0);
    }
}

/* (Re)builds the solver after the netlist or the sample rate changed. */
static void ck_build(Circuit* k, double sample_rate)
{
    int i;
    k->T = 1.0 / sample_rate;
    k->dim = k->nodes + k->nsrc + (k->op_o ? 1 : 0) + (k->op2_o ? 1 : 0) + (k->buf_out ? 1 : 0);
    k->nz = k->nx + k->nsrc + 1;
    /* Control voltages: a diode pair's own voltage; a JFET's gate-source and drain-source. */
    k->nctl = 0;
    for (i = 0; i < k->nnl; ++i) {
        const CkN* n = &k->nl[i];
        if (n->type == NL_DIODES) {
            k->ctl_a[k->nctl] = n->a;
            k->ctl_b[k->nctl++] = n->b;
        } else {
            k->ctl_a[k->nctl] = n->c;
            k->ctl_b[k->nctl++] = n->b;
            k->ctl_a[k->nctl] = n->a;
            k->ctl_b[k->nctl++] = n->b;
        }
    }
    ck_build_mode(k, 0);
    if (k->op_o) {
        ck_build_mode(k, 1);
        ck_build_mode(k, 2);
    }
}

static void ck_reset(Circuit* k)
{
    memset(k->q, 0, sizeof k->q);
    memset(k->v, 0, sizeof k->v);
    memset(k->z, 0, sizeof k->z);
    memset(k->inl, 0, sizeof k->inl);
    k->mode = 0;
}

static inline double ck_dot(const double* a, const double* b, int n)
{
    double s = 0.0;
    int i;
    for (i = 0; i < n; ++i)
        s += a[i] * b[i];
    return s;
}

/* Any unknown (a node voltage for i = node - 1) after the last ck_step. */
static inline double ck_unknown(const Circuit* k, int i)
{
    return ck_dot(k->W[k->mode][i], k->z, k->nz) + ck_dot(k->SN[k->mode][i], k->inl, k->nnl);
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

/* n-channel JFET drain current (drain to source) and its slopes in vgs and vds. Symmetric:
 * with vds negative, the drain and source swap roles. A tiny leakage keeps the Jacobian
 * regular in cut-off. */
static inline double ck_jfet_i(const CkN* n, double vgs, double vds, double* dg, double* dd)
{
    const double gmin = 1e-9;
    double vov, id;
    if (vds < 0.0) {
        /* Mirror: source and drain swap, so the controlling voltage is vgd = vgs - vds. */
        const double vgd = vgs - vds, vsd = -vds;
        double g2, d2;
        vov = vgd - n->vp;
        if (vov <= 0.0) {
            id = 0.0;
            g2 = d2 = 0.0;
        } else if (vsd < vov) {
            id = n->beta * (2.0 * vov * vsd - vsd * vsd);
            g2 = n->beta * 2.0 * vsd;
            d2 = n->beta * (2.0 * vov - 2.0 * vsd);
        } else {
            id = n->beta * vov * vov;
            g2 = n->beta * 2.0 * vov;
            d2 = 0.0;
        }
        /* id(vgs, vds) = -f(vgs - vds, -vds) */
        *dg = -g2;
        *dd = g2 + d2;
        return -id + gmin * vds;
    }
    vov = vgs - n->vp;
    if (vov <= 0.0) {
        *dg = 0.0;
        *dd = gmin;
        return gmin * vds;
    }
    if (vds < vov) {
        id = n->beta * (2.0 * vov * vds - vds * vds);
        *dg = n->beta * 2.0 * vds;
        *dd = n->beta * (2.0 * vov - 2.0 * vds) + gmin;
    } else {
        id = n->beta * vov * vov;
        *dg = n->beta * 2.0 * vov;
        *dd = gmin;
    }
    return id + gmin * vds;
}

/* Solves the nonlinear devices for k->z in this mode; leaves their currents in k->inl. */
static void ck_nl_solve(Circuit* k, int mode)
{
    double p[CK_MAX_CTL], v[CK_MAX_CTL];
    const int nc = k->nctl, ni = k->nnl;
    int i, j, it;
    if (!ni)
        return;
    for (i = 0; i < nc; ++i) {
        p[i] = ck_dot(k->Wv[mode][i], k->z, k->nz);
        v[i] = p[i] + ck_dot(k->Kv[mode][i], k->inl, ni); /* exact if the currents haven't moved */
        if (fabs(v[i] - k->v[i]) > 0.5)
            v[i] = k->v[i];
    }
    for (it = 0; it < 80; ++it) {
        double f[CK_MAX_CTL], J[CK_MAX_CTL][CK_MAX_CTL + 1], G[CK_MAX_NL][CK_MAX_CTL], cur[CK_MAX_NL];
        double worst = 0.0;
        int c = 0;
        memset(G, 0, sizeof G);
        for (j = 0; j < ni; ++j) {
            const CkN* n = &k->nl[j];
            if (n->type == NL_DIODES) {
                cur[j] = ck_diode_i(v[c], &G[j][c]);
                c += 1;
            } else {
                cur[j] = ck_jfet_i(n, v[c], v[c + 1], &G[j][c], &G[j][c + 1]);
                c += 2;
            }
        }
        /* F(v) = v - p - Kv i(v); J = I - Kv G */
        for (i = 0; i < nc; ++i) {
            f[i] = v[i] - p[i] - ck_dot(k->Kv[mode][i], cur, ni);
            for (c = 0; c < nc; ++c) {
                double s = 0.0;
                for (j = 0; j < ni; ++j)
                    s += k->Kv[mode][i][j] * G[j][c];
                J[i][c] = (i == c ? 1.0 : 0.0) - s;
            }
            J[i][nc] = f[i];
        }
        /* Gaussian elimination with partial pivoting on the small system. */
        for (c = 0; c < nc; ++c) {
            int piv = c;
            for (i = c + 1; i < nc; ++i)
                if (fabs(J[i][c]) > fabs(J[piv][c]))
                    piv = i;
            if (piv != c)
                for (j = 0; j <= nc; ++j) {
                    const double t = J[c][j];
                    J[c][j] = J[piv][j];
                    J[piv][j] = t;
                }
            if (fabs(J[c][c]) < 1e-300)
                J[c][c] = 1e-300;
            for (i = c + 1; i < nc; ++i) {
                const double m = J[i][c] / J[c][c];
                for (j = c; j <= nc; ++j)
                    J[i][j] -= m * J[c][j];
            }
        }
        for (i = nc - 1; i >= 0; --i) {
            double s = J[i][nc];
            for (j = i + 1; j < nc; ++j)
                s -= J[i][j] * J[j][nc];
            J[i][nc] = s / J[i][i];
        }
        c = 0;
        for (j = 0; j < ni; ++j) {
            const double lim = k->nl[j].type == NL_DIODES ? 0.1 : 1.0;
            const int nn = k->nl[j].type == NL_DIODES ? 1 : 2;
            for (i = c; i < c + nn; ++i) {
                const double step = clampd(J[i][nc], -lim, lim);
                v[i] -= step;
                if (fabs(step) > worst)
                    worst = fabs(step);
            }
            c += nn;
        }
        if (worst < 1e-7) /* -140 dB re 1 V */
            break;
    }
    {
        int c = 0;
        for (j = 0; j < ni; ++j) {
            double g1, g2;
            if (k->nl[j].type == NL_DIODES) {
                k->inl[j] = ck_diode_i(v[c], &g1);
                c += 1;
            } else {
                k->inl[j] = ck_jfet_i(&k->nl[j], v[c], v[c + 1], &g1, &g2);
                c += 2;
            }
        }
    }
    memcpy(k->v, v, sizeof(double) * (size_t)nc);
}

/* One sample. u[] are the caller-driven source voltages, in the order added. The op amp clips
 * at op_lo/op_hi: if the solution goes past a limit, the output is pinned there instead (the
 * inverting input then no longer follows). A second op amp, used only in a mixer, is clipped
 * by the caller. */
static void ck_step(Circuit* k, const double* u)
{
    int i, n = 0;
    for (i = 0; i < k->nx; ++i)
        k->z[i] = k->q[i];
    for (i = 0; i < k->nsrc; ++i)
        k->z[k->nx + i] = k->src[i].is_dc ? k->src[i].dc : u[n++];
    k->z[CK_Z_CLIP(k)] = 0.0;
    if (!k->ok[0])
        return;
    k->mode = 0;
    ck_nl_solve(k, 0);
    if (k->op_o && !k->noclip) {
        const double vo = ck_unknown(k, k->op_o - 1);
        if (vo > k->op_hi || vo < k->op_lo) {
            const int mode = vo > k->op_hi ? 1 : 2;
            k->z[CK_Z_CLIP(k)] = mode == 1 ? k->op_hi : k->op_lo;
            if (k->ok[mode]) {
                k->mode = mode;
                ck_nl_solve(k, mode);
            }
        }
    }
    for (i = 0; i < k->nx; ++i) {
        const double g = ck_xg(k, i);
        const double vx = ck_dot(k->Wx[k->mode][i], k->z, k->nz) + ck_dot(k->SNx[k->mode][i], k->inl, k->nnl);
        /* Capacitor: i = g v - h, next h = g v + i. Inductor: i = g v + h, next h = g v + i.
         * With q the source term (current a->b = g v + q): */
        k->q[i] = k->x[i].ind ? 2.0 * g * vx + k->q[i] : -2.0 * g * vx - k->q[i];
    }
}

/* Starts a stage at its DC operating point: steps it with a very long time step (capacitors
 * nearly open, so the bias settles), then sets each capacitor's history so it carries no
 * current at the real rate. Circuits with inductors are AC only and start from rest. */
static void ck_settle(Circuit* k, double sample_rate, const double* u)
{
    double vdc[CK_MAX_C];
    int i;
    ck_reset(k);
    for (i = 0; i < k->nx; ++i)
        if (k->x[i].ind) {
            ck_build(k, sample_rate);
            return;
        }
    ck_build(k, 1e-9);
    /* With histories cleared, each long step is a plain DC solve (capacitors all but open);
     * repeating it lets the nonlinear devices converge from rest. (Keeping the trapezoidal
     * histories would ring, sign flipping every step, instead of settling.) */
    for (i = 0; i < 8; ++i) {
        memset(k->q, 0, sizeof k->q);
        ck_step(k, u);
    }
    for (i = 0; i < k->nx; ++i)
        vdc[i] = ck_dot(k->Wx[k->mode][i], k->z, k->nz) + ck_dot(k->SNx[k->mode][i], k->inl, k->nnl);
    ck_build(k, sample_rate);
    for (i = 0; i < k->nx; ++i)
        k->q[i] = -ck_xg(k, i) * vdc[i]; /* a capacitor at its DC voltage carries no current */
    ck_step(k, u); /* so the node voltages read back at the operating point */
}

#endif

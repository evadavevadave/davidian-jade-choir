/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* The amplifier's stages as netlists, with the part values of the original 1984 head's
 * circuit diagram (preamp board 70126811, input board 70126820). Part numbers (R23, C49, VR4...)
 * are the schematic's. Every stage ends at an op amp output (or a node with no load), so the
 * stages are solved one after another. */
#ifndef JD_AMP_H
#define JD_AMP_H

#include "circuit.h"

#define K_ 1e3
#define M_ 1e6
#define U_ 1e-6
#define N_ 1e-9
#define P_ 1e-12

/* Pot tapers. "B" is linear. "A" (audio) and "C" (reverse audio) follow the usual Japanese
 * curves: 15% of the track at half rotation for A, 85% for C. */
#define TAPER_A_BASE 32.1111 /* (1/0.15 - 1)^2, so taper_a(0.5) = 0.15 */
static inline double taper_a(double x)
{
    x = clampd(x, 0.0, 1.0);
    return (pow(TAPER_A_BASE, x) - 1.0) / (TAPER_A_BASE - 1.0);
}

/* A pot track section never quite reaches zero ohms. */
#define POT_MIN 1.0
static inline double pot(double r) { return r < POT_MIN ? POT_MIN : r; }

/* A closed switch contact or a shorted jack. */
#define SHORT_R 0.1

typedef struct {
    double volume, treble, middle, bass, hitreble, distortion; /* rotations 0-1 */
    int low_input;   /* LOW jack instead of HIGH */
    int dist_fs;     /* distortion footswitch on (or no footswitch plugged in) */
    int dist_switch; /* VR1's switch: closed unless the knob is at OFF */
} PreampSet;

/* ---- Input jacks and IC2b, the first gain stage ----
 * HIGH: R6 33k in series. LOW: R7 100k in series, with R6 to ground through the unused HIGH
 * jack's switch contact. C12 couples into IC2b (+), biased by R20 1M; R19/C19 stop RF.
 * IC2b gain 1 + R18/R16 (100k/39k), with C20 across R18. */
enum { IN_G = 1, IN_N2, IN_N3, IN_P, IN_M, IN_O, IN_NODES = IN_O };

static void build_input(Circuit* k, const PreampSet* s, double fs)
{
    ck_begin(k, IN_NODES);
    ck_src(k, IN_G);
    if (s->low_input) {
        ck_r(k, IN_G, IN_N2, 100 * K_); /* R7 */
        ck_r(k, IN_N2, 0, 33 * K_);     /* R6 */
    } else {
        ck_r(k, IN_G, IN_N2, 33 * K_);  /* R6 */
    }
    ck_c(k, IN_N2, IN_N3, 0.082 * U_);  /* C12 */
    ck_r(k, IN_N3, IN_P, 1 * K_);       /* R19 */
    ck_c(k, IN_P, 0, 100 * P_);         /* C19 */
    ck_r(k, IN_P, 0, 1 * M_);           /* R20 */
    ck_r(k, IN_M, 0, 39 * K_);          /* R16 */
    ck_r(k, IN_M, IN_O, 100 * K_);      /* R18 */
    ck_c(k, IN_M, IN_O, 47 * P_);       /* C20 */
    ck_opamp(k, IN_P, IN_M, IN_O);      /* IC2b */
    ck_build(k, fs);
}

/* ---- IC2a (distortion gain), the diode clippers, the tone stack and the volume pot ----
 * IC2a: C23 in, R31 1M bias, gain 1 + R23 47k / (R22 1k + C18 + VR1 50kC), C24 100p across
 * R23. The VR1 leg returns to ground through the distortion footswitch jack (pin 6).
 * Clippers: C30, R34, then R32 3.9k to D9/D11 + C32 0.082u, R43 3.9k to D8/D10 + C33 1u, and
 * C31 0.01u across the output. Their common return is grounded through VR1's switch and the
 * footswitch jack; with the switch at OFF they float on R169 1.5M.
 * Tone stack: R74 100k, C49 150p, C50 0.082u, C51 0.047u, treble VR4 250kB, bass VR6 250kA +
 * R97 1k, middle VR5 10kB. Volume VR2 1MB fed through R42 100k, with the hi-treble path
 * C42 470p + VR3 1MB from the treble wiper to the volume wiper. C37 and R47 1M into IC3a. */
enum {
    DR_S = 1, DR_P, DR_M, DR_N4, DR_N5, DR_N6, DR_O, DR_X, DR_A, DR_DA, DR_B, DR_DB, DR_L,
    DR_TM, DR_T10, DR_W4, DR_PN, DR_Q, DR_V11, DR_W2, DR_H13, DR_OUT, DR_NODES = DR_OUT
};

static void build_drive(Circuit* k, const PreampSet* s, double fs)
{
    const int dist_on = s->dist_switch; /* VR1's switch, open at the OFF end */
    ck_begin(k, DR_NODES);
    ck_src(k, DR_S);
    /* IC2a */
    ck_c(k, DR_S, DR_P, 0.082 * U_);          /* C23 */
    ck_r(k, DR_P, 0, 1 * M_);                 /* R31 */
    ck_r(k, DR_M, DR_O, 47 * K_);             /* R23 */
    ck_c(k, DR_M, DR_O, 100 * P_);            /* C24 */
    ck_r(k, DR_M, DR_N4, 1 * K_);             /* R22 */
    ck_c(k, DR_N4, DR_N5, 10 * U_);           /* C18 */
    ck_r(k, DR_N5, DR_N6, pot(50 * K_ * taper_a(1.0 - s->distortion))); /* VR1 50kC */
    if (s->dist_fs)
        ck_r(k, DR_N6, 0, SHORT_R);           /* footswitch jack */
    if (dist_on)
        ck_r(k, DR_N6, DR_L, SHORT_R);        /* VR1 switch */
    ck_opamp(k, DR_P, DR_M, DR_O);
    /* Clippers */
    ck_c(k, DR_O, DR_X, 0.082 * U_);          /* C30 */
    ck_r(k, DR_X, 0, 270 * K_);               /* R34 */
    ck_r(k, DR_X, DR_A, 3.9 * K_);            /* R32 */
    ck_diodes(k, DR_A, DR_DA);                /* D9, D11 */
    ck_c(k, DR_DA, DR_L, 0.082 * U_);         /* C32 */
    ck_r(k, DR_A, DR_B, 3.9 * K_);            /* R43 */
    ck_c(k, DR_B, DR_L, 0.01 * U_);           /* C31 */
    ck_diodes(k, DR_B, DR_DB);                /* D8, D10 */
    ck_c(k, DR_DB, DR_L, 1 * U_);             /* C33 */
    ck_r(k, DR_L, 0, 1.5 * M_);               /* R169 */
    /* Tone stack */
    ck_r(k, DR_B, DR_TM, 100 * K_);           /* R74 */
    ck_c(k, DR_B, DR_T10, 150 * P_);          /* C49 */
    ck_c(k, DR_TM, DR_PN, 0.082 * U_);        /* C50 */
    ck_c(k, DR_TM, DR_Q, 0.047 * U_);         /* C51 */
    ck_r(k, DR_T10, DR_W4, pot(250 * K_ * (1.0 - s->treble))); /* VR4 250kB */
    ck_r(k, DR_W4, DR_PN, pot(250 * K_ * s->treble));
    ck_r(k, DR_PN, DR_Q, pot(250 * K_ * taper_a(s->bass)) + 1 * K_); /* VR6 250kA + R97 */
    ck_r(k, DR_Q, 0, pot(10 * K_ * s->middle));                      /* VR5 10kB */
    /* Volume and hi-treble */
    ck_r(k, DR_W4, DR_V11, 100 * K_);         /* R42 */
    ck_r(k, DR_V11, DR_W2, pot(1 * M_ * (1.0 - s->volume))); /* VR2 1MB */
    ck_r(k, DR_W2, 0, pot(1 * M_ * s->volume));
    ck_c(k, DR_W4, DR_H13, 470 * P_);         /* C42 */
    ck_r(k, DR_H13, DR_W2, pot(1 * M_ * (1.0 - s->hitreble))); /* VR3 1MB */
    ck_c(k, DR_W2, DR_OUT, 0.082 * U_);       /* C37 */
    ck_r(k, DR_OUT, 0, 1 * M_);               /* R47 */
    ck_build(k, fs);
}

/* ---- IC3a: gain 1 + R46 270k / R48 22k, C38 100p across R46 ---- */
enum { A3_S = 1, A3_M, A3_O, A3_NODES = A3_O };

static void build_ic3a(Circuit* k, double fs)
{
    ck_begin(k, A3_NODES);
    ck_src(k, A3_S);
    ck_r(k, A3_M, 0, 22 * K_);       /* R48 */
    ck_r(k, A3_M, A3_O, 270 * K_);   /* R46 */
    ck_c(k, A3_M, A3_O, 100 * P_);   /* C38 */
    ck_opamp(k, A3_S, A3_M, A3_O);
    ck_build(k, fs);
}

/* ---- IC3b: non-inverting. C40 and R65 270k, then R59 33k into the junction with R58 100k
 * (the SUB IN jack's input, grounded by the jack when unused), C41 into (+) with R45 270k to
 * ground (D13/D14 only clamp it). Gain 1 + R57 22k / R49 27k, C39 47p across R57. */
enum { B3_S = 1, B3_A, B3_J, B3_P, B3_M, B3_O, B3_NODES = B3_O };

static void build_ic3b(Circuit* k, double fs)
{
    ck_begin(k, B3_NODES);
    ck_src(k, B3_S);
    ck_c(k, B3_S, B3_A, 1 * U_);     /* C40 */
    ck_r(k, B3_A, 0, 270 * K_);      /* R65 */
    ck_r(k, B3_A, B3_J, 33 * K_);    /* R59 */
    ck_r(k, B3_J, 0, 100 * K_);      /* R58, SUB IN */
    ck_c(k, B3_J, B3_P, 1 * U_);     /* C41 */
    ck_r(k, B3_P, 0, 270 * K_);      /* R45 */
    ck_r(k, B3_M, 0, 27 * K_);       /* R49 */
    ck_r(k, B3_M, B3_O, 22 * K_);    /* R57 */
    ck_c(k, B3_M, B3_O, 47 * P_);    /* C39 */
    ck_opamp(k, B3_P, B3_M, B3_O);
    ck_build(k, fs);
}

/* ---- Chorus input filter and IC5a, before the BBD ----
 * C58, R110 270k to the BBD bias (decoupled by C82 100u), R109 5.6k / C64 2.2n,
 * R108 56k / C65 220p into IC5a (+). IC5a gain 1 + R123 22k / (R121 22k || (R122 5.6k + C71)),
 * a treble pre-emphasis, with C73 47p across R123. */
enum { CP_S = 1, CP_A, CP_B, CP_P, CP_M, CP_C, CP_O, CP_NODES = CP_O };

static void build_chorus_pre(Circuit* k, double fs)
{
    ck_begin(k, CP_NODES);
    ck_src(k, CP_S);
    ck_c(k, CP_S, CP_A, 0.047 * U_); /* C58 */
    ck_r(k, CP_A, 0, 270 * K_);      /* R110 */
    ck_r(k, CP_A, CP_B, 5.6 * K_);   /* R109 */
    ck_c(k, CP_B, 0, 2.2 * N_);      /* C64 */
    ck_r(k, CP_B, CP_P, 56 * K_);    /* R108 */
    ck_c(k, CP_P, 0, 220 * P_);      /* C65 */
    ck_r(k, CP_M, 0, 22 * K_);       /* R121 */
    ck_r(k, CP_M, CP_C, 5.6 * K_);   /* R122 */
    ck_c(k, CP_C, 0, 0.01 * U_);     /* C71 */
    ck_r(k, CP_M, CP_O, 22 * K_);    /* R123 */
    ck_c(k, CP_M, CP_O, 47 * P_);    /* C73 */
    ck_opamp(k, CP_P, CP_M, CP_O);
    ck_build(k, fs);
}

/* ---- After the BBD: C79 and R132 100k, a third-order filter (R133, R131, R129 8.2k,
 * C76 2.2n, C74 8.2n to Q27's emitter, C75 470p) with Q27 as the follower, then C72,
 * R120 100k / R128 33k into IC5b, whose feedback R107 47k || (R119 8.2k + C70 4.7n) over
 * R111 47k is the de-emphasis. */
enum {
    CO_S = 1, CO_A, CO_B, CO_C, CO_D, CO_E, CO_F, CO_P, CO_M, CO_G, CO_O, CO_NODES = CO_O
};

static void build_chorus_post(Circuit* k, double fs)
{
    ck_begin(k, CO_NODES);
    ck_src(k, CO_S);
    ck_c(k, CO_S, CO_A, 0.047 * U_); /* C79 */
    ck_r(k, CO_A, 0, 100 * K_);      /* R132 */
    ck_r(k, CO_A, CO_B, 8.2 * K_);   /* R133 */
    ck_c(k, CO_B, 0, 2.2 * N_);      /* C76 */
    ck_r(k, CO_B, CO_C, 8.2 * K_);   /* R131 */
    ck_c(k, CO_C, CO_E, 8.2 * N_);   /* C74 */
    ck_r(k, CO_C, CO_D, 8.2 * K_);   /* R129 */
    ck_c(k, CO_D, 0, 470 * P_);      /* C75 */
    ck_buffer(k, CO_D, CO_E);        /* Q27 */
    ck_r(k, CO_E, 0, 10 * K_);       /* R127 */
    ck_c(k, CO_E, CO_F, 0.047 * U_); /* C72 */
    ck_r(k, CO_F, CO_P, 100 * K_);   /* R120 */
    ck_r(k, CO_P, 0, 33 * K_);       /* R128 */
    ck_r(k, CO_M, 0, 47 * K_);       /* R111 */
    ck_r(k, CO_M, CO_O, 47 * K_);    /* R107 */
    ck_r(k, CO_M, CO_G, 8.2 * K_);   /* R119 */
    ck_c(k, CO_G, CO_O, 4.7 * N_);   /* C70 */
    ck_opamp(k, CO_P, CO_M, CO_O);   /* IC5b */
    ck_build(k, fs);
}

/* ---- Reverb send: C34 and R29 270k into the driver IC1b ---- */
enum { RS_S = 1, RS_A, RS_NODES = RS_A };

static void build_send(Circuit* k, double fs)
{
    ck_begin(k, RS_NODES);
    ck_src(k, RS_S);
    ck_c(k, RS_S, RS_A, 0.047 * U_); /* C34 */
    ck_r(k, RS_A, 0, 270 * K_);      /* R29 */
    ck_build(k, fs);
}

/* ---- Reverb recovery: C52, R81 15k, IC4b gain 1 + R79 56k / R76 8.2k, C46 47p ---- */
enum { RR_S = 1, RR_A, RR_M, RR_O, RR_NODES = RR_O };

static void build_return(Circuit* k, double fs)
{
    ck_begin(k, RR_NODES);
    ck_src(k, RR_S);
    ck_c(k, RR_S, RR_A, 0.082 * U_); /* C52 */
    ck_r(k, RR_A, 0, 15 * K_);       /* R81 */
    ck_r(k, RR_M, 0, 8.2 * K_);      /* R76 */
    ck_r(k, RR_M, RR_O, 56 * K_);    /* R79 */
    ck_c(k, RR_M, RR_O, 47 * P_);    /* C46 */
    ck_opamp(k, RR_A, RR_M, RR_O);
    ck_build(k, fs);
}

/* ---- The two power amp mixers, IC1a (left, "normal") and IC4a (right, "effect") ----
 * Dry (from IC3b through C96): R66 47k to IC1a, and through the JFET switch Q17 and R73 47k to
 * IC4a. Reverb: C55, R87 1k, VR7 10kB (its top is the reverb footswitch jack), wiper to R67
 * 100k (IC1a) and R93 100k (IC4a). Chorus/vibrato: C63, R102 270k, then the JFET switch Q21
 * and R95 47k to IC4a. Feedback R27 / R75 100k with C17 / C47 47p.
 * Each JFET switch is a 47k branch plus its channel resistance, ron / s, where s (0-1) is how
 * far the channel is turned on. */
enum {
    MX_D = 1, MX_RV, MX_WT, MX_D1, MX_ML, MX_OL, MX_R1, MX_T7, MX_W7, MX_MR, MX_OR, MX_C63,
    MX_NODES = MX_C63
};

#define JFET_RON 100.0 /* 2SK117 channel resistance, fully on (estimate; see README) */

typedef struct {
    double reverb;   /* VR7 rotation */
    int reverb_fs;   /* reverb footswitch on */
    double s_dry;    /* Q17 conduction */
    double s_wet;    /* Q21 conduction */
} MixSet;

static void build_mixer(Circuit* k, const MixSet* s, double fs)
{
    ck_begin(k, MX_NODES);
    ck_src(k, MX_D);
    ck_src(k, MX_RV);
    ck_src(k, MX_WT);
    ck_c(k, MX_D, MX_D1, 1 * U_);             /* C96 */
    ck_r(k, MX_D1, MX_ML, 47 * K_);           /* R66 */
    if (s->s_dry > 1e-4)
        ck_r(k, MX_D1, MX_MR, 47 * K_ + JFET_RON / s->s_dry); /* Q17 + R73 */
    ck_r(k, MX_ML, MX_OL, 100 * K_);          /* R27 */
    ck_c(k, MX_ML, MX_OL, 47 * P_);           /* C17 */
    ck_c(k, MX_RV, MX_R1, 0.082 * U_);        /* C55 */
    ck_r(k, MX_R1, MX_T7, 1 * K_);            /* R87 */
    ck_r(k, MX_T7, MX_W7, pot(10 * K_ * (1.0 - s->reverb))); /* VR7 10kB */
    ck_r(k, MX_W7, 0, pot(10 * K_ * s->reverb));
    if (!s->reverb_fs)
        ck_r(k, MX_T7, 0, SHORT_R);           /* footswitch */
    ck_r(k, MX_W7, MX_ML, 100 * K_);          /* R67 */
    ck_r(k, MX_W7, MX_MR, 100 * K_);          /* R93 */
    ck_r(k, MX_MR, MX_OR, 100 * K_);          /* R75 */
    ck_c(k, MX_MR, MX_OR, 47 * P_);           /* C47 */
    ck_c(k, MX_WT, MX_C63, 0.047 * U_);       /* C63 */
    ck_r(k, MX_C63, 0, 270 * K_);             /* R102 */
    if (s->s_wet > 1e-4)
        ck_r(k, MX_C63, MX_MR, 47 * K_ + JFET_RON / s->s_wet); /* Q21 + R95 */
    ck_opamp(k, 0, MX_ML, MX_OL);             /* IC1a, (+) on R28 */
    ck_opamp(k, 0, MX_MR, MX_OR);             /* IC4a, (+) on R80 */
    k->noclip = 1; /* both outputs are clipped by the caller */
    ck_build(k, fs);
}

/* ---- Line outputs: after the muting FET, C22 1u, R26 100k, and the power amp input (C21,
 * R35 1k, R36 56k, C29 100p) as the load. The line out jack hangs off this node through
 * R24 (R98) 5.6k; the model gives the open-circuit jack voltage. */
enum { LO_S = 1, LO_A, LO_B, LO_C, LO_NODES = LO_C };

static void build_lineout(Circuit* k, double fs)
{
    ck_begin(k, LO_NODES);
    ck_src(k, LO_S);
    ck_c(k, LO_S, LO_A, 1 * U_);      /* C22 / C59 */
    ck_r(k, LO_A, 0, 100 * K_);       /* R26 / R100 */
    ck_c(k, LO_A, LO_B, 0.082 * U_);  /* C21 / C60 */
    ck_r(k, LO_B, LO_C, 1 * K_);      /* R35 / R103 */
    ck_r(k, LO_C, 0, 56 * K_);        /* R36 / R105 */
    ck_c(k, LO_C, 0, 100 * P_);       /* C29 / C67 */
    ck_build(k, fs);
}

#endif

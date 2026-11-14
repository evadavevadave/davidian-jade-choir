/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* The amplifier's stages as netlists, with the part values of the original two-channel combo's
 * circuit diagram (service notes, fifth edition, 1983: pre-main boards CH-1 7011628000 and
 * CH-2 7011629000). Part numbers (R23, C49, VR4...) are the schematic's, per channel.
 * Stages are joined at low-impedance points (followers, op amp outputs) or at JFET gates,
 * which draw no current, so each stage is solved on its own. Stages with JFETs carry their
 * supply rails and start at their DC operating point. */
#ifndef JD_AMP_H
#define JD_AMP_H

#include "circuit.h"

#define K_ 1e3
#define M_ 1e6
#define U_ 1e-6
#define N_ 1e-9
#define P_ 1e-12

/* 2SK117-GR JFETs. The schematic prints most of their DC operating points, and JFETs of one
 * type vary widely, so each gets its own pinch-off voltage, worked out from its printed DC
 * point with IDSS at the GR grade's 6.5 mA (its range is 2.6-6.5 mA; VGS(off) -0.2 to -1.5 V):
 *   CH-1 Q1  +12.6 V drain on +30 V: 1.45 mA at -0.55 V   -> -1.043 V
 *   CH-2 Q1  +12.8 V drain on +26 V: 1.10 mA at -0.32 V   -> -0.544 V
 *   CH-2 Q2  +13.3 V source:         1.11 mA at -0.30 V   -> -0.511 V
 *   CH-2 Q3  +10.4 V drain:          1.04 mA at -0.31 V   -> -0.518 V
 *   CH-2 Q5  +5.17 V drain:          0.61 mA at -0.35 V   -> -0.505 V
 * CH-1 Q3 and CH-2 Q19 have no printed voltages: they take their board's other value (CH-1
 * Q1's, and the middle of CH-2's). Gate voltages come from the bias dividers (the printed
 * gate readings are loaded by the meter). test_dsp checks the printed signal levels these
 * give. */
#define JFET_IDSS 6.5e-3
#define VP_CH1_Q1 (-1.043)
#define VP_CH1_Q3 (-1.043)
#define VP_CH2_Q1 (-0.544)
#define VP_CH2_Q2 (-0.511)
#define VP_CH2_Q3 (-0.518)
#define VP_CH2_Q5 (-0.505)
#define VP_CH2_Q19 (-0.518)

/* Pot tapers. "B" is linear. "A" (audio) and "C" (reverse audio) follow the usual Japanese
 * curves: 15% of the track at half rotation for A, 85% for C. */
#define TAPER_A_BASE 32.1111 /* (1/0.15 - 1)^2, so taper_a(0.5) = 0.15 */
static inline double taper_a(double x)
{
    x = clampd(x, 0.0, 1.0);
    return (pow(TAPER_A_BASE, x) - 1.0) / (TAPER_A_BASE - 1.0);
}

#define POT_MIN 1.0
static inline double pot(double r) { return r < POT_MIN ? POT_MIN : r; }
#define SHORT_R 0.1

/* A JFET switch: its series resistor plus the channel, ron / s for conduction s (0-1). */
#define JFET_RON 100.0 /* channel resistance fully on (estimate; see README) */
static inline double sw_r(double r, double s) { return r + JFET_RON / (s > 1e-6 ? s : 1e-6); }

typedef struct {
    double volume, treble, middle, bass; /* rotations 0-1 */
    int bright;
    int low_input;
} ChanSet;

/* ================= Input, first stage and tone stack (both channels) =================
 * HIGH through R2 (CH-2: R4) 33k. LOW through R1 (R3) 68k, with the 33k grounded by the
 * unused HIGH jack's switch. C1 (C3) 100p. C3 (C1) 0.082u and R6 (R5) 1k to Q1's gate, biased
 * by R5 (R1) 1M from R7 (R6) 220k / R3 (R2) 6.8k. Q1: drain R9 12k, source R4 (R8) 1k, C4 (C2)
 * 47p drain to gate. Tone stack from the drain: C5 (C22) 150p, R36 100k, C20 (C23) 0.082u,
 * TREBLE VR2 250kB, BASS VR4 250kA + R43 (R158) 1k, MIDDLE VR3 10kB, and C26 0.056u (CH-2:
 * C27 0.047u).
 * CH-1 then: VOLUME VR1 1MB from the treble wiper, BRIGHT C6 330p (R10 1M across the
 * switch), C8 0.082u and R16 1k to Q3's gate, biased by R15 1M from R14 220k / R13 4.7k.
 * CH-2 then: C17 0.082u and R15 1k to Q2's gate, biased by R13 470k from R12 / R11 220k.
 * CH-2's preamp runs from the +26 V line (R7 1k from +30 V). */
enum {
    IN_G = 1, IN_N, IN_X, IN_GT, IN_D, IN_S, IN_VB, IN_RAIL, IN_T10, IN_W2, IN_TM, IN_PN, IN_Q,
    IN_W1, IN_BR, IN_Y, IN_OUT, IN_VB2, IN_NODES = IN_VB2
};

static void build_chan_in(Circuit* k, const ChanSet* s, int ch2, double fs, int settle)
{
    double u = 0.0;
    ck_begin(k, IN_NODES);
    ck_src(k, IN_G);
    ck_rail(k, IN_RAIL, ch2 ? 26.0 : 30.0);
    if (s->low_input) {
        ck_r(k, IN_G, IN_N, 68 * K_);
        ck_r(k, IN_N, 0, 33 * K_);
    } else {
        ck_r(k, IN_G, IN_N, 33 * K_);
    }
    ck_c(k, IN_N, 0, 100 * P_);
    ck_c(k, IN_N, IN_X, 0.082 * U_);
    ck_r(k, IN_X, IN_GT, 1 * K_);
    ck_r(k, IN_GT, IN_VB, 1 * M_);
    ck_r(k, IN_RAIL, IN_VB, 220 * K_);
    ck_r(k, IN_VB, 0, 6.8 * K_);
    ck_r(k, IN_RAIL, IN_D, 12 * K_);
    ck_r(k, IN_S, 0, 1 * K_);
    ck_c(k, IN_D, IN_GT, 47 * P_);
    ck_jfet(k, IN_D, IN_GT, IN_S, JFET_IDSS, ch2 ? VP_CH2_Q1 : VP_CH1_Q1);
    /* Tone stack */
    ck_c(k, IN_D, IN_T10, 150 * P_);
    ck_r(k, IN_D, IN_TM, 100 * K_);
    ck_c(k, IN_TM, IN_PN, 0.082 * U_);
    ck_c(k, IN_TM, IN_Q, (ch2 ? 0.047 : 0.056) * U_);
    ck_r(k, IN_T10, IN_W2, pot(250 * K_ * (1.0 - s->treble)));
    ck_r(k, IN_W2, IN_PN, pot(250 * K_ * s->treble));
    ck_r(k, IN_PN, IN_Q, pot(250 * K_ * taper_a(s->bass)) + 1 * K_);
    ck_r(k, IN_Q, 0, pot(10 * K_ * s->middle));
    if (!ch2) {
        ck_r(k, IN_W2, IN_W1, pot(1 * M_ * (1.0 - s->volume)));
        ck_r(k, IN_W1, 0, pot(1 * M_ * s->volume));
        ck_c(k, IN_W2, IN_BR, 330 * P_);
        ck_r(k, IN_BR, IN_W1, s->bright ? SHORT_R : 1 * M_);
        ck_c(k, IN_W1, IN_Y, 0.082 * U_);
        ck_r(k, IN_Y, IN_OUT, 1 * K_);
        ck_r(k, IN_OUT, IN_VB2, 1 * M_);
        ck_r(k, IN_RAIL, IN_VB2, 220 * K_);
        ck_r(k, IN_VB2, 0, 4.7 * K_);
    } else {
        ck_c(k, IN_W2, IN_Y, 0.082 * U_);
        ck_r(k, IN_Y, IN_OUT, 1 * K_);
        ck_r(k, IN_OUT, IN_VB2, 470 * K_);
        ck_r(k, IN_RAIL, IN_VB2, 220 * K_);
        ck_r(k, IN_VB2, 0, 220 * K_);
        /* No volume or bright on CH-2's input board: tie the unused nodes off. */
        ck_r(k, IN_W1, 0, 1 * M_);
        ck_r(k, IN_BR, 0, 1 * M_);
    }
    if (settle)
        ck_settle(k, fs, &u);
    else
        ck_build(k, fs);
}

/* ================= CH-1: Q3 and the Q2 emitter follower =================
 * Q3: drain R8 33k to +30 V, source R17 2.2k with R18 120 + C10 10u across it. Q2 follows the
 * drain (taken as unity gain); its emitter resistor is R12 12k. */
enum { Q3_G = 1, Q3_D, Q3_S, Q3_SB, Q3_E, Q3_RAIL, Q3_NODES = Q3_RAIL };

static void build_ch1_q3(Circuit* k, double fs, double gate_dc, int settle)
{
    ck_begin(k, Q3_NODES);
    ck_src(k, Q3_G);
    ck_rail(k, Q3_RAIL, 30.0);
    ck_r(k, Q3_RAIL, Q3_D, 33 * K_);
    ck_r(k, Q3_S, 0, 2.2 * K_);
    ck_r(k, Q3_S, Q3_SB, 120.0);
    ck_c(k, Q3_SB, 0, 10 * U_);
    ck_jfet(k, Q3_D, Q3_G, Q3_S, JFET_IDSS, VP_CH1_Q3);
    ck_buffer(k, Q3_D, Q3_E, 1.0);
    ck_r(k, Q3_E, 0, 12 * K_);
    if (settle)
        ck_settle(k, fs, &gate_dc);
    else
        ck_build(k, fs);
}

/* ================= CH-2: Q2 follower, distortion stage, VOLUME and BRIGHT =================
 * Q2 source follower (drain +26 V, source R10 12k). Clean path: C4 0.082u, R16 100k to the
 * VOLUME pot's top. Distortion: C6 0.082u and R17 33k to Q3's gate (R22 1M from R21 220k /
 * R20 47k), D1/D2 from the gate through C10 0.082u to the drain (clipping in the feedback
 * loop), C11 100p drain to gate. Q3: drain R23 15k to +26 V, source R24 4.7k, and
 * C43 1u + R29 330 + DIST VR5 10kC to ground. Out through C12 0.082u, R28 10k, the JFET switch
 * Q4 (R27 10k, R26 1M to its control line) and C15 0.082u to the VOLUME pot's top.
 * VOLUME VR1 1MB, BRIGHT C5 220p (R14 1M across the switch), then C16 0.082u and R30 1k to
 * Q5's gate, biased by R40 1M from R50 220k (+12.5 V) / R39 4.7k. */
enum {
    DR_G = 1, DR_S2, DR_C4, DR_C6, DR_G3, DR_VB3, DR_L, DR_D3, DR_S3, DR_C43, DR_R29,
    DR_C12, DR_M, DR_SW, DR_V1T, DR_W1, DR_BR, DR_C16, DR_OUT, DR_VB5, DR_RAIL, DR_RAIL5,
    DR_NODES = DR_RAIL5
};

typedef struct {
    double volume, distortion; /* rotations */
    int bright;
    double s_q4;               /* Q4 conduction 0-1 */
} DriveSet;

static void build_ch2_drive(Circuit* k, const DriveSet* s, double fs, double gate_dc, int settle)
{
    ck_begin(k, DR_NODES);
    ck_src(k, DR_G);
    ck_rail(k, DR_RAIL, 26.0);
    ck_rail(k, DR_RAIL5, 12.5);
    /* Q2 */
    ck_jfet(k, DR_RAIL, DR_G, DR_S2, JFET_IDSS, VP_CH2_Q2);
    ck_r(k, DR_S2, 0, 12 * K_);
    /* Clean path */
    ck_c(k, DR_S2, DR_C4, 0.082 * U_);
    ck_r(k, DR_C4, DR_V1T, 100 * K_);
    /* Distortion stage */
    ck_c(k, DR_S2, DR_C6, 0.082 * U_);
    ck_r(k, DR_C6, DR_G3, 33 * K_);
    ck_r(k, DR_G3, DR_VB3, 1 * M_);
    ck_r(k, DR_RAIL, DR_VB3, 220 * K_);
    ck_r(k, DR_VB3, 0, 47 * K_);
    ck_diodes(k, DR_G3, DR_L);
    ck_c(k, DR_L, DR_D3, 0.082 * U_);
    ck_c(k, DR_D3, DR_G3, 100 * P_);
    ck_r(k, DR_RAIL, DR_D3, 15 * K_);
    ck_r(k, DR_S3, 0, 4.7 * K_);
    ck_c(k, DR_S3, DR_C43, 1 * U_);
    ck_r(k, DR_C43, DR_R29, 330.0);
    ck_r(k, DR_R29, 0, pot(10 * K_ * taper_a(1.0 - s->distortion)));
    ck_jfet(k, DR_D3, DR_G3, DR_S3, JFET_IDSS, VP_CH2_Q3);
    ck_c(k, DR_D3, DR_C12, 0.082 * U_);
    ck_r(k, DR_C12, DR_M, 10 * K_);
    ck_r(k, DR_M, 0, 10 * K_);                       /* R27, to the control line (AC ground) */
    ck_r(k, DR_M, DR_SW, sw_r(0.0, s->s_q4));        /* Q4 */
    ck_r(k, DR_SW, 0, 1 * M_);                       /* R26 */
    ck_c(k, DR_SW, DR_V1T, 0.082 * U_);
    /* VOLUME, BRIGHT, to Q5 */
    ck_r(k, DR_V1T, DR_W1, pot(1 * M_ * (1.0 - s->volume)));
    ck_r(k, DR_W1, 0, pot(1 * M_ * s->volume));
    ck_c(k, DR_V1T, DR_BR, 220 * P_);
    ck_r(k, DR_BR, DR_W1, s->bright ? SHORT_R : 1 * M_);
    ck_c(k, DR_W1, DR_C16, 0.082 * U_);
    ck_r(k, DR_C16, DR_OUT, 1 * K_);
    ck_r(k, DR_OUT, DR_VB5, 1 * M_);
    ck_r(k, DR_RAIL5, DR_VB5, 220 * K_);
    ck_r(k, DR_VB5, 0, 4.7 * K_);
    if (settle)
        ck_settle(k, fs, &gate_dc);
    else
        ck_build(k, fs);
}

/* ================= CH-2: Q5, the effect sends and Q19 =================
 * Q5: drain R80 12k to +12.5 V, source R35 1k (C21 not fitted). From the drain: C52 0.082u,
 * R96 1k to Q16 (R99 1M); R81 100k, C36 0.47u and R62 100k to the reverb driver; C55 0.047u,
 * R106 1k to Q19 (R107 1M from R109 / R108 220k), a source follower with R110 12k feeding the
 * chorus. The reverb footswitch grounds the send. */
enum {
    Q5_G = 1, Q5_D, Q5_S, Q5_C52, Q5_G16, Q5_R81, Q5_REV, Q5_C55, Q5_G19, Q5_VB19, Q5_S19,
    Q5_RAIL, Q5_NODES = Q5_RAIL
};

static void build_ch2_q5(Circuit* k, int reverb_fs, double fs, double gate_dc, int settle)
{
    ck_begin(k, Q5_NODES);
    ck_src(k, Q5_G);
    ck_rail(k, Q5_RAIL, 12.5);
    ck_r(k, Q5_RAIL, Q5_D, 12 * K_);
    ck_r(k, Q5_S, 0, 1 * K_);
    ck_jfet(k, Q5_D, Q5_G, Q5_S, JFET_IDSS, VP_CH2_Q5);
    ck_c(k, Q5_D, Q5_C52, 0.082 * U_);
    ck_r(k, Q5_C52, Q5_G16, 1 * K_);
    ck_r(k, Q5_G16, 0, 1 * M_);
    ck_r(k, Q5_D, Q5_R81, 100 * K_);
    if (!reverb_fs)
        ck_r(k, Q5_R81, 0, SHORT_R);
    ck_c(k, Q5_R81, Q5_REV, 0.47 * U_);
    ck_r(k, Q5_REV, 0, 100 * K_);
    ck_c(k, Q5_D, Q5_C55, 0.047 * U_);
    ck_r(k, Q5_C55, Q5_G19, 1 * K_);
    ck_r(k, Q5_G19, Q5_VB19, 1 * M_);
    ck_r(k, Q5_RAIL, Q5_VB19, 220 * K_);
    ck_r(k, Q5_VB19, 0, 220 * K_);
    ck_jfet(k, Q5_RAIL, Q5_G19, Q5_S19, JFET_IDSS, VP_CH2_Q19);
    ck_r(k, Q5_S19, 0, 12 * K_);
    if (settle)
        ck_settle(k, fs, &gate_dc);
    else
        ck_build(k, fs);
}

/* ================= CH-2: Q16/Q15, the output amplifier =================
 * Q16 (JFET) and Q15 (PNP) in one feedback loop, treated as an op amp: gain
 * 1 + R93 100k / (R94 8.2k + C53 10u), C50 150p across R93, R87 2.2k load. It sits at +16.27 V
 * between 0 V and the +30 V rail, so it clips 16 V down and 13.5 V up. */
enum { OA_S = 1, OA_M, OA_A, OA_O, OA_NODES = OA_O };

static void build_ch2_out(Circuit* k, double fs)
{
    ck_begin(k, OA_NODES);
    ck_src(k, OA_S);
    ck_r(k, OA_O, OA_M, 100 * K_);
    ck_c(k, OA_O, OA_M, 150 * P_);
    ck_c(k, OA_M, OA_A, 10 * U_);
    ck_r(k, OA_A, 0, 8.2 * K_);
    ck_r(k, OA_O, 0, 2.2 * K_);
    ck_opamp(k, OA_S, OA_M, OA_O);
    k->op_lo = -16.0;
    k->op_hi = 13.5;
    ck_build(k, fs);
}

/* ================= Reverb driver and tank coils =================
 * IC1a (NJM4558 on +30 V, biased at 15 V): (+) from the send, R60 47k || C34 47p feedback,
 * (-) through C32 10u to R64 680 and, through R44 680, the current-sense resistor R37 0.5 at
 * the coil's cold end. Q8/Q7 push-pull (unity gain here) through R46/R45 10 ohm and C19 47u
 * to the tank. The tank is taken as an Accutronics 8AB2D1A (the listed replacement): input
 * coil 10 ohm at 1 kHz with 0.8 ohm DC, so 1.59 mH. */
#define TANK_IN_R 0.8
#define TANK_IN_L (sqrt(10.0 * 10.0 - 0.8 * 0.8) / (2.0 * 3.14159265358979 * 1000.0))
#define TANK_OUT_R 200.0
#define TANK_OUT_L (sqrt(2575.0 * 2575.0 - 200.0 * 200.0) / (2.0 * 3.14159265358979 * 1000.0))
#define SENSE_R 0.5

enum { RD_S = 1, RD_M, RD_A, RD_O, RD_E, RD_T, RD_TI, RD_TL, RD_SN, RD_NODES = RD_SN };

static void build_reverb_drive(Circuit* k, double fs)
{
    ck_begin(k, RD_NODES);
    ck_src(k, RD_S);
    ck_r(k, RD_O, RD_M, 47 * K_);
    ck_c(k, RD_O, RD_M, 47 * P_);
    ck_c(k, RD_M, RD_A, 10 * U_);
    ck_r(k, RD_A, 0, 680.0);
    ck_r(k, RD_A, RD_SN, 680.0);
    ck_opamp(k, RD_S, RD_M, RD_O);
    ck_buffer(k, RD_O, RD_E, 1.0);
    ck_r(k, RD_E, RD_T, 10.0);
    ck_c(k, RD_T, RD_TI, 47 * U_);
    ck_r(k, RD_TI, RD_TL, TANK_IN_R);
    ck_l(k, RD_TL, RD_SN, TANK_IN_L);
    ck_r(k, RD_SN, 0, SENSE_R);
    ck_build(k, fs);
}

/* ================= Reverb recovery =================
 * The tank's output coil (200 ohm DC, 2575 ohm at 1 kHz, so 0.41 H) into C35 1u and R65 100k,
 * IC1b gain 1 + R68 56k / (R72 1k + C40 10u), C39 47p. */
enum { RR_S = 1, RR_L, RR_TO, RR_P, RR_M, RR_A, RR_O, RR_NODES = RR_O };

static void build_reverb_return(Circuit* k, double fs)
{
    ck_begin(k, RR_NODES);
    ck_src(k, RR_S);
    ck_r(k, RR_S, RR_L, TANK_OUT_R);
    ck_l(k, RR_L, RR_TO, TANK_OUT_L);
    ck_c(k, RR_TO, RR_P, 1 * U_);
    ck_r(k, RR_P, 0, 100 * K_);
    ck_r(k, RR_O, RR_M, 56 * K_);
    ck_c(k, RR_O, RR_M, 47 * P_);
    ck_r(k, RR_M, RR_A, 1 * K_);
    ck_c(k, RR_A, 0, 10 * U_);
    ck_opamp(k, RR_P, RR_M, RR_O);
    ck_build(k, fs);
}

/* ================= Chorus input: C59, filter and IC2b pre-emphasis =================
 * C59 0.047u, R114 220k to the bias, R115 5.6k / C61 2.2n, R120 56k / C62 220p into IC2b (+).
 * Feedback R125 56k || C64 100p; (-) to the bias through R121 27k and R123 4.7k + C65 0.01u.
 * IC2 runs on +15 V, biased at 7.5 V, so it swings 6.5 V each way. */
enum { CP_S = 1, CP_A, CP_B, CP_P, CP_M, CP_C, CP_O, CP_NODES = CP_O };

static void build_chorus_pre(Circuit* k, double fs)
{
    ck_begin(k, CP_NODES);
    ck_src(k, CP_S);
    ck_c(k, CP_S, CP_A, 0.047 * U_);
    ck_r(k, CP_A, 0, 220 * K_);
    ck_r(k, CP_A, CP_B, 5.6 * K_);
    ck_c(k, CP_B, 0, 2.2 * N_);
    ck_r(k, CP_B, CP_P, 56 * K_);
    ck_c(k, CP_P, 0, 220 * P_);
    ck_r(k, CP_O, CP_M, 56 * K_);
    ck_c(k, CP_O, CP_M, 100 * P_);
    ck_r(k, CP_M, 0, 27 * K_);
    ck_r(k, CP_M, CP_C, 4.7 * K_);
    ck_c(k, CP_C, 0, 0.01 * U_);
    ck_opamp(k, CP_P, CP_M, CP_O);
    k->op_lo = -6.5;
    k->op_hi = 6.5;
    ck_build(k, fs);
}

/* ================= After the BBD: filter, Q23 and IC2a de-emphasis =================
 * C68 0.047u, R130 220k; R137 8.2k / C75 3.3n; R136 8.2k, then C69 8.2n to Q23's emitter and
 * R135 8.2k / C70 470p to its base; R131 10k; C60 0.047u, R116 220k into IC2a, whose feedback
 * R111 47k || (R113 8.2k + C58 4.7n) over R112 22k is the de-emphasis. */
enum { CO_S = 1, CO_A, CO_B, CO_C, CO_D, CO_E, CO_P, CO_M, CO_G, CO_O, CO_NODES = CO_O };

static void build_chorus_post(Circuit* k, double fs)
{
    ck_begin(k, CO_NODES);
    ck_src(k, CO_S);
    ck_c(k, CO_S, CO_A, 0.047 * U_);
    ck_r(k, CO_A, 0, 220 * K_);
    ck_r(k, CO_A, CO_B, 8.2 * K_);
    ck_c(k, CO_B, 0, 3.3 * N_);
    ck_r(k, CO_B, CO_C, 8.2 * K_);
    ck_c(k, CO_C, CO_E, 8.2 * N_);
    ck_r(k, CO_C, CO_D, 8.2 * K_);
    ck_c(k, CO_D, 0, 470 * P_);
    ck_buffer(k, CO_D, CO_E, 1.0);
    ck_r(k, CO_E, 0, 10 * K_);
    ck_c(k, CO_E, CO_P, 0.047 * U_);
    ck_r(k, CO_P, 0, 220 * K_);
    ck_r(k, CO_O, CO_M, 47 * K_);
    ck_r(k, CO_O, CO_G, 8.2 * K_);
    ck_c(k, CO_G, CO_M, 4.7 * N_);
    ck_r(k, CO_M, 0, 22 * K_);
    ck_opamp(k, CO_P, CO_M, CO_O);
    k->op_lo = -6.5;
    k->op_hi = 6.5;
    ck_build(k, fs);
}

/* ================= The mixer between the channels and the two power amps =================
 * Left (CH-1's power amp, node 7): CH-1 through C2 1u and R11 47k, CH-2's dry signal through
 * C30 1u and R58 47k, and the reverb through R61 100k. Right (CH-2's power amp): CH-1 through
 * R38 47k (from C2, board terminals 5 and 9), the reverb through R59 100k, and through C81
 * 0.082u either CH-2's dry signal (C42 1u, R67 1M, R70 47k, switch Q10) or the chorus/vibrato
 * (C48 0.047u, R66 47k, switch Q9). Reverb: C38 0.082u, R69 1k, REV VR6 10kB. Each side
 * then: C9 (C26) 0.082u and R21 (R49) 1k to the follower Q5 (Q6), biased by 1M from a
 * 220k / 220k divider. */
enum {
    MX_E1 = 1, MX_P, MX_RV, MX_WT, MX_N5, MX_A, MX_Y, MX_Z, MX_X, MX_W, MX_R1, MX_T6, MX_W6,
    MX_SR, MX_A9, MX_GL, MX_A26, MX_GR, MX_NODES = MX_GR
};

typedef struct {
    double reverb;   /* VR6 rotation */
    double s_dry;    /* Q10 conduction */
    double s_wet;    /* Q9 conduction */
} MixSet;

static void build_mixer(Circuit* k, const MixSet* s, double fs)
{
    ck_begin(k, MX_NODES);
    ck_src(k, MX_E1);
    ck_src(k, MX_P);
    ck_src(k, MX_RV);
    ck_src(k, MX_WT);
    ck_c(k, MX_E1, MX_N5, 1 * U_);            /* CH-1 C2 */
    ck_r(k, MX_N5, MX_A, 47 * K_);            /* CH-1 R11 */
    ck_r(k, MX_N5, MX_SR, 47 * K_);           /* R38 */
    ck_c(k, MX_P, MX_Y, 1 * U_);              /* C30 */
    ck_r(k, MX_Y, MX_A, 47 * K_);             /* R58 */
    ck_c(k, MX_P, MX_Z, 1 * U_);              /* C42 */
    ck_r(k, MX_Z, 0, 1 * M_);                 /* R67 */
    ck_r(k, MX_Z, MX_W, sw_r(47 * K_, s->s_dry)); /* R70, Q10 */
    ck_c(k, MX_WT, MX_X, 0.047 * U_);         /* C48 */
    ck_r(k, MX_X, MX_W, sw_r(47 * K_, s->s_wet)); /* R66, Q9 */
    ck_r(k, MX_W, 0, 1 * M_);                 /* R56 */
    ck_c(k, MX_W, MX_SR, 0.082 * U_);         /* C81 */
    ck_c(k, MX_RV, MX_R1, 0.082 * U_);        /* C38 */
    ck_r(k, MX_R1, MX_T6, 1 * K_);            /* R69 */
    ck_r(k, MX_T6, MX_W6, pot(10 * K_ * (1.0 - s->reverb)));
    ck_r(k, MX_W6, 0, pot(10 * K_ * s->reverb));
    ck_r(k, MX_W6, MX_A, 100 * K_);           /* R61 */
    ck_r(k, MX_W6, MX_SR, 100 * K_);          /* R59 */
    ck_c(k, MX_A, MX_A9, 0.082 * U_);         /* CH-1 C9 */
    ck_r(k, MX_A9, MX_GL, 1 * K_);            /* CH-1 R21 */
    ck_r(k, MX_GL, 0, 1 * M_ + 110 * K_);     /* R24 and the R19/R20 divider */
    ck_c(k, MX_SR, MX_A26, 0.082 * U_);       /* C26 */
    ck_r(k, MX_A26, MX_GR, 1 * K_);           /* R49 */
    ck_r(k, MX_GR, 0, 1 * M_ + 110 * K_);     /* R47 and R42/R43 */
    ck_build(k, fs);
}

/* ================= MAIN IN and the power amp (each side) =================
 * The follower (CH-1 Q5, CH-2 Q6, unity gain here) through C12 (C25) 1u and R22 (R53) 100k,
 * then the power amp's input C15 (C44) 0.082u, R27 (R82) 100k, R28 (R83) 1k, C17 (C45) 150p.
 * The power amp is a discrete class AB stage with gain 1 + R35 27k / (R32 1k + C16 10u)
 * (CH-2: R95, R89, C49), treated as an op amp that clips at the rated output, 22 V rms into
 * 8 ohm (31.1 V peak). */
#define POWER_CLIP 31.1

enum { PA_S = 1, PA_MI, PA_A, PA_B, PA_M, PA_F, PA_O, PA_NODES = PA_O };

static void build_power(Circuit* k, double fs)
{
    ck_begin(k, PA_NODES);
    ck_src(k, PA_S);
    ck_c(k, PA_S, PA_MI, 1 * U_);
    ck_r(k, PA_MI, 0, 100 * K_);
    ck_c(k, PA_MI, PA_A, 0.082 * U_);
    ck_r(k, PA_A, 0, 100 * K_);
    ck_r(k, PA_A, PA_B, 1 * K_);
    ck_c(k, PA_B, 0, 150 * P_);
    ck_r(k, PA_O, PA_M, 27 * K_);
    ck_c(k, PA_M, PA_F, 10 * U_);
    ck_r(k, PA_F, 0, 1 * K_);
    ck_opamp(k, PA_B, PA_M, PA_O);
    k->op_lo = -POWER_CLIP;
    k->op_hi = POWER_CLIP;
    ck_build(k, fs);
}

#endif

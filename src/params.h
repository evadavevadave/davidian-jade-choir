/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* Control metadata for the hosts that don't read bundle/jade_choir.ttl: the VST3 and Audio Unit
 * builds. Values are the same plain values as the LV2 ports, so a setting means the same thing
 * in every format. test/params_check.c checks this table against the TTL and the panel. */
#ifndef JD_PARAMS_H
#define JD_PARAMS_H

#include <math.h>
#include <stdio.h>

#include "ports.h"

typedef struct {
    int port;          /* also the parameter ID in VST3 and the Audio Unit */
    const char* name;
    const char* short_name;
    float min, max, def;
    int log;           /* logarithmic */
    int steps;         /* 0: continuous; otherwise a switch with steps + 1 positions */
    const char* unit;  /* "Hz", "V", "dB", or "" for 0-1 knobs shown as 0-10 like the panel */
    const char* opts[3];
    int output;        /* read only: the LFO lamp */
} JdParam;

static const JdParam JD_PARAMS[] = {
    { PORT_INPUT_JACK, "Input Jack", "Input", 0, 1, 0, 0, 1, "", { "High", "Low", 0 }, 0 },
    { PORT_VOLUME, "Volume", "Volume", 0, 1, 0.4f, 0, 0, "", { 0, 0, 0 }, 0 },
    { PORT_TREBLE, "Treble", "Treble", 0, 1, 0.5f, 0, 0, "", { 0, 0, 0 }, 0 },
    { PORT_MIDDLE, "Middle", "Middle", 0, 1, 0.5f, 0, 0, "", { 0, 0, 0 }, 0 },
    { PORT_BASS, "Bass", "Bass", 0, 1, 0.5f, 0, 0, "", { 0, 0, 0 }, 0 },
    { PORT_HI_TREBLE, "Hi-Treble", "Hi-Treb", 0, 1, 0, 0, 0, "", { 0, 0, 0 }, 0 },
    { PORT_DISTORTION, "Distortion", "Dist", 0, 1, 0, 0, 0, "", { 0, 0, 0 }, 0 },
    { PORT_REVERB, "Reverb", "Reverb", 0, 1, 0.3f, 0, 0, "", { 0, 0, 0 }, 0 },
    { PORT_SPEED, "Vibrato Speed", "Speed", 0, 1, 0.5f, 0, 0, "", { 0, 0, 0 }, 0 },
    { PORT_DEPTH, "Vibrato Depth", "Depth", 0, 1, 0.5f, 0, 0, "", { 0, 0, 0 }, 0 },
    { PORT_MODE, "Chorus Mode", "Mode", 0, 2, 2, 0, 2, "", { "Vibrato", "Off", "Chorus" }, 0 },
    { PORT_FS_DIST, "Distortion Footswitch", "FS Dist", 0, 1, 1, 0, 1, "", { "Off", "On", 0 }, 0 },
    { PORT_FS_REVERB, "Reverb Footswitch", "FS Rev", 0, 1, 1, 0, 1, "", { "Off", "On", 0 }, 0 },
    { PORT_FS_CHORUS, "Chorus Footswitch", "FS Chor", 0, 1, 1, 0, 1, "", { "Off", "On", 0 }, 0 },
    { PORT_LINE_OUT, "Line Out", "Line Out", 0, 1, 0, 0, 1, "", { "Stereo", "Mixed", 0 }, 0 },
    { PORT_VOLTS, "Volts at 0 dBFS", "Volts", 0.1f, 10.0f, 1.0f, 1, 0, "V", { 0, 0, 0 }, 0 },
    { PORT_OUTPUT, "Output", "Output", -24.0f, 12.0f, 0.0f, 0, 0, "dB", { 0, 0, 0 }, 0 },
    { PORT_ENABLED, "Enabled", "Enabled", 0, 1, 1, 0, 1, "", { "Off", "On", 0 }, 0 },
    { PORT_LFO, "LFO", "LFO", 0, 1, 0, 0, 0, "", { 0, 0, 0 }, 1 },
};
#define JD_NUM_PARAMS ((int)(sizeof JD_PARAMS / sizeof JD_PARAMS[0]))

/* The number of saved values: every input control, in port order from JD_FIRST_CONTROL. */
#define JD_NUM_INPUTS (PORT_ENABLED - JD_FIRST_CONTROL + 1)

static const JdParam* jd_param(int port)
{
    int i;
    for (i = 0; i < JD_NUM_PARAMS; ++i)
        if (JD_PARAMS[i].port == port)
            return &JD_PARAMS[i];
    return NULL;
}

static float jd_clamp(const JdParam* p, float v)
{
    if (!(v >= p->min)) /* also catches NaN */
        v = p->min;
    if (v > p->max)
        v = p->max;
    return p->steps ? floorf(v + 0.5f) : v;
}

/* 0-1 host value to plain value. Switches split 0-1 into equal parts, as the VST3 SDK does. */
static float jd_from_norm(const JdParam* p, double n)
{
    n = n < 0.0 ? 0.0 : (n > 1.0 ? 1.0 : n);
    if (p->steps) {
        int i = (int)(n * (p->steps + 1));
        return p->min + (float)(i > p->steps ? p->steps : i);
    }
    if (p->log)
        return (float)(p->min * pow((double)p->max / p->min, n));
    return (float)(p->min + n * (p->max - p->min));
}

static double jd_to_norm(const JdParam* p, float v)
{
    v = jd_clamp(p, v);
    if (p->steps)
        return (double)(v - p->min) / p->steps;
    if (p->log)
        return log((double)v / p->min) / log((double)p->max / p->min);
    return (double)(v - p->min) / (p->max - p->min);
}

/* Text for a value, matching the panel's display. */
static void jd_format(const JdParam* p, float v, char* buf, size_t n)
{
    v = jd_clamp(p, v);
    if (p->steps)
        snprintf(buf, n, "%s", p->opts[(int)(v - p->min)]);
    else if (p->unit[0] == 'd')
        snprintf(buf, n, "%+.1f", v);
    else if (p->unit[0])
        snprintf(buf, n, "%.2f", v);
    else if (p->port == PORT_DISTORTION && v < 0.005f)
        snprintf(buf, n, "Off");
    else if (p->output)
        snprintf(buf, n, "%.0f%%", 100.0f * v);
    else
        snprintf(buf, n, "%.1f", 10.0f * v);
}

/* The reverse of jd_format; returns 0 if the text means nothing. */
static int jd_parse(const JdParam* p, const char* s, float* out)
{
    double d;
    int i;
    for (i = 0; p->steps && i <= p->steps; ++i) {
        const char *a = p->opts[i], *b = s;
        while (*a && *b && (*a | 32) == (*b | 32))
            ++a, ++b;
        if (!*a && !*b) {
            *out = p->min + (float)i;
            return 1;
        }
    }
    if (p->port == PORT_DISTORTION && (s[0] | 32) == 'o' && (s[1] | 32) == 'f') {
        *out = 0.0f;
        return 1;
    }
    if (sscanf(s, "%lf", &d) != 1)
        return 0;
    if (!p->unit[0] && !p->steps)
        d /= 10.0; /* 0-10 scale */
    *out = jd_clamp(p, (float)d);
    return 1;
}

#endif

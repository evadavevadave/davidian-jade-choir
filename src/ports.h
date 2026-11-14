/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* Port indices, shared by the plugin and its GUI. They must match bundle/jade_choir.ttl. */
#ifndef PORTS_H
#define PORTS_H

#define JD_URI    "urn:davidian:jade-choir"
#define JD_UI_URI "urn:davidian:jade-choir#ui"

enum {
    PORT_IN, PORT_OUT_L, PORT_OUT_R,
    PORT_INPUT_JACK, PORT_VOLUME, PORT_TREBLE, PORT_MIDDLE, PORT_BASS, PORT_HI_TREBLE,
    PORT_DISTORTION, PORT_REVERB, PORT_SPEED, PORT_DEPTH, PORT_MODE,
    PORT_FS_DIST, PORT_FS_REVERB, PORT_FS_CHORUS, PORT_LINE_OUT,
    PORT_VOLTS, PORT_OUTPUT, PORT_ENABLED,
    PORT_LFO, /* output: the chorus/vibrato LFO, 0-1, for the GUI's lamp */
    PORT_COUNT
};

/* The first control port; every port from here to PORT_ENABLED is a saved input control. */
#define JD_FIRST_CONTROL PORT_INPUT_JACK

/* PORT_MODE positions, in the order of the panel switch. */
enum { MODE_VIB, MODE_OFF, MODE_CHORUS };

#endif

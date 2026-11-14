/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* The Jade Choir engine as a small C API, shared by the LV2, VST3 and Audio Unit builds.
 * It follows the LV2 model: every port in ports.h is a float pointer the wrapper connects,
 * control ports read one value per block, and the opto lamp outputs are written per block. */
#ifndef JD_ENGINE_H
#define JD_ENGINE_H

#include <stdint.h>

#include "ports.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct JdEngine JdEngine;

JdEngine* jd_engine_new(double sample_rate); /* allocates; not real-time safe */
void jd_engine_connect(JdEngine* e, uint32_t port, void* data);
void jd_engine_activate(JdEngine* e); /* resets the circuit; connect the controls first */
void jd_engine_run(JdEngine* e, uint32_t frames); /* in place is fine: each sample is read before it is written */
void jd_engine_free(JdEngine* e);

#ifdef __cplusplus
}
#endif

#endif

/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 Davidian. See LICENSE and NOTICE.md. */
/* Audio Unit (AUv2 effect) build of the Jade Choir for macOS: engine.c behind Apple's
 * AudioUnitSDK (Apache 2.0 licence; see NOTICE.md), with parameters from params.h and a Cocoa
 * view from au_view.m. */
#include <AudioUnitSDK/AUEffectBase.h>
#include <AudioToolbox/AudioUnitProperties.h>

#include <cstring>
#include <vector>

extern "C" {
#include "../engine.h"
}
#include "../params.h"

/* Both set by CMakeLists.txt, which also puts them in Info.plist and au_view.m. */
#ifndef JD_AU_BUNDLE_ID
#    error "JD_AU_BUNDLE_ID must name the component's bundle identifier"
#endif
#ifndef JD_AU_VIEW_FACTORY
#    error "JD_AU_VIEW_FACTORY must name the Cocoa view factory class"
#endif
#define JD_STR2(x) #x
#define JD_STR(x) JD_STR2(x)

class DavidianJadeChoirAU : public ausdk::AUEffectBase {
public:
    explicit DavidianJadeChoirAU(AudioComponentInstance ci) : AUEffectBase(ci, false)
    {
        /* Parameter IDs are the LV2 port numbers. The host's bypass replaces "Enabled". */
        for (int i = 0; i < JD_NUM_PARAMS; ++i) {
            values[JD_PARAMS[i].port] = JD_PARAMS[i].def;
            if (JD_PARAMS[i].port != PORT_ENABLED)
                Globals()->SetParameter((AudioUnitParameterID)JD_PARAMS[i].port, JD_PARAMS[i].def);
        }
    }

    ~DavidianJadeChoirAU() override { jd_engine_free(engine); }

    OSStatus Initialize() override
    {
        const OSStatus r = AUEffectBase::Initialize();
        if (r != noErr)
            return r;
        jd_engine_free(engine);
        engine = jd_engine_new(GetSampleRate());
        if (!engine)
            return kAudioUnitErr_FailedInitialization;
        for (int port = JD_FIRST_CONTROL; port < PORT_COUNT; ++port)
            jd_engine_connect(engine, (uint32_t)port, &values[port]);
        mono.assign(GetMaxFramesPerSlice(), 0.0f);
        pull();
        jd_engine_activate(engine);
        return noErr;
    }

    void Cleanup() override
    {
        jd_engine_free(engine);
        engine = nullptr;
        AUEffectBase::Cleanup();
    }

    OSStatus Reset(AudioUnitScope scope, AudioUnitElement element) override
    {
        if (engine) {
            pull();
            jd_engine_activate(engine);
        }
        return AUEffectBase::Reset(scope, element);
    }

    /* Stereo out (normal and effect line outs), from mono in, or stereo in averaged into the
     * amplifier's single input. */
    UInt32 SupportedNumChannels(const AUChannelInfo** out) override
    {
        static const AUChannelInfo configs[] = { { 2, 2 }, { 1, 2 } };
        if (out)
            *out = configs;
        return 2;
    }

    bool SupportsTail() override { return true; }
    Float64 GetTailTime() override { return 3.0; } /* the spring reverb's decay */

    OSStatus GetParameterInfo(AudioUnitScope scope, AudioUnitParameterID id, AudioUnitParameterInfo& info) override
    {
        const JdParam* p = jd_param((int)id);
        if (scope != kAudioUnitScope_Global)
            return kAudioUnitErr_InvalidScope;
        if (!p || p->port == PORT_ENABLED)
            return kAudioUnitErr_InvalidParameter;
        std::memset(&info, 0, sizeof info);
        FillInParameterName(info, CFStringCreateWithCString(nullptr, p->name, kCFStringEncodingUTF8), true);
        info.flags |= kAudioUnitParameterFlag_IsReadable;
        if (p->output)
            info.flags |= kAudioUnitParameterFlag_MeterReadOnly;
        else
            info.flags |= kAudioUnitParameterFlag_IsWritable;
        if (p->log)
            info.flags |= kAudioUnitParameterFlag_DisplayLogarithmic;
        if (p->steps) {
            info.unit = kAudioUnitParameterUnit_Indexed;
        } else if (!std::strcmp(p->unit, "Hz")) {
            info.unit = kAudioUnitParameterUnit_Hertz;
        } else if (!std::strcmp(p->unit, "dB")) {
            info.unit = kAudioUnitParameterUnit_Decibels;
        } else if (!std::strcmp(p->unit, "V")) {
            info.unit = kAudioUnitParameterUnit_CustomUnit;
            info.unitName = CFSTR("V");
        } else {
            info.unit = kAudioUnitParameterUnit_Generic;
        }
        info.minValue = p->min;
        info.maxValue = p->max;
        info.defaultValue = p->def;
        return noErr;
    }

    OSStatus GetParameterValueStrings(AudioUnitScope scope, AudioUnitParameterID id, CFArrayRef* out) override
    {
        const JdParam* p = jd_param((int)id);
        CFStringRef names[3];
        if (scope != kAudioUnitScope_Global || !p || !p->steps || p->port == PORT_ENABLED)
            return kAudioUnitErr_InvalidProperty;
        if (!out) /* the SDK asks whether there are any */
            return noErr;
        for (int i = 0; i <= p->steps; ++i)
            names[i] = CFStringCreateWithCString(nullptr, p->opts[i], kCFStringEncodingUTF8);
        *out = CFArrayCreate(nullptr, (const void**)names, p->steps + 1, &kCFTypeArrayCallBacks);
        for (int i = 0; i <= p->steps; ++i)
            CFRelease(names[i]);
        return noErr;
    }

    OSStatus GetPropertyInfo(AudioUnitPropertyID id, AudioUnitScope scope, AudioUnitElement element,
                             UInt32& size, bool& writable) override
    {
        if (scope == kAudioUnitScope_Global && id == kAudioUnitProperty_CocoaUI) {
            size = sizeof(AudioUnitCocoaViewInfo);
            writable = false;
            return noErr;
        }
        return AUEffectBase::GetPropertyInfo(id, scope, element, size, writable);
    }

    OSStatus GetProperty(AudioUnitPropertyID id, AudioUnitScope scope, AudioUnitElement element,
                         void* data) override
    {
        if (scope == kAudioUnitScope_Global && id == kAudioUnitProperty_CocoaUI) {
            CFBundleRef bundle = CFBundleGetBundleWithIdentifier(CFSTR(JD_AU_BUNDLE_ID));
            CFURLRef url = bundle ? CFBundleCopyBundleURL(bundle) : nullptr;
            AudioUnitCocoaViewInfo info;
            if (!url)
                return kAudioUnitErr_InvalidProperty;
            /* The host releases both. */
            info.mCocoaAUViewBundleLocation = url;
            info.mCocoaAUViewClass[0] = CFStringCreateWithCString(nullptr, JD_STR(JD_AU_VIEW_FACTORY),
                                                                  kCFStringEncodingUTF8);
            std::memcpy(data, &info, sizeof info);
            return noErr;
        }
        return AUEffectBase::GetProperty(id, scope, element, data);
    }

    /* The engine fades to the dry signal itself, so the host's bypass goes through it. */
    bool ShouldBypassEffect() override { return false; }

    OSStatus ProcessBufferLists(AudioUnitRenderActionFlags& flags, const AudioBufferList& in,
                                AudioBufferList& out, UInt32 frames) override
    {
        if (!engine || in.mNumberBuffers < 1 || out.mNumberBuffers < 2)
            return kAudioUnitErr_FormatNotSupported;
        pull();
        float* src = static_cast<float*>(in.mBuffers[0].mData);
        if (in.mNumberBuffers > 1) {
            const float* r = static_cast<const float*>(in.mBuffers[1].mData);
            if (mono.size() < frames)
                return kAudioUnitErr_TooManyFramesToProcess;
            for (UInt32 i = 0; i < frames; ++i)
                mono[i] = 0.5f * (src[i] + r[i]);
            src = mono.data();
        }
        jd_engine_connect(engine, PORT_IN, src);
        jd_engine_connect(engine, PORT_OUT_L, out.mBuffers[0].mData);
        jd_engine_connect(engine, PORT_OUT_R, out.mBuffers[1].mData);
        jd_engine_run(engine, frames);
        /* The LFO lamp, for the view. */
        SetParameter(PORT_LFO, values[PORT_LFO]);
        flags &= ~kAudioUnitRenderAction_OutputIsSilence;
        return noErr;
    }

private:
    void pull()
    {
        for (int i = 0; i < JD_NUM_PARAMS; ++i) {
            const JdParam* p = &JD_PARAMS[i];
            if (!p->output && p->port != PORT_ENABLED)
                values[p->port] = GetParameter((AudioUnitParameterID)p->port);
        }
        values[PORT_ENABLED] = IsBypassEffect() ? 0.0f : 1.0f;
    }

    JdEngine* engine = nullptr;
    float values[PORT_COUNT] = {};
    std::vector<float> mono; /* a stereo input, averaged */
};

AUSDK_COMPONENT_ENTRY(ausdk::AUBaseFactory, DavidianJadeChoirAU)

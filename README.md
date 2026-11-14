# Davidian Jade Choir

A circuit-modelled two-channel stereo chorus combo amplifier (LV2, VST3 and Audio Unit), built part by part from the manufacturer's 1983 service notes for the original combo (fifth edition): both JFET preamps with their bright switches and tone stacks, channel 2's switchable FET distortion with diode clipping, the spring reverb with its current-feedback driver, the BBD chorus/vibrato, the JFET switches that route it, and the passive mixing into the two 60 W power amps.

**A close model, not an exact copy.** It simulates the circuit from its component values; it is not sampled or measured from an amplifier. It reproduces the DC voltages printed on the schematic to within 0.03 V and the printed signal levels to within 2 dB, but some parts have no published values, and those are estimates, listed under [Estimates](#estimates). It has not been compared side by side with a real amplifier.

![The GUI](docs/gui.png)

Mono in, stereo out, no latency. The two outputs are the two speakers: left is channel 1's power amp, right is channel 2's. That is how the original makes its spatial chorus: channel 2's dry signal goes to the left speaker and the chorus or vibrato to the right, and they mix in the air. Channel 1 and the reverb go to both. A stereo input is averaged into the single input. Every format is built from the same circuit code, shows the same GUI and sounds identical. GPL-3.0-or-later.

| Format | Windows | macOS | Linux |
|---|---|---|---|
| LV2 | yes | yes | yes |
| VST3 | yes | | |
| Audio Unit (AUv2) | | yes | |

## Sources

- **Service notes** (the original combo's, fifth edition, January 1983): the circuit diagram of both pre-main boards, the parts list, the specifications, and the DC and signal voltages printed on the diagram. This is what the plugin is built from.
- **Owner's manual** (1996): control functions.
- **Reverb tank:** the original Z-3F's listed replacement, the Accutronics 8AB2D1A (short three-spring type 8, 10 ohm input, 2575 ohm output, medium decay), from the tank code system and sellers' data.
- **Datasheets:** 2SK117-GR JFET ranges, NJM4558 output swing, 1S2473 diode as the 1N4148.

A 1984 head schematic (a different, op-amp based, single-channel circuit) was used for the first version of this plugin and is superseded.

## Install

Build it (see [Build](#build)), then copy the plugin folder for your host's format into its plugin folder and restart the host (or rescan plugins):

| Format | Copy | Into |
|---|---|---|
| VST3 (Windows) | `Davidian Jade Choir.vst3` | `C:\Program Files\Common Files\VST3` |
| Audio Unit (macOS) | `Davidian Jade Choir.component` | `~/Library/Audio/Plug-Ins/Components` |
| LV2 (Windows) | `jade_choir.lv2` | `%APPDATA%\LV2` (create it if it doesn't exist) |
| LV2 (macOS) | `jade_choir.lv2` | `~/Library/Audio/Plug-Ins/LV2` |
| LV2 (Linux) | `jade_choir.lv2` | `~/.lv2` |

In the host it is listed as **Davidian Jade Choir** (VST3, LV2) or under **Davidian** as **Jade Choir** (Audio Unit; manufacturer code `Dvdn`, subtype `Jdch`). The LV2 URI is `urn:davidian:jade-choir`.

## Controls

| Panel | Control | Circuit part |
|---|---|---|
| Channel 1 | Bright | C6 330p across the top of VOLUME |
| Channel 1 | Volume, Treble, Middle, Bass | VR1 1MB, VR2 250kB, VR3 10kB, VR4 250kA |
| Channel 2 | Bright | C5 220p across the top of VOLUME |
| Channel 2 | Volume, Treble, Middle, Bass | as channel 1 (the stack's mid capacitor is 0.047u here, 0.056u on channel 1) |
| Channel 2 | Distortion | VR5 10kC in Q3's source. At OFF its switch turns the distortion path off |
| Channel 2 | Reverb | VR6 10kB |
| Chorus - Vibrato | Speed, Depth | VR7 100kA, VR8 100kB (vibrato only) |
| Chorus - Vibrato | Mode: Vib / Off / Chorus | SW2. Chorus has a fixed rate and depth |
| Input | Channel: CH 1 / CH 2 / Both | Which channel's input the plugin's input feeds (Both: as with a Y cable) |
| Input | Jack: High / Low | HIGH through 33k; LOW through 68k with the HIGH jack shunting it (about 10 dB less) |
| Levels | Input V | Volts at the input jack for a 0 dBFS sample (0.1 to 10, default 1.0) |
| Levels | Output | Not on the amp: -24 to +12 dB. At 0 dB, full scale is the power amp's clipping level |
| Footswitches | Chorus-Vib, Reverb, Distortion | The footswitch jacks. On (LED lit) is the same as no footswitch plugged in |
| (host) | Enabled | Plugin bypass with a 20 ms crossfade to the dry input on both sides |

The amber LFO lamp follows the chorus/vibrato LFO. The readout shows the name and value of the last control you touched. Drag a knob up or down, hold Shift to fine-tune, use the mouse wheel, and double-click to reset. Click the upper or lower part of a toggle (or the middle of a three-way) to pick that position.

## How it is modelled

Each stage is a netlist with the schematic's part values (`src/amp.h`), solved by a small circuit simulator (`src/circuit.h`): modified nodal analysis with trapezoidal capacitors and inductors, n-channel JFETs (square law, with the triode region) and antiparallel diodes solved by Newton's method, ideal op amps that clip at their limits, and the supply rails, so the JFET stages run at their DC operating points. Each stage starts at its operating point. The preamps run at twice the host rate below 88.2 kHz.

- **Channel 1:** input jacks, Q1 (JFET), tone stack, VOLUME and BRIGHT, Q3 (JFET) and the Q2 follower.
- **Channel 2:** input jacks, Q1, tone stack, the Q2 follower, then the clean path (R16 100k) and the distortion stage: Q3 with D1/D2 clipping in its feedback loop through C10, its gain set by DIST, switched in by the JFET Q4. Then VOLUME and BRIGHT, Q5 (JFET), and the Q16/Q15 output amplifier.
- **Reverb:** the send from Q5's drain, IC1a and the push-pull pair driving the tank's input coil with current feedback (R37 0.5 ohm sense resistor, R44, R64), the springs, the output coil into IC1b (gain 57), and REVERB.
- **Chorus/vibrato:** Q19, the input filter and IC2b pre-emphasis, the MN3007 BBD (1024 stages, delay = 512 clock periods), the third-order output filter with Q23, and IC2a de-emphasis.
- **Mixing:** the passive network between the boards: channel 1 to both sides (R11, R38 47k), channel 2 dry to the left (C30, R58 47k), the reverb to both (R61, R59 100k), and to the right either channel 2 dry (switch Q10) or the chorus/vibrato (switch Q9). The JFET switches fade over about 50 ms, as their gate networks set.
- **Power amps:** the MAIN IN coupling, the input network, and gain 1 + R35 / (R32 + C16) = 28, clipping at the rated output, 22 V rms into 8 ohm.

### JFET calibration

The schematic prints most JFETs' DC operating points. JFETs of one type vary widely, so each one's pinch-off voltage is worked out from its own printed operating point, with IDSS at the 2SK117-GR grade's 6.5 mA maximum:

| JFET | Printed | Pinch-off |
|---|---|---|
| CH-1 Q1 | +12.6 V drain (on +30 V) | -1.043 V |
| CH-2 Q1 | +12.8 V drain (on +26 V) | -0.544 V |
| CH-2 Q2 | +13.3 V source | -0.511 V |
| CH-2 Q3 | +10.4 V drain | -0.518 V |
| CH-2 Q5 | +5.17 V drain | -0.505 V |

All fall inside the datasheet's -0.2 to -1.5 V. CH-1 Q3 and CH-2 Q19 have no printed voltages and take their board's other value. With these, the stages' AC gains, which were not fitted, land within 1.2 dB of the printed signal levels.

### Checked against the schematic

`test_dsp` drives each stage at the schematic's test conditions (1 kHz; BASS and TREBLE fully clockwise, MIDDLE fully anticlockwise):

| Point | Model | Schematic |
|---|---|---|
| DC: CH-1 Q1, CH-2 Q1, Q2, Q3, Q5 | +12.60, +12.80, +13.27, +10.41, +5.18 V | +12.6, +12.8, +13.3, +10.4, +5.17 V |
| CH-1 Q1 drain, 19 mV HIGH / 59 mV LOW | 166 / 171 mV | 180 mV |
| CH-2 Q1 drain, 19 mV HIGH | 176 mV | 180 mV |
| CH-2 after the tone stack | 24 mV | 24 mV |
| CH-2 Q2 source | 24 mV | 27 mV |
| CH-2 Q5 drain, 20 mV at its gate | 197 mV | 200 mV |
| Reverb send | 98 mV | 100 mV |
| Chorus send (Q19) | 194 mV | 185 mV |
| Q15 output | 2.58 V | 2.1 V |
| IC2b output (into the BBD) | 906 mV | 940 mV |
| IC2a output | 2.2 V | 2.2 V (sets the BBD's gain) |
| Speaker, 800 mV at MAIN IN | 22.3 V | 21.5 V |
| Chorus LFO | 1.2 s, 9 V p-p | 1.2 s, 9 V p-p |
| Vibrato SPEED | 500 to 120 ms | 500 to 120 ms |
| BBD delay (chorus) | 1.28 to 6.14 ms | clock 2.5 to 12 us |

The linear stages (the output amplifier, reverb driver and recovery, chorus filters, mixer and power amps) also match an independent AC analysis of the same netlists to within 0.0001 dB, 40 Hz to 6 kHz.

Two levels are off by more than 1 dB: CH-1 Q1 comes out 0.7 dB low (with CH-1's drawn +30 V rail its printed DC point and printed gain can't both be met inside the GR grade; on CH-2's +26 V they are), and Q15's output is 1.8 dB high (the feedback values as drawn give 13.2x where the printed levels imply 10.5x).

### Estimates

These have no values on the schematic and no published data was found, so they are estimates:

- **The springs** (`src/fx.h`). The coils are modelled from the 8AB2D1A's figures, but what happens between them is a generic three-spring model: transit times 29.5, 33.7 and 37.1 ms, a 2.4 s decay (middle of the medium range), dispersion from stretched all-pass filters, a 4.5 kHz loop low-pass, and a level set so that full REVERB is about as loud as the dry signal. This is the largest departure from the hardware.
- **The LFO between its printed ends.** The schematic prints the chorus waveform and the vibrato waveform at both SPEED ends; in between, the rate is taken as geometric in rotation and the vibrato triangle grows linearly into its clipping. The LFO circuit itself (IC5) is not simulated: read from the scan, it gives 1.5 s for chorus where the printed waveform says 1.2 s, so the printed figures are used.
- **BBD clock law:** the clock period is taken as linear in the LFO voltage (the schematic gives its range, not its shape). Vibrato's larger swing extends it, held between 1.5 and 16 us.
- **JFET switches** (2SK246): pinch-off -1.5 V and 100 ohm on-resistance. They only set the exact timing of the 50 ms fades.
- **Pot tapers:** A 15% at half rotation, C 85%.
- **Datasheet typicals:** NJM4558 swing 1 V from each rail; diodes as the 1N4148.

### Simplified or left out

- The followers (CH-1 Q2, Q5; CH-2 Q6; Q23; the reverb driver's push-pull pair) are unity-gain buffers.
- The power amps are their gain and their clipping level; their crossover and output stage are not modelled.
- **Q25/Q26**, which sense the signal and feed IC5's reference, are not modelled; the LFO always runs.
- The speakers and cabinet are not modelled: the outputs are the voltages at the speaker terminals.
- The BBD is a clean variable delay; at these clock rates its images are far above the output filter.

## Build

The LV2 build is plain C11 (the macOS window layer is compiled as Objective-C) and needs no LV2 SDK or other libraries. The VST3 and the Audio Unit need their SDKs, which CMake downloads while configuring: the Steinberg VST3 SDK 3.8.1 (MIT) and Apple's AudioUnitSDK 1.2.0 (Apache 2.0).

| Source | What it is |
|---|---|
| `src/circuit.h` | The circuit simulator |
| `src/amp.h` | The amplifier's stages as netlists, with the schematic's part values and the JFET calibration |
| `src/fx.h` | LFO, BBD delay, JFET switch timing, springs, oversampling filters |
| `src/engine.c`, `src/engine.h` | Routing, controls and the per-sample loop, behind a small C API that every format uses |
| `src/params.h`, `src/ports.h` | Parameter names, ranges and text for VST3 and Audio Units, and the LV2 port numbers they match |
| `src/plugin.c`, `src/ui/ui_lv2.c` | LV2 plugin and GUI |
| `src/vst3/` | VST3 processor, controller and editor, and its CMake build |
| `src/au/` | Audio Unit, its Cocoa view and its Info.plist |
| `src/ui/panel.h`, `src/ui/view.c` | The panel and the window layer under it (Win32, Cocoa, X11), shared by every GUI |
| `src/ui/canvas.h`, `src/ui/font.h`, `src/ui/style.h` | Davidian GUI house style: software renderer, stroke font and palette |
| `src/lv2_abi.h` | The LV2 C interface, so no SDK is needed |
| `bundle/` | LV2 metadata |
| `licenses/` | The VST3 SDK and AudioUnitSDK licences, shipped inside the bundles that use them |

### Windows (Visual Studio)

Needs Visual Studio 2017 or later with the C++ tools. `build.bat` builds the LV2 bundle, `build\jade_choir.lv2`, and the Windows-only LV2 checks, without CMake.

```bash
build.bat
```

### Every platform (CMake)

Needs CMake 3.25 or later and a C/C++ compiler. On macOS, the Xcode command-line tools; add `-DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"` for a universal build. On Linux, gcc or clang plus the X11 headers (`libx11-dev` on Debian/Ubuntu) for the GUI; without them CMake builds the plugin alone.

```bash
cmake -S . -B build-cmake -DCMAKE_BUILD_TYPE=Release
```

```bash
cmake --build build-cmake --config Release
```

The VST3 lands in `build-cmake\VST3\Release` (Windows) and the Audio Unit in `build-cmake/AU` (macOS, signed ad hoc). They are on by default on Windows and macOS respectively; turn them off with `-DJADE_CHOIR_VST3=OFF` or `-DJADE_CHOIR_AU=OFF`.

## Tests

Windows, with `build.bat`:

```bash
build-test\test_dsp.exe build\jade_choir.lv2
```

```bash
build-test\lilv_check.exe build
```

```bash
build-test\ui_check.exe build\jade_choir.lv2
```

Every platform, with CMake:

```bash
ctest --test-dir build-cmake -C Release
```

- `test_dsp` checks the linear stages against AC analysis, every stage against the schematic's printed DC and signal voltages, the LFO and BBD, and the built LV2 library (routing of both channels, chorus, distortion, reverb tail, a silent start, bypass, every sample rate, CPU time).
- `ui_render` draws the panel to a BMP file without opening a window: `build-test\ui_render.exe out.bmp`.
- `ui_check` (Windows) hosts the built GUI and drives it with mouse messages.
- `lilv_check` (Windows) validates the bundle with the lilv LV2 parser (from an Ardour 9 install).
- `params_check` checks the VST3 and Audio Unit parameters against the LV2 ports and the panel.
- `vst3_check` (Windows) hosts the VST3, drives its editor and compares its sound with the LV2 build. The VST3 build also runs Steinberg's validator.
- `au_check` (macOS) loads the Audio Unit, compares its sound with the LV2 build and opens its view.

CPU: about 18% of one core at 48 kHz with the defaults, about 27% with both channels, distortion, vibrato and reverb at full (measured on the development machine). Every JFET is simulated, which costs; a channel with nothing at its input is held at rest instead of simulated.

## Use of AI

Made with generative AI (Claude Code), directed and reviewed by Davidian. See [AI_USAGE.md](AI_USAGE.md) for what the author and the AI each did, and what has and hasn't been checked.

## Licence

GPL-3.0-or-later. See [LICENSE](LICENSE), [NOTICE.md](NOTICE.md) and [AI_USAGE.md](AI_USAGE.md). `src/lv2_abi.h` is under the ISC licence, which is GPL-compatible, because it reproduces the LV2 interface declarations. The VST3 plugin includes code from the Steinberg VST3 SDK (MIT) and the Audio Unit from Apple's AudioUnitSDK (Apache 2.0), both GPL-3.0-compatible. The build copies `LICENSE`, `NOTICE.md` and `AI_USAGE.md` into every bundle. VST is a registered trademark of Steinberg Media Technologies GmbH.

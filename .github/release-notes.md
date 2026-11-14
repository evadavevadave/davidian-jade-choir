Ready-made **Davidian Jade Choir** plugins for Windows, macOS and Linux, built and tested from the same source by GitHub Actions. This is the first release: a circuit model of a classic two-channel stereo chorus combo amplifier, from its 1983 service notes.

## Downloads

| File | For | Contains |
|---|---|---|
| `...-windows-x64.zip` | 64-bit Windows 10 and 11 | VST3 and LV2 |
| `...-macos-universal.zip` | macOS 10.13 and later, Apple silicon and Intel | Audio Unit and LV2 |
| `...-linux-x64.zip` | 64-bit Linux on x86 (glibc 2.29 or later, such as Ubuntu 20.04 or Debian 11) | LV2 |
| `...-linux-arm64.zip` | 64-bit Linux on ARM, such as a Raspberry Pi 4 or 5 | LV2 |

## Install

Unzip, close your host, and copy the format it uses:

- **VST3 (Windows):** copy the `Davidian Jade Choir.vst3` folder into `C:\Program Files\Common Files\VST3`.
- **Audio Unit (macOS):** copy `Davidian Jade Choir.component` into `~/Library/Audio/Plug-Ins/Components`. It isn't notarised, so clear the download quarantine once in Terminal: `xattr -dr com.apple.quarantine ~/Library/Audio/Plug-Ins/Components/"Davidian Jade Choir.component"`.
- **LV2:** copy the `jade_choir.lv2` folder into `%APPDATA%\LV2` (Windows), `~/Library/Audio/Plug-Ins/LV2` (macOS, then clear the quarantine the same way) or `~/.lv2` (Linux).

Restart the host (or rescan its plugins) and look for **Jade Choir** by **Davidian**.

## What it is

- Mono in, stereo out: the two outputs are the two speakers. Channel 2's dry signal goes to the left and its chorus or vibrato to the right, so through two speakers you get the original's spatial chorus. Channel 1 and the reverb go to both.
- Both channels: Bright, Volume, Treble, Middle and Bass; channel 2 adds Distortion (with its OFF position) and Reverb. Vibrato Speed and Depth, the Vibrato/Off/Chorus switch, the three footswitches, and which channel the input feeds (or both).
- The power amps are modelled up to their clipping; the speakers and cabinet are not.

## Testing status

- Every build passes the DSP tests (each stage against an AC analysis of its netlist, and against the signal voltages printed on the schematic) and the GUI render test.
- **VST3:** passes Steinberg's validator. A test host drives its editor with simulated mouse input, and its output matches the LV2 build sample for sample.
- **Audio Unit:** checked by Apple's `auval` in strict mode and by a test host that compares it with the LV2 build.
- The Linux bundles are also checked with the lilv LV2 parser.
- Not yet tried in a DAW by a person.

## Licence and AI usage

- **Licence:** GPL-3.0-or-later. The licence, `NOTICE.md` and `AI_USAGE.md` are inside each bundle. The VST3 includes the Steinberg VST3 SDK (MIT) and the Audio Unit includes Apple's AudioUnitSDK (Apache 2.0). Their licences are in the bundles too.
- **AI usage:** this plugin was made with generative AI (Claude Code). See `AI_USAGE.md`.

VST is a registered trademark of Steinberg Media Technologies GmbH.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

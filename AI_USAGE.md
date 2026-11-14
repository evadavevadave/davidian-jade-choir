# AI usage

This plugin was built with generative AI.

- **Tool:** Claude Code, Anthropic's coding assistant, using Anthropic's Claude models (Opus).
- **Human role:** Davidian set the requirements and the design direction, supplied the reference schematics, service notes and manual, pointed to the reverb tank's replacement, and decides what is released.
- **What the AI did:**
  - Read the component values, wiring and printed DC and signal voltages off the service notes' circuit diagram, and the control functions off the owner's manual.
  - Calibrated each JFET to its printed DC operating point, within the datasheet's ranges.
  - Wrote the circuit simulator and the amplifier model, the LV2 plugin and GUI code, and the panel layout. The stroke font, renderer and window code come from the earlier Davidian Bi-Lumen, also AI-written.
  - Wrote the VST3 and Audio Unit wrappers around the same circuit and panel code.
  - Wrote the build files, the LV2 metadata, the tests and the documentation.
  - Looked up datasheets and published information online for values the schematic doesn't give, and chose the estimates listed in the README where none was found.
- **How it was checked:**
  - Automated tests in `test/`: every stage's simulation against an independent AC analysis of the same netlist, the stages against the signal voltages printed on the schematic, the LFO and BBD delay against the schematic's figures, plugin and GUI loading, and the lilv LV2 validator.
  - The VST3 and the Audio Unit are hosted by test programs that compare their output with the LV2 build sample for sample, and drive the VST3 editor with simulated mouse input. The VST3 passes Steinberg's validator. The Audio Unit is checked with Apple's `auval` in CI; it has not yet been built on a Mac.
  - CMake (`ctest`) and `build.bat` build the tests, and the README lists the commands that run them.
  - No listening test in a DAW has been recorded yet.
- **Training data and copyright:** no third-party code was knowingly copied. The only reproduced material is the LV2 interface declarations, credited in [NOTICE.md](NOTICE.md). The VST3 and Audio Unit builds link the Steinberg VST3 SDK and Apple's AudioUnitSDK unmodified, under their own licences (also in NOTICE.md). The GUI artwork and font are generated from code written for this project, not from images.

The AI's output is released under the project licence, GPL-3.0-or-later (see `LICENSE`). Under the GPL, there is no warranty.

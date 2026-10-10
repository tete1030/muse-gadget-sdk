# Waveshare 1.75-B CJK, reading pace and rotation

Source baseline: `812c46f9fe42b69d916613a9dc702cbac5ff7b54`.
The implementation and tests reproduce the complete source patch shipped with
the app-only firmware already installed on the device (patch SHA256
`71ceed63a59332f329758c6b6de1740b859660f01956b972870679a7150ec487`).

The standard non-C 1.75 overlay enables CJK fonts. Silent reading changes from
16 to 6 UTF-8 bytes/s, with the final page held for 6 rather than 2 seconds.
Settings -> Screen offers manual 0/90/180/270-degree clockwise rotation and
opt-in Auto; the default remains manual 0 degrees. Two new NVS settings retain
the manual angle and Auto preference. The existing DMA band buffers perform
software pixel rotation, with inverse calibrated touch coordinates. QMI8658
acceleration must remain plausible and stable for 700 ms before Auto changes
direction. Rotation waits for finger release; sensor failure falls back to the
saved manual direction. Partition layout, pairing protocol and TTS are unchanged.

Validation reported by the original firmware package: ESP-IDF v6.0.1 full and
incremental target builds passed; 297 host tests ran, 296 passed and one skipped
because host PSA Crypto dependencies were missing. These are prior cloud build
results, not a new local target build.

New local validation during publication: the 57 rotation, 16 orientation and
9 caption-wrap tests all passed (82 tests total). Full host discovery ran 273
tests with 13 errors and 4 skips: errors came from missing managed cJSON source
and missing CMake. The fresh checkout has no configured ESP-IDF environment;
no target build was repeated. `git diff --check` and the changed-file credential
and private-path scan passed.

The installed app has booted and reconnected, as reported by the original device
task. Physical four-direction touch/gesture checks and Auto axis mapping remain
pending user confirmation. Axis signs were inferred from the official PCB
drawing; this does not establish physical acceptance. Check all four directions,
manual/Auto switching, flat/shake hold, sleep/wake, screen edges, button hints
and frame/voice performance before treating the feature as hardware-validated.

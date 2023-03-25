---
layout: page
title: A simple sequencer with MIDI and CV/gate output
---

[Schematic](https://6px.eu/seeq/fabrication/seeq-schematic.pdf)
[Gerber files](https://6px.eu/seeq/fabrication/seeq-zip.zip)
[Download BOM](https://6px.eu/seeq/fabrication/seeq-bom.csv)
[View BOM](https://6px.eu/seeq/fabrication/seeq-bom.html)

![3D View top](https://6px.eu/seeq/fabrication/seeq-3D_top.png)

![3D View bottom](https://6px.eu/seeq/fabrication/seeq-3D_bottom.png)

This project is a simple sequencer I designed and built. I started a project to build an analog synthesizer, but I did not have anything that would be able to output the necessary CV signals, so I decided to build one.

And then I added MIDI input and output to be able to interface with other things.

The way it works is fairly simple.

You have 16 steps, with one key and one LED for each step. Turning the knob in one direction makes the tempo faster, and in the other direction makes it slower.

If you press a key while turning the knob, this changes the pitch of that step.

If you turn the knob while pressing on it, it changes the number of steps in the sequence.

If you turn the knob while pressing on it while having a step switch pressed, it changes the velocity of that (this only has an effect for the MIDI output)

If you press the knob quickly once, it pauses or restarts playback.

If you press the knob twice, you enter recording mode. This then takes MIDI input (over USB or the midi in plug) and assigns the note you play to the current step and advances to the next step.
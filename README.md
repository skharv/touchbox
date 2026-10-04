## Touchbox 

This is my project for creating a smashbox style controller using the Waveshare RP2040-Touch-LCD-1.28

This is all very basic and built based on the examples found in the crates I've used.

Currently in a constant state of iteration, so nothing is final.

### MIDI mode

The controller also has a USB MIDI mode for playing scale notes, chords, and a latched bass drone. When the mode selection menu appears, press the MCP1 A7 menu button to select MIDI. See [the MIDI mode guide](docs/midi-mode.md) for the current button mapping and settings.

### Converting Images for use on the screen

I convert .png files to .raw using this ffmpeg command:

```
ffmpeg -i input.png \
  -f rawvideo \
  -pix_fmt rgb565be \
  output.raw
```

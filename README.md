## Touchbox 

This is my project for creating a smashbox style controller using the Waveshare RP2040-Touch-LCD-1.28

This is all very basic and built based on the examples found in the crates I've used.

Currently in a constant state of iteration, so nothing is final.

### Converting Images for use on the screen

I convert .png files to .raw using this ffmpeg command:

```
ffmpeg -i input.png \
  -f rawvideo \
  -pix_fmt rgb565be \
  output.raw
```

## Touchbox 

This is my project for creating a smashbox style controller using the Waveshare RP2040-Touch-LCD-1.28

This is all very basic and built based on the examples found in the crates I've used.

Currently in a constant state of iteration, so nothing is final.

### Converting Images for use on the screen

Converting a bmp to .raw in GIMP will give you an rgb32 style .raw image. This will cause big graphical issues. You need to then convert them again to rgb565. I did this using the below ffmpeg command:

```
ffmpeg -vcodec rawvideo -f rawvideo -pix_fmt rgb32 -s 240x240 -i input.raw -f image2 -vcodec rawvideo -pix_fmt rgb565 out.raw
```

You could probably convert the bmp directly, but I've not tried yet.

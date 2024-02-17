# Bubble Level
 An little Arduino project to combine an ESP32S3 dev board (Sunton ESP32-1732S019) and Sparkfun ICM-20948 accelerometer breakout board into a minature digital bubble level.

The dev board has a 1.9-inch 170*320 lcd screen.  This is setup with TFT_eSPI to display two graphical bubble gauges showing x and y orientation with angle readout below.  The boot push button is used to zero the level to current orientation.  QWIIC cable was connected right to the dev board to attached to the accelerometer board.  Dev board and ICM-20948 breakout board were both mounted to laser cut piece of 3mm thick acrylic sheet.

Filtering could be improved but was good enough for intended purpose.

![](https://github.com/mulcmu/bubble_level/blob/32aac6afe0dd3afef6ec3096fcc7d17c9af5880a/bubble%20level%20on%20delta.jpg)

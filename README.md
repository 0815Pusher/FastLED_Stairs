# FastLED_Stairs thanks @romi06 to start with a nice Code...
My Stairs at Home, 15Leds per Step, 8 Steps 2xPIR 1xLDR_BH1750

Configs: Stair/LEDS_PER_STAIR, LightSensor On/Off, LDRThreshold

Button / Top PIR Fire up the LEDs if the LDR Threshold OK (LDRThreshold)

switch useLDR true/false to use or not use a Light Sensor (useLDR)

(numReadings) Define the number of samples to keep track of. The higher the number, the more the readings will be smoothed, but the slower the output will respond to the input.
For our use case (determine the ammout of light) smoothing is good, so walk-by the LDR sensor or a sensor read spike is ingnored.

Show that stair will not turn on / on PIR detection, due to LDR logic (daylight mode detected)

NOTE: These "Serial" statement can be deleted when everything works fine. (Serial.println(LDRValue);)
its here for finetuning the LDRThreshold value, which you should configure in the begin of this file.

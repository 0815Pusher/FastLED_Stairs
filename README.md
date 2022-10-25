# FastLED_Stairs thanks @romi06 to start with a nice Code...
My Stairs at Home, 15Leds per Step, 8 Steps 2xPIR 1xLDR_BH1750

Configs: Stair/LEDS_PER_STAIR, LightSensor On/Off, LDRThreshold

Button / Top PIR Fire up the LEDs if the LDR Threshold OK (LDRThreshold)

switch useLDR true/false to use or not use a Light Sensor (useLDR)

Show that stair will not turn on / on PIR detection, due to LDR logic (daylight mode detected)

NOTE: These "Serial" statement can be deleted when everything works fine. (Serial.println(LDRValue);)
its here for finetuning the LDRThreshold value, which you should configure in the begin of this file.

10_2021/ Switch from Arduino to VisualCode, Implement the LDR Sensor...

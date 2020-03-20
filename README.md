# dŵr controller

"dŵr controller" ("dŵr") is a relatively simple closed loop snippet to provide two basic functionalities to a turntable drive system - (1) RPM counter and (2) feedback for an external motor controller.

It also aims to do some psychoacoustic majic in between code loops.

## Features

- Calculates RPM from up to 12 (*) control points on a turntable platter via a single sensor;
- Handles an external controller through a rough PI loop (running speed, speed mode (33/45) and control method);
- Rough PI parameter auto-tune;
- Dynamically calculates a standardized wow value, averaged over the control cycle.;

(*) Technically, can be as many as needed, or as many as can physically fit on the turntable platter.

## Description

"dŵr" acts principally as a tachometer, utilizing a combination of magnets attached to the underside of a turntable drive platter, and a single fixed location hall effect sensor, connected to an Arduino (Nano for most of principal development). This information is continuously displayed on an OLED screen controlled via I2C.

Assembled data derives a difference quotient from desired speed which is then used to send commands to an external controller. (**)

(**) Development has revolved mostly around the SG4 sinewave generator, courtesy of Pyramid on DIYAudio.com.

Additional features include:

- Calculation of wow as a key component of W&F;
- Recording of standard play time to gauge cartridge life;
- Dynamic parameter logging for latter analyses. 

## Dependencies

- [OneButton](https://github.com/mathertel/OneButton) 
- [U8G2](https://github.com/olikraus/u8g2)

## Hardware

- Arduino Nano (or similar)
- Hall Effect Sensor (or an optical sensor)
- Magnets (or strips of reflective tape for an optical sensor)
- OLED (using a SSD1306 based 0.91" via I2C)
- BC337 (or similar NPN transistor) (***)
- 1kOhm resistors

(***) Replaced CD4066B switch as it was unreliable in this application.
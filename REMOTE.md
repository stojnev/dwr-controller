# dŵr controller

"dŵr controller" ("dŵr") uses two pushbuttons to access / activate various features. In order to have a minimal number of buttons, yet retain full functionality, dŵr relies on a combination of (1) various button events (click, double click) and (2) turntable running state (spinning, not spinning).

* [LSP] - Left button press;
* [LDP] - Left button hold press;
* [RSP] - Right button press;
* [RDP] - Right button hold press;

Left button refers to momentary switch typically connected to pin 4, right button refers to switch typically connected to pin 5.

## Platter Not Spinning

* [LSP] Start rotation.
* [RDP] Access play time for cartridge life.

## Timer Mode Active

* [LDP] Reset play time for cartridge life.
* [RDP] Exit timer mode.

## Platter Spinning

* [LSP] Stop rotation.
* [LDP] Show calculated W&F.
* [RSP] Change rotation speed (33/45).
* [RDP] Switch between automatic speed regulation and manual speed regulation (no feedback).
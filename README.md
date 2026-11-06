# Quanta Test (QT-1.0)

Hardware test program for the board "Quanta 4 Channel Driver_BK FIXED4" V1.0.
It uses the same chip drivers and pin settings as the Quanta firmware (lib/QuantaHal/src/config.h).

## Run it
1. Plug in the **15 V adapter** (the outputs need it) and the USB/programmer cable.
2. Plug the LED test board into all 4 jacks.
3. VS Code > PlatformIO: **Upload**, then **Serial Monitor**.
4. The automatic tests run by themselves and print a results table.
5. Type **a** + Enter to run everything, including the status LED and buttons (they need you to look and press).

## What each result means
- **PASS**: works.
- **FAIL**: something is wrong. The line says what to check.
- **CHECK**: works but needs a look (for example, no LED current on an output, or the battery not proven yet).

## Battery clock test (needs a power cut)
1. Type **w** and enter your Wi-Fi, so the test sets the clock from the internet.
2. Unplug **all** power (15 V and USB) for 1 minute.
3. Plug back in. "Clock battery" shows PASS if the CR2032 kept the time.

## Commands
a everything · q automatic only · s I2C/DAC/ADC · o outputs with numbers · v each LED fades in turn ·
p 3 50 (OUT3 at 50 %) / p all 100 / p off · c battery clock · t internet time · w Wi-Fi · l status LED · b buttons · r results

## Without a computer
After start-up the status LED shows: green = all passed, amber = something to check, blinking red = a failure.
Each short press of SW1 lights the next output at full: OUT1, OUT2 ... OUT8, all on, off.

## Output readings
The monitor reads the op-amp side of each output's 1k resistor, so
load current (mA) = (op-amp V - set V) / 1k. An LED pulls current, so its op-amp reading is higher than the set voltage.
If an LED pulls more than ~3.5 mA the op-amp reaches its limit near 10 V; the test reports that as CHECK, not a fault.
If an output's own monitor doesn't move but another one does, the channel order differs from DAC_MAP / MON_MAP in config.h.


esp32->orange->red(don't use if 15v in)->blk->ylw->blu->grey
loader...  usbc left   ylw,blu,grey
                       org,blk,red                        
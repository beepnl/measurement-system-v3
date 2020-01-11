# BEEP - base v3
This is a repo containing the files to create the open source hardware for the Beep measurement system version 3.

This version features a:

- Stainless steel (easy to assemble) frame
- 1 weight sensor
- 1-5 temperature sensors
- 1 mic for on board audio FFT analysis
- Custom developed PCB (by Ideetron) with Nordic nRF BLE and LoRa


## Measurement system parts

The measurement system frame holds the weight sensor and the electonics box housing, and functions as a stiff weighing frame construction.

![BEEP base v3](https://github.com/beepnl/measurement-system-v3/raw/master/hardware/beep-base-v3.png)

### hardware/frame
The folder 'frame' contains the drawings of the stainless steel parts and the standard waterproof casing drill holes. 

### hardware/pcb
The pcb of the measurement system contains a lot of features, that can be seen in the image below:
![BEEP base - PCB v3](https://github.com/beepnl/measurement-system-v3/blob/master/hardware/pcb/beep-pcb-v3.jpg)

The folder contains the gerber files and the schematic. A print layout is added to see find the exact wire locations of the screw terminals

With the Gerber files, you can directly order a PCB. It contains all files required for the manufacturing of the PCB. 

## Assembly

The assembly is straight forward and can be seen in this video:
[BEEP base - Assembly](https://youtu.be/ZGfoobvGa-Y)

## Firmware
In the 'firmware' folder, the HEX bootloader and the standard BEEP application are available.

You can update the firmware over the air via the Nordic NRF connect app. Then you have the full BLE and LoRa API to your possession to configre the PCB as you like. See the [manual](https://github.com/beepnl/measurement-system-v3/blob/master/firmware/BEEP_base_firmware_v1.3.2_manual_NL.pdf) for all firmware and BLE API features. You will be amazed :-) 

## Improvements
If you have improvements, or ideas for creating a better frame, please send an e-mail to pim@beep.nl explaining your thoughts. Or just fork this repo, and do a pull request with you adjustments, or additions.

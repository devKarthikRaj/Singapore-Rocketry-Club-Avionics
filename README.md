High Altitude Telemetry System via The Automatic Packet Reporting System

# System Overview
LightAPRS is the sole transmitter being used throughout all systems. However, on the receiving end, multiple different receiver architectures and software have been tried.

## GNU Radio + Direwolf (Pluto SDR)
LightAPRS > Siretta Delta RF ANT 151MHZ/173MHZ WHIP STR Antenna > Air > Nagoya NA-771 or Stock 900MHz antenna > Adalm Pluto SDR > GNU Radio > VB-Audio Cable > Direwolf > aprs.fi

GNU Radio demod was fine when quadrature decoding was being used. APRS tone was nice and clean. NBFM, WBFM demod did not work nicely, APRS tone was not clear and was very soft.

Flow of audio from GNU Radio to Direwolf through VB-Audio cable was tested using Audacity - Was able to verify that the audio was in fact going through. Direwolf also indicated that it was reeiving audio on Ch0 (50-90 audio level).

## GQRX Only (Pluto SDR)
LightAPRS > Siretta Delta RF ANT 151MHZ/173MHZ WHIP STR Antenna > Air > Nagoya NA-771 > Adalm Pluto SDR > GQRX > AFSK1200 (In built APRS decoder inside GQRX)

GQRX was receiving the APRS tone cleanly. With squlech set to -60dB, the noise was removed as well but again AFSK1200 decoding did not work! Many videos on YouTube make it look easy but it did not work here.

## GQRX to Direwolf (Pluto SDR)
LightAPRS > Siretta Delta RF ANT 151MHZ/173MHZ WHIP STR Antenna > Air > Nagoya NA-771 > Adalm Pluto SDR > GQRX > VB-Audio Cable > Direwolf > aprs.fi

Flow of audio from GQRX to Direwolf through VB-Audio cable was tested using Audacity - Was able to verify that the audio was in fact going through. Direwolf also indicated that it was reeiving audio on Ch0 (50-90 audio level).

## GQRX Only (Baofeng UV17M)
LightAPRS > Siretta Delta RF ANT 151MHZ/173MHZ WHIP STR Antenna > Air > Nagoya NA-771 > Baofeng UV17M > Direwolf

Baofeng is able to receive APRS cleanly and the APRS tone is clean. However When the Baofeng audio was fed to Direwolf directly from speaker to mic (audio enhancements turned off to prevent Windows from modifying the APRS tone) - APRS decoding did not work. Connecting Baofeng to Direwolf through the laptop audio jack did not work as well.

# Uploading Code to the LightAPRS
## Why Arduino CLI?
* LightAPRS runs off an ATmega1284 MCU.
* The only way to upload code to this onboard MCU is through the USB port on the LightAPRS.
LightAPRS seems to be using an Optiboot-style UART bootloader (used by MightCore on ATmega1284p).
* The timing sequence of events expected by this Optiboot is very specific.
* It is extremely time sensitive aka "Bootloader Timing Hell" !!!
* This is an AVR rite of passage.
* This has cost the former SRC team countless debugging hours.
* The easiest most straightforward workaround that is guaranteed (?) to work is to use the Arduino CLI to interact with the LightAPRS. Do not use Arduino IDE 1.0 and 2.0. It will not work.
* Arduino CLI has minimal overhead - no delay - direct interaction with the underlying commands - No overhead from GUI - Uploads immediately without waiting for the computer to re-identify (re-enumerate) the COM port and then upload.

# How to use Arduino CLI?
Execute the below commands in the same folder as the Arduino code (.ino file).

## Compile Arduino CLI
arduino-cli compile --fqbn MightyCore:avr:1284:bootloader=uart0,BOD=2v7,LTO=Os,clock=8MHz_external,variant=modelP,pinout=standard .

Before proceeding, identify the COM port that the LightAPRS is connected to from the device manager.

## Upload with Arduino CLI
arduino-cli upload -p COM12 --fqbn MightyCore:avr:1284:bootloader=uart0,BOD=2v7,LTO=Os,clock=8MHz_external,variant=modelP,pinout=standard --verify --verbose

## Serial Monitor with Arduino CLI
arduino-cli monitor -p COM12 -c baudrate=57600

# Further Troubleshooting
* The problem seems to be deeper than we thought. The fact that AFSK1200 is not decoding APRS packets inside GQRX and Direwolf is also not decoding APRS packets is worrying.
* The next step would be to find a clean [APRS tone](https://commons.wikimedia.org/wiki/File%3AAFSK_1200_baud.ogg?utm_source=chatgpt.com) and record this as a WAV file and playback to Direwolf and see if it can decode it. 
* If Direwolf can decode this clean sample APRS tone then the problem is with the LightAPRS transmitter - Analyse the LightAPRS code and tweak the SA818V settings to fix it.
* If Direwolf cannot decode this clean sample APRS tone then the problem is with the receiver side DSP.
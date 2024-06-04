# ts-streamer
This app implements a simple yet flexible modem targetted at MPEG Transport Stream packets.

## Dependencies
 - [liquid-dsp](https://github.com/vankxr/liquid-dsp)
 - [SoapySDR](https://github.com/pothosware/SoapySDR)

## Build instructions
    make

## Usage
    Usage: icyradio-ts-streamer [-h] [-d] [-q] [-z] [-t] [-r] [-g <gain_dB>] [-a] [-f <freq>] [-s <rate>] [-n <num>]
      -h: Print this help message
      -d: Dummy mode (generate random TX data and discard RX data)
      -q: Quiet mode (no verbose debug output)
      -z: No radio mode (TX outputs samples to stdout and RX reads samples from stdin)
      -t: TX mode
      -r: RX mode (default)
      -g <gain_dB>: RX gain (default: 0 dB) / TX attenuation (default: 10 dB)
      -a: RX use AGC
      -f <freq>: Carrier frequency (default: 433 MHz)
      -s <rate>: Sample rate (default: 2.4 Msps)
      -n <num>: Number of TS frames per packet (default: 32)
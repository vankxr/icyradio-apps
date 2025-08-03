#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <liquid/liquid.h>
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include "debug_macros.h"
#include "framegen.h"
#include "framesync.h"

#define TS_PACKET_SYNC 0x47
#define TS_PACKET_LEN 187 // 188 bytes per TS packet, sync byte extracted

#define BW_FRACTION 0.75 // Fraction of the sample rate to use as bandwidth

uint8_t g_ubDummyMode = 0;
uint8_t g_ubQuietMode = 0;
volatile sig_atomic_t g_iStop = 0;

void signal_detected_cb(void *ptr, const framesync_stats_t *stats)
{
    DBGPRINTLN_CTX("Signal detected (RSSI: %.2f dB)", stats->rssi);
}
void signal_lost_cb(void *ptr, const framesync_stats_t *stats)
{
    DBGPRINTLN_CTX("Signal lost (RSSI: %.2f dB)", stats->rssi);
}
void sync_cb(void *ptr, const framesync_stats_t *stats)
{
    DBGPRINTLN_CTX("Preamble synchronized (RSSI: %.2f dB, CFO: %.2f %%Fs, Rot: %.0f deg, Corr: %.2f %%)", stats->rssi, stats->cfo * 100, stats->phi * 180 / M_PI, stats->rxy * 100);
}
uint8_t packet_received_cb(void *ptr, const framesync_stats_t *stats, const uint8_t *header, const uint8_t header_valid, const uint8_t *payload, const size_t payload_len, const uint8_t payload_valid)
{
    if(!header_valid)
        return 1;

    if(g_iStop)
        return 0;

    // DBGPRINTLN_CTX("Packet received (H %s, P: (%lu bytes, %s], RSSI: %.2f dB, EVM: %.2f dB, CFO %.2f %%Fs)", header_valid ? "OK" : "FAIL", payload_len, payload_valid ? "OK" : "FAIL", stats->rssi, stats->evm, stats->cfo * 100);

    static uint64_t ullTotalPackets = 0;
    static uint64_t ullTotalFailedPackets = 0;
    static uint64_t ullTotalLostPackets = 0;
    static uint16_t usLastPacket = 0;
    uint16_t usPacket = *(uint16_t *)header;
    int32_t lLostPackets = (int32_t)usPacket - (int32_t)usLastPacket - 1;
    uint8_t ubPrintStats = !payload_valid || lLostPackets > 0;

    usLastPacket = usPacket;

    ullTotalPackets++;
    ullTotalFailedPackets += !payload_valid;

    if(lLostPackets > 0 && (uint64_t)lLostPackets < ullTotalPackets)
    {
        ullTotalLostPackets += lLostPackets;

        DBGPRINTLN_CTX("At least %d packets lost", lLostPackets);
    }

    if(ubPrintStats)
        DBGPRINTLN_CTX("Total packets: %lu, failed packets: %lu (%.2f %%), lost packets: %lu (%.2f %%)", ullTotalPackets, ullTotalFailedPackets, (double)ullTotalFailedPackets / ullTotalPackets * 100, ullTotalLostPackets, (double)ullTotalLostPackets / ullTotalPackets * 100);

    if(!g_ubQuietMode)
    {
        DBGPRINTLN_CTX("Packet %hu received (H %s, P: [%lu bytes, %s], RSSI: %.2f dB, EVM: %.2f dB, CFO: %.2f %%Fs)", usPacket, header_valid ? "OK" : "FAIL", payload_len, payload_valid ? "OK" : "FAIL", stats->rssi, stats->evm, stats->cfo * 100);

        // DBGPRINT_CTX("  Header: ");
        // for(int i = 0; i < 14; i++)
        //     DBGPRINT("%02X ", pubHeader[i]);
        // DBGPRINTLN("");
    }

    if(!g_ubDummyMode)
    {
        if(payload_len % TS_PACKET_LEN)
            DBGPRINTLN_CTX("Invalid payload length: %lu", payload_len);

        for(size_t i = 0; i < payload_len; i += TS_PACKET_LEN)
        {
            uint8_t sync = TS_PACKET_SYNC;

            size_t _ = fwrite(&sync, 1, 1, stdout);

            if(!payload_valid)
                *(uint8_t *)(payload + i) |= 0x80; // Set the TEI bit

            _ = fwrite(payload + i, TS_PACKET_LEN, 1, stdout);
        }
    }

    return 0;
}
void signal_handler(int iSignal)
{
    if(iSignal == SIGINT)
        DBGPRINTLN_CTX("Kayboard interrupt, stopping...");

    g_iStop = 1;
}

int main(int argc, char *argv[])
{
    srand(time(NULL));

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    signal(SIGINT, signal_handler);

    // Parse arguments
    uint8_t ubNoRadio = 0;
    uint8_t ubTXnRX = 0;
    double fGain = 0.0;
    uint8_t ubAGC = 0;
    double fCarrierFreq = 433e6;
    double fSampleRate = 2e6;
    size_t ulNumTSFramesPerPacket = 32;
    char *pszAntenna = NULL;
    size_t ulChannel = 0; // Inferred from antenna
    char *pszSoapyDeviceArgs = NULL;

    int iOpt;
    while(optind < argc)
    {
        if((iOpt = getopt(argc, argv, "hdqztrg:af:s:n:w:")) != EOF)
        {
            switch(iOpt)
            {
                case 'h':
                {
                    DBGPRINTLN("Usage: %s [-h] [-d] [-q] [-z] [-t] [-r] [-g <gain_dB>] [-a] [-f <freq>] [-s <rate>] [-n <num>] [-w <antenna>] [<enum_args>]", argv[0]);
                    DBGPRINTLN("  -h: Print this help message");
                    DBGPRINTLN("  -d: Dummy mode (generate random TX data and discard RX data)");
                    DBGPRINTLN("  -q: Quiet mode (no verbose debug output)");
                    DBGPRINTLN("  -z: No radio mode (TX outputs samples to stdout and RX reads samples from stdin)");
                    DBGPRINTLN("  -t: TX mode");
                    DBGPRINTLN("  -r: RX mode (default)");
                    DBGPRINTLN("  -g <gain_dB>: RX gain (default: 0 dB) / TX attenuation (default: 10 dB)");
                    DBGPRINTLN("  -a: RX use AGC");
                    DBGPRINTLN("  -f <freq>: Carrier frequency (default: 433 MHz)");
                    DBGPRINTLN("  -s <rate>: Sample rate (default: 2.4 Msps)");
                    DBGPRINTLN("  -n <num>: Number of TS frames per packet (default: 32)");
                    DBGPRINTLN("  -w <antenna>: Antenna (default: RX1A / TX1A)");
                    DBGPRINTLN("  <enum_args>: SoapySDR device enumeration arguments");

                    return 0;
                }
                break;
                case 'd':
                {
                    g_ubDummyMode = 1;
                }
                break;
                case 'q':
                {
                    g_ubQuietMode = 1;
                }
                break;
                case 'z':
                {
                    ubNoRadio = 1;
                }
                break;
                case 't':
                {
                    ubTXnRX = 1;

                    if(!pszAntenna)
                    {
                        pszAntenna = "TX1A";
                        ulChannel = 0;
                    }
                }
                break;
                case 'r':
                {
                    ubTXnRX = 0;

                    if(!pszAntenna)
                    {
                        pszAntenna = "RX1A";
                        ulChannel = 0;
                    }
                }
                break;
                case 'g':
                {
                    fGain = atof(optarg);
                }
                break;
                case 'a':
                {
                    ubAGC = 1;
                }
                break;
                case 'f':
                {
                    fCarrierFreq = atof(optarg);
                }
                break;
                case 's':
                {
                    fSampleRate = atof(optarg);
                }
                break;
                case 'n':
                {
                    int n = atoi(optarg);

                    ulNumTSFramesPerPacket = n <= 0 ? 32 : n;
                }
                break;
                case 'w':
                {
                    pszAntenna = optarg;

                    if(!strncmp(pszAntenna, "RX1", 3) || !strncmp(pszAntenna, "TX1", 3))
                        ulChannel = 0;
                    else if(!strncmp(pszAntenna, "RX2", 3) || !strncmp(pszAntenna, "TX2", 3))
                        ulChannel = 1;
                    else
                        ulChannel = 0;
                }
                break;
                default:
                    exit(-1);
            }
        }
        else
        {
            if(pszSoapyDeviceArgs)
                DBGPRINTLN_CTX("Ignoring extra argument: %s", argv[optind++]);
            else
                pszSoapyDeviceArgs = argv[optind++];
        }
    }

    // Init device
    size_t ulDeviceCount = 0;
    SoapySDRKwargs *pResults = NULL;
    SoapySDRDevice *pSDR;

    if(!ubNoRadio)
    {
        SoapySDRKwargs xArgs = {};

        DBGPRINTLN_CTX("Searching for devices%s%s%s...", pszSoapyDeviceArgs ? " with args: \"" : "", pszSoapyDeviceArgs ? pszSoapyDeviceArgs : "", pszSoapyDeviceArgs ? "\"" : "");

        if(pszSoapyDeviceArgs)
            xArgs = SoapySDRKwargs_fromString(pszSoapyDeviceArgs);

        // SoapySDRKwargs_set(&xArgs, "driver", "icyradio");
        // SoapySDRKwargs_set(&args, "serial", "");

        pResults = SoapySDRDevice_enumerate(&xArgs, &ulDeviceCount);

        SoapySDRKwargs_clear(&xArgs);
    }

    if(ubNoRadio)
    {
        DBGPRINTLN_CTX("No radio mode, using stdin/stdout for RX/TX...");

        pSDR = NULL;
    }
    else if(!ulDeviceCount)
    {
        DBGPRINTLN_CTX("No devices found, using dummy data for RX and discarding TX!");

        pSDR = NULL;
    }
    else
    {
        if(ulDeviceCount > 1)
            DBGPRINTLN_CTX("Multiple devices found, using the first one...");

        pSDR = SoapySDRDevice_make(&pResults[0]);

        if(!pSDR)
        {
            SoapySDRKwargsList_clear(pResults, ulDeviceCount);

            DBGPRINTLN_CTX("Error making device!");

            return -1;
        }
    }

    if(!ubNoRadio)
        SoapySDRKwargsList_clear(pResults, ulDeviceCount);

    // Modulation parameters
    framegenprops_t xFrameHeaderProps = {
        .crc = LIQUID_CRC_8,
        .i_fec = LIQUID_FEC_NONE,
        .o_fec = LIQUID_FEC_NONE,
        .mod = LIQUID_MODEM_QPSK,
        .pilots = 0,
    };
    framegenprops_t xFramePayloadProps = {
        .crc = LIQUID_CRC_8,
        .i_fec = LIQUID_FEC_CONV_V29P56,
        .o_fec = LIQUID_FEC_RS_M8_DVB,
        .mod = LIQUID_MODEM_QAM64,
        .pilots = 0,
    };

    uint64_t ullNumSamples = 0;
    double dTotalTime = 0.0;

    if(ubTXnRX) // TX
    {
        framegen_t *xFrameGen = framegen_create(&xFrameHeaderProps, 2);

        framegen_set_payload_props(xFrameGen, &xFramePayloadProps);

        uint8_t *pubPayload = (uint8_t *)malloc(TS_PACKET_LEN * ulNumTSFramesPerPacket);
        uint16_t usPacketNumber = 0;

        // Print stats
        {
            // Header
            size_t ulHeaderLen = 2 + 6; // Packet number (2 bytes) + modulation (6 bytes)
            size_t ulHeaderSymbols = ceilf((float)ulHeaderLen * 8 / modulation_types[xFrameHeaderProps.mod].bps);
            size_t ulCodedHeaderLen = fec_get_enc_msg_length(xFrameHeaderProps.i_fec, fec_get_enc_msg_length(xFrameHeaderProps.o_fec, ulHeaderLen + ((1 << (size_t)xFrameHeaderProps.crc) / 8)));
            size_t ulCodedHeaderSymbols = ceilf((float)ulCodedHeaderLen * 8 / modulation_types[xFrameHeaderProps.mod].bps);
            float fHeaderCodingEfficiency = (float)ulHeaderLen / ulCodedHeaderLen;

            // Payload
            size_t ulPayloadLen = TS_PACKET_LEN * ulNumTSFramesPerPacket;
            size_t ulPayloadSymbols = ceilf((float)ulPayloadLen * 8 / modulation_types[xFramePayloadProps.mod].bps);
            size_t ulCodedPayloadLen = fec_get_enc_msg_length(xFramePayloadProps.i_fec, fec_get_enc_msg_length(xFramePayloadProps.o_fec, ulPayloadLen + ((1 << (size_t)xFramePayloadProps.crc) / 8)));
            size_t ulCodedPayloadSymbols = ceilf((float)ulCodedPayloadLen * 8 / modulation_types[xFramePayloadProps.mod].bps);
            float fPayloadCodingEfficiency = (float)ulPayloadLen / ulCodedPayloadLen;

            // Ratio of useful data (payload) to total data (header + payload)
            float fTotalCodingEfficiency = (float)ulPayloadLen / (ulCodedHeaderLen + ulCodedPayloadLen);

            // Assembly dummy packet to get the total number of symbols
            size_t ulPacketDataSymbols = ulCodedHeaderSymbols + ulCodedPayloadSymbols;

            framegen_assemble(xFrameGen, (uint8_t *)&usPacketNumber, pubPayload, TS_PACKET_LEN * ulNumTSFramesPerPacket);

            size_t ulPacketSamples = framegen_get_sample_count(xFrameGen);
            size_t ulPacketSymbols = framegen_get_symbol_count(xFrameGen);

            framegen_reset(xFrameGen);

            size_t ulInterpolation = ulPacketSamples / ulPacketSymbols;
            float fSymbolRate = fSampleRate / ulInterpolation;

            // Ratio of coded data (header + payload) to total data (preamble + header + payload + etc...)
            float fFramerEfficiency = (float)ulPacketDataSymbols / ulPacketSymbols;

            // Ratio of useful data (uncoded payload) to total data (preamble + header + payload + etc...)
            float fTotalEfficiency = fFramerEfficiency * fTotalCodingEfficiency;

            // Time for one packet
            float fPacketTime = (float)ulPacketSamples / fSampleRate;
            float fPayloadRate = (float)ulPayloadLen * 8 / fPacketTime;
            float fTSMuxRate = (float)(ulPayloadLen + ulNumTSFramesPerPacket) * 8 / fPacketTime; // Include the sync byte since we also receive it from the muxer

            DBGPRINTLN_CTX("Header data length: %lu bytes (%lu symbols)", ulHeaderLen, ulHeaderSymbols);
            DBGPRINTLN_CTX("Coded header length: %lu bytes (%lu symbols)", ulCodedHeaderLen, ulCodedHeaderSymbols);
            DBGPRINTLN_CTX("Header coding efficiency: %.2f %%", fHeaderCodingEfficiency * 100);
            DBGPRINTLN_CTX("--------------------");
            DBGPRINTLN_CTX("Payload data length: %lu bytes (%lu symbols)", ulPayloadLen, ulPayloadSymbols);
            DBGPRINTLN_CTX("Coded payload length: %lu bytes (%lu symbols)", ulCodedPayloadLen, ulCodedPayloadSymbols);
            DBGPRINTLN_CTX("Payload coding efficiency: %.2f %%", fPayloadCodingEfficiency * 100);
            DBGPRINTLN_CTX("Total payload coding efficiency: %.2f %%", fTotalCodingEfficiency * 100);
            DBGPRINTLN_CTX("--------------------");
            DBGPRINTLN_CTX("Packet data length: %lu symbols", ulPacketDataSymbols);
            DBGPRINTLN_CTX("Packet length: %lu symbols", ulPacketSymbols);
            DBGPRINTLN_CTX("Framer efficiency: %.2f %%", fFramerEfficiency * 100);
            DBGPRINTLN_CTX("Total efficiency (payload only): %.2f %%", fTotalEfficiency * 100);
            DBGPRINTLN_CTX("--------------------");
            DBGPRINTLN_CTX("Sampling rate: %.2f Msps", fSampleRate / 1e6);
            DBGPRINTLN_CTX("Symbol rate: %.2f Msps", fSymbolRate / 1e6);
            DBGPRINTLN_CTX("Packet time: %.6f ms", fPacketTime * 1e3);
            DBGPRINTLN_CTX("Payload bitrate: %.6f kbps", fPayloadRate / 1e3);
            DBGPRINTLN_CTX("Bandwidth efficiency: %.6f bits/s/Hz", fPayloadRate / (fSymbolRate * 1.2f));
            DBGPRINTLN_CTX("TS Mux bitrate: %.0f bps", ceilf(fTSMuxRate));
        }

        if(pSDR)
        {
            SoapySDRDevice_setSampleRate(pSDR, SOAPY_SDR_TX, ulChannel, fSampleRate);
            SoapySDRDevice_setBandwidth(pSDR, SOAPY_SDR_TX, ulChannel, fSampleRate * BW_FRACTION);
            SoapySDRDevice_setFrequency(pSDR, SOAPY_SDR_TX, ulChannel, fCarrierFreq, NULL);
            SoapySDRDevice_setAntenna(pSDR, SOAPY_SDR_TX, ulChannel, pszAntenna);
            SoapySDRDevice_setGain(pSDR, SOAPY_SDR_TX, ulChannel, fGain);

            DBGPRINTLN_CTX("Transmitting on %.2f MHz at %.2f ksps with %.2f kHz of bandwidth and %.2f dB of gain via antenna %s...", fCarrierFreq / 1e6, fSampleRate / 1e3, fSampleRate * BW_FRACTION / 1e3, fGain, pszAntenna);
        }

        SoapySDRStream *pStream = NULL;
        size_t ulMTU = 1024;

        if(pSDR)
        {
            size_t ulChannels[] = {ulChannel};

            pStream = SoapySDRDevice_setupStream(pSDR, SOAPY_SDR_TX, SOAPY_SDR_CF32, ulChannels, sizeof(ulChannels) / sizeof(ulChannels[0]), NULL);
            ulMTU = SoapySDRDevice_getStreamMTU(pSDR, pStream);
        }

        float complex *pfBuffer = (float complex *)malloc(ulMTU * sizeof(float complex));

        while(!g_iStop)
        {
            if(!g_ubDummyMode)
            {
                for(size_t i = 0; i < ulNumTSFramesPerPacket; i++)
                {
                    uint8_t sync;

                    size_t _ = fread(&sync, 1, 1, stdin);

                    if(sync != TS_PACKET_SYNC)
                    {
                        DBGPRINTLN_CTX("Invalid sync byte: 0x%02X", sync);

                        break;
                    }

                    _ = fread(pubPayload + i * TS_PACKET_LEN, TS_PACKET_LEN, 1, stdin);
                }
            }
            else
            {
                for(size_t i = 0; i < TS_PACKET_LEN * ulNumTSFramesPerPacket; i++)
                    pubPayload[i] = rand() % 256;
            }

            if(g_iStop)
                break;

            clock_t begin = clock();
            framegen_assemble(xFrameGen, (uint8_t *)&usPacketNumber, pubPayload, TS_PACKET_LEN * ulNumTSFramesPerPacket);
            clock_t end = clock();

            usPacketNumber++;

            if(g_iStop)
                break;

            size_t ulSamplesLeft = framegen_get_sample_count(xFrameGen);

            ullNumSamples += ulSamplesLeft;
            dTotalTime += (double)(end - begin) / CLOCKS_PER_SEC;

            while(ulSamplesLeft)
            {
                size_t ulSamples = ulSamplesLeft > ulMTU ? ulMTU : ulSamplesLeft;

                ulSamples = framegen_write_samples(xFrameGen, pfBuffer, ulSamples);
                ulSamplesLeft -= ulSamples;

                size_t ulOffset = 0;

                while(ulSamples)
                {
                    size_t ulWritten = ulSamples;

                    if(pSDR)
                    {
                        void *pSamples = (void *)&pfBuffer[ulOffset];
                        int iFlags = 0;

                        // End burst only if no samples left
                        if(!ulSamplesLeft)
                            iFlags |= SOAPY_SDR_END_BURST;

                        int iWritten = SoapySDRDevice_writeStream(pSDR, pStream, (const void * const *)&pSamples, ulSamples, &iFlags, 0, 1e6);

                        if(iWritten < 0)
                        {
                            DBGPRINTLN_CTX("Error writing samples: %d", iWritten);

                            break;
                        }

                        ulWritten = iWritten;
                    }
                    else if(ubNoRadio)
                    {
                        ulWritten = fwrite(&pfBuffer[ulOffset], sizeof(float complex), ulSamples, stdout);
                    }

                    ulSamples -= ulWritten;
                    ulOffset += ulWritten;
                }
            }
        }

        if(pSDR)
            SoapySDRDevice_deactivateStream(pSDR, pStream, 0, 0);

        free(pfBuffer);

        if(pSDR)
            SoapySDRDevice_closeStream(pSDR, pStream);

        free(pubPayload);

        framegen_delete(xFrameGen);
    }
    else
    {
        framesync_t *xFrameSync = framesync_create(&xFrameHeaderProps, 2);

        framesync_set_callback(xFrameSync, signal_detected_cb, signal_lost_cb, sync_cb, packet_received_cb, NULL);
        framesync_set_header_soft_demod(xFrameSync, 0);
        framesync_set_payload_soft_demod(xFrameSync, 0);
        framesync_set_eq_strategy(xFrameSync, FRAMESYNC_EQ_STRAT_DEC_DIR);

        if(pSDR)
        {
            SoapySDRDevice_setSampleRate(pSDR, SOAPY_SDR_RX, ulChannel, fSampleRate);
            SoapySDRDevice_setBandwidth(pSDR, SOAPY_SDR_RX, ulChannel, fSampleRate * BW_FRACTION);
            SoapySDRDevice_setFrequency(pSDR, SOAPY_SDR_RX, ulChannel, fCarrierFreq, NULL);
            SoapySDRDevice_setAntenna(pSDR, SOAPY_SDR_RX, ulChannel, pszAntenna);
            SoapySDRDevice_setGainMode(pSDR, SOAPY_SDR_RX, ulChannel, ubAGC);
            SoapySDRDevice_setGain(pSDR, SOAPY_SDR_RX, ulChannel, fGain);

            DBGPRINTLN_CTX("Receiving on %.2f MHz at %.2f ksps with %.2f kHz of bandwidth and %.2f dB of gain via antenna %s...", fCarrierFreq / 1e6, fSampleRate / 1e3, fSampleRate * BW_FRACTION / 1e3, fGain, pszAntenna);
        }

        SoapySDRStream *pStream = NULL;
        size_t ulMTU = 1024;

        if(pSDR)
        {
            size_t ulChannels[] = {ulChannel};

            pStream = SoapySDRDevice_setupStream(pSDR, SOAPY_SDR_RX, SOAPY_SDR_CF32, ulChannels, sizeof(ulChannels) / sizeof(ulChannels[0]), NULL);
            ulMTU = SoapySDRDevice_getStreamMTU(pSDR, pStream);
        }

        float complex *pfBuffer = (float complex *)malloc(ulMTU * sizeof(float complex));

        if(pSDR)
            SoapySDRDevice_activateStream(pSDR, pStream, 0, 0, 0);

        while(!g_iStop)
        {
            size_t ulRead = ulMTU;

            if(pSDR)
            {
                int iFlags = 0;
                long long llTimeNs = 0;

                int iRead = SoapySDRDevice_readStream(pSDR, pStream, (void * const *)&pfBuffer, ulMTU, &iFlags, &llTimeNs, 1e6);

                if(iRead == SOAPY_SDR_OVERFLOW)
                {
                    DBGPRINT("O");

                    continue;
                }
                else if(iRead < 0)
                {
                    DBGPRINTLN_CTX("Error reading samples: %d", iRead);

                    break;
                }

                ulRead = iRead;
            }
            else if(ubNoRadio)
            {
                ulRead = fread(pfBuffer, sizeof(float complex), ulMTU, stdin);
            }

            if(g_iStop)
                break;

            clock_t begin = clock();
            framesync_process_samples(xFrameSync, pfBuffer, ulRead);
            clock_t end = clock();

            ullNumSamples += ulRead;
            dTotalTime += (double)(end - begin) / CLOCKS_PER_SEC;
        }

        if(pSDR)
            SoapySDRDevice_deactivateStream(pSDR, pStream, 0, 0);

        free(pfBuffer);

        if(pSDR)
            SoapySDRDevice_closeStream(pSDR, pStream);

        framesync_delete(xFrameSync);
    }

    DBGPRINTLN_CTX("Total processed samples: %lu", ullNumSamples);
    DBGPRINTLN_CTX("Total processing time: %.2f s", dTotalTime);
    DBGPRINTLN_CTX("Average processing time: %.2f us/sample", dTotalTime / (double)ullNumSamples * 1e6);
    DBGPRINTLN_CTX("Average processing rate: %.2f Msps", (double)ullNumSamples / dTotalTime / 1e6);

    if(pSDR)
        SoapySDRDevice_unmake(pSDR);

    return 0;
}

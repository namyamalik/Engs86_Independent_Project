/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== adcBufContinuousSampling.c ========
 */
#include <stdint.h>
#include <stdio.h>
/* For sleep() */
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/PIN.h>

/* Example/Board Header files */
#include "Board.h"

#define ADCBUFFERSIZE    (500)
#define UARTBUFFERSIZE   (500)

uint16_t sampleBufferOne[ADCBUFFERSIZE];
uint16_t sampleBufferTwo[ADCBUFFERSIZE];
uint32_t microVoltBuffer[ADCBUFFERSIZE];
uint32_t buffersCompletedCounter = 0;
char uartTxBuffer[UARTBUFFERSIZE];

uint16_t samplesArray[ADCBUFFERSIZE];

//**************************  added code for PWM **************************//

PIN_Config pinTable[] =
{
#if defined(Board_CC1350_LAUNCHXL)
 Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
 Board_DIO15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 PIN_TERMINATE
};

/* Pin driver handle */
static PIN_Handle pinHandle;
static PIN_State pinState;

//****************************************************//

/* Driver handle shared between the task and the callback function */
UART_Handle uart;


/*
 * This function is called whenever an ADC buffer is full.
 * The content of the buffer is then converted into human-readable format and
 * sent to the PC via UART.
 */
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion,
    void *completedADCBuffer, uint32_t completedChannel)
{
    uint_fast16_t i;
    uint_fast16_t uartTxBufferOffset = 0;

    /* Adjust raw ADC values and convert them to microvolts */
    ADCBuf_adjustRawValues(handle, completedADCBuffer, ADCBUFFERSIZE,
        completedChannel);
    ADCBuf_convertAdjustedToMicroVolts(handle, completedChannel,
        completedADCBuffer, microVoltBuffer, ADCBUFFERSIZE);

    //**************************  added code for calculations **************************//

     uint16_t a = 0;
     uint16_t b = 0;
     uint64_t sum = 0;
     uint32_t average = 0;
     uint32_t max = 0;

     // loop through the 10 bins
     while (a < (ADCBUFFERSIZE/50)) {
         // loop through the 50 microvoltbuffer values associated with each bin (b<50, 50<b<100, 100<b<150, etc.)
         while (b < (ADCBUFFERSIZE/10 + 50*a)) {
             sum = sum + abs(microVoltBuffer[b]);
             b++; // increment sample number (will do 50 samples for each bin)

             // printf("%u\n", (unsigned int) microVoltBuffer[b]);
         }
         average = sum / 500; // calculate average of the bin
         // to find max average value among the 10 bins
         if (average > max) {
             max = average;
         }
         a++; // increment bin number
     }

     // make pin go high if peak average value over the 2.5 ms sampling interval is greater than a threshold value
     if (max > 150000) { // 150mV
         // digital pin 15 goes high
         PIN_setOutputValue(pinHandle, Board_DIO15, 1);
     }
     else {
         // digital pin 15 goes low
         PIN_setOutputValue(pinHandle, Board_DIO15, 0);
     }

     //****************************************************//

    /* Start with a header message. */
    uartTxBufferOffset = snprintf(uartTxBuffer,
        UARTBUFFERSIZE - uartTxBufferOffset, "\r\nBuffer %u finished.",
        (unsigned int)buffersCompletedCounter++);

//    /* Write raw adjusted values to the UART buffer if there is room. */
//    uartTxBufferOffset += snprintf(uartTxBuffer + uartTxBufferOffset,
//        UARTBUFFERSIZE - uartTxBufferOffset, "\r\nRaw Buffer: ");
//
//    for (i = 0; i < ADCBUFFERSIZE && uartTxBufferOffset < UARTBUFFERSIZE; i++) {
//        uartTxBufferOffset += snprintf(uartTxBuffer + uartTxBufferOffset,
//            UARTBUFFERSIZE - uartTxBufferOffset, "%u,",
//        *(((uint16_t *)completedADCBuffer) + i));
//    }

    /* Write microvolt values to the UART buffer if there is room. */
    if (uartTxBufferOffset < UARTBUFFERSIZE) {
        uartTxBufferOffset += snprintf(uartTxBuffer + uartTxBufferOffset,
            UARTBUFFERSIZE - uartTxBufferOffset, "\r\nMicrovolts: ");

        for (i = 0; i < ADCBUFFERSIZE && uartTxBufferOffset < UARTBUFFERSIZE; i++) {
            uartTxBufferOffset += snprintf(uartTxBuffer + uartTxBufferOffset,
                UARTBUFFERSIZE - uartTxBufferOffset, "%u,",
                (unsigned int)microVoltBuffer[i]);
        }

//        for (i = 0; i < ADCBUFFERSIZE/50 && uartTxBufferOffset < UARTBUFFERSIZE/50; i++) {
//                    uartTxBufferOffset += snprintf(uartTxBuffer + uartTxBufferOffset,
//                        UARTBUFFERSIZE/50 - uartTxBufferOffset, " Max is %u,",
//                        (unsigned int) max);
//        }
    }

    /*
     * Ensure we don't write outside the buffer.
     * Append a newline after the data.
     */
    if (uartTxBufferOffset < UARTBUFFERSIZE) {
        uartTxBuffer[uartTxBufferOffset++] = '\n';
    }
    else {
        uartTxBuffer[UARTBUFFERSIZE-1] = '\n';
    }

    /* Display the data via UART */
    UART_write(uart, uartTxBuffer, uartTxBufferOffset);
}

/*
 * Callback function to use the UART in callback mode. It does nothing.
 */
void uartCallback(UART_Handle handle, void *buf, size_t count) {
   return;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    UART_Params uartParams;
    ADCBuf_Handle adcBuf;
    ADCBuf_Params adcBufParams;
    ADCBuf_Conversion continuousConversion;

    /* Open LED pins */
    pinHandle = PIN_open(&pinState, pinTable);
    if (pinHandle == NULL) {
        while(1);
    }

    /* Call driver init functions */
    ADCBuf_init();
    UART_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.writeMode = UART_MODE_CALLBACK;
    uartParams.writeCallback = uartCallback;
    uartParams.baudRate = 115200;
    uart = UART_open(Board_UART0, &uartParams);

    /* Set up an ADCBuf peripheral in ADCBuf_RECURRENCE_MODE_CONTINUOUS */
    ADCBuf_Params_init(&adcBufParams);
    adcBufParams.callbackFxn = adcBufCallback;
    adcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_CONTINUOUS;
    adcBufParams.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    adcBufParams.samplingFrequency = 200000; // 200kHz
    adcBuf = ADCBuf_open(Board_ADCBUF0, &adcBufParams);

    /* Configure the conversion struct */
    continuousConversion.arg = NULL;
    continuousConversion.adcChannel = Board_ADCBUF0CHANNEL0;
    continuousConversion.sampleBuffer = sampleBufferOne;
    continuousConversion.sampleBufferTwo = sampleBufferTwo;
    continuousConversion.samplesRequestedCount = ADCBUFFERSIZE;

    if (adcBuf == NULL){
        /* ADCBuf failed to open. */
        while(1);
    }

    /* Start converting. */
    if (ADCBuf_convert(adcBuf, &continuousConversion, 1) !=
        ADCBuf_STATUS_SUCCESS) {
        /* Did not start conversion process correctly. */
        while(1);
    }

    /*
     * Go to sleep in the foreground thread forever. The ADC hardware will
     * perform conversions and invoke the callback function when a buffer is
     * full.
     */
    while(1) {
        sleep(1000);
    }
}

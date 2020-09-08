/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/UART.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"

/***** Definitions for ADC Sampling *****/
#define ADCBUFFERSIZE    (500)
#define UARTBUFFERSIZE   (500)

uint16_t sampleBufferOne[ADCBUFFERSIZE];
uint16_t sampleBufferTwo[ADCBUFFERSIZE];
uint32_t microVoltBuffer[ADCBUFFERSIZE];
uint32_t buffersCompletedCounter = 0;
char uartTxBuffer[UARTBUFFERSIZE];

/***** Definitions for RF *****/
/* Packet TX/RX Configuration */
#define PAYLOAD_LENGTH      30
/* Set packet interval to 1000ms */
#define PACKET_INTERVAL     (uint32_t)(4000000*1.0f)
/* Set Receive timeout to 500ms */
#define RX_TIMEOUT          (uint32_t)(4000000*0.5f)
/* NOTE: Only two data entries supported at the moment */
#define NUM_DATA_ENTRIES    2
/* The Data Entries data field will contain:
 * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
 * Max 30 payload bytes
 * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */
#define NUM_APPENDED_BYTES  2

/* Log radio events in the callback */
//#define LOG_RADIO_EVENTS

/***** Prototypes *****/
static void echoCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion,
    void *completedADCBuffer, uint32_t completedChannel);
void uartCallback(UART_Handle handle, void *buf, size_t count);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;
UART_Handle uart; /* Driver handle shared between the task and the callback function */

/* Pin driver handle */
static PIN_Handle pinHandle;
static PIN_State pinState;

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is aligned to a 4 byte boundary
 * (requirement from the RF core)
 */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(rxDataEntryBuffer, 4)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)]
                                                  __attribute__((aligned(4)));
#else
#error This compiler is not supported
#endif //defined(__TI_COMPILER_VERSION__)

/* Receive Statistics */
static rfc_propRxOutput_t rxStatistics;

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;

static uint8_t txPacket[PAYLOAD_LENGTH];
static uint8_t rxPacket[PAYLOAD_LENGTH + NUM_APPENDED_BYTES - 1];
static uint16_t seqNumber;

static volatile bool bRxSuccess = false;

#ifdef LOG_RADIO_EVENTS
static volatile RF_EventMask eventLog[32];
static volatile uint8_t evIndex = 0;
#endif // LOG_RADIO_EVENTS

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
#if defined(Board_CC1350_LAUNCHXL)
 Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
 Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_DIO15 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_DIO24_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_DIO26_ANALOG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 PIN_TERMINATE
};

/***** Function definitions *****/

void *mainThread(void *arg0)
{
    /******************** Setup for PWM code to create 40kHz square-wave burst for 1ms ********************/

    /* Period and duty */
    uint16_t   pwmPeriod = 25; // in microseconds (40kHz)
    uint32_t   duty;

    /* Sleep time in microseconds */
    PWM_Handle pwm2 = NULL;
    PWM_Params params;

    /* Call driver init functions. */
    PWM_init();

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_FRACTION;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;

    // Board_PWM2 = CC2640R2_LAUNCHXL_PWM2 (in Board.h) which I assigned to CC2640R2_LAUNCHXL_DIO21 in CC2640R2_LAUNCHXL.h
    pwm2 = PWM_open(Board_PWM2, &params);
    if (pwm2 == NULL) {
        /* Board_PWM2 did not open */
        while (1);
    }

    PWM_start(pwm2);

    /***** Added ADC Sampling Params *****/
        UART_Params uartParams;
        ADCBuf_Handle adcBuf;
        ADCBuf_Params adcBufParams;
        ADCBuf_Conversion continuousConversion;

        /********** Added ADC Code **********/
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

        //        adcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_ONE_SHOT;
        //        adcBufParams.returnMode = ADCBuf_RETURN_MODE_BLOCKING;

            adcBufParams.samplingFrequency = 200000; // 200kHz
            adcBuf = ADCBuf_open(Board_ADCBUF0, &adcBufParams);

            /* Configure the conversion struct */
            continuousConversion.arg = NULL;
            continuousConversion.adcChannel = Board_ADCBUF0CHANNEL0;
            continuousConversion.sampleBuffer = sampleBufferOne;
            continuousConversion.sampleBufferTwo = sampleBufferTwo;
            continuousConversion.samplesRequestedCount = ADCBUFFERSIZE;

    /******************** Setup for rfTx code to send RF signal and later receive echo (board 1)********************/

    /* Open LED pins */
    pinHandle = PIN_open(&pinState, pinTable);
    if (pinHandle == NULL) {
        while(1);
    }

    uint32_t curtime;
    uint32_t Txtime;
    RF_Params rfParams;
    RF_Params_init(&rfParams);


    if(RFQueue_defineQueue(&dataQueue,
                           rxDataEntryBuffer,
                           sizeof(rxDataEntryBuffer),
                           NUM_DATA_ENTRIES,
                           PAYLOAD_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
        while(1);
    }

    /* Modify CMD_PROP_TX and CMD_PROP_RX commands for application needs */
    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = txPacket;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

    RF_cmdPropTx.pNextOp = (rfc_radioOp_t *)&RF_cmdPropRx;
    /* Only run the RX command if TX is successful */
    RF_cmdPropTx.condition.rule = COND_STOP_ON_FALSE;

    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = PAYLOAD_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 0;
    RF_cmdPropRx.pktConf.bRepeatNok = 0;
    RF_cmdPropRx.pOutput = (uint8_t *)&rxStatistics;
    /* Receive operation will end RX_TIMEOUT ms after command starts */
    RF_cmdPropRx.endTrigger.triggerType = TRIG_REL_PREVEND;
    RF_cmdPropRx.endTime = RX_TIMEOUT;

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);


    while(1)
    {

        _delay_cycles(47300000-1); // 1s delay between each cycle of US bursts

        /*********** Delay transmission of RF packet to be after US signal
         * because both cannot happen at same time ************/

        curtime = RF_getCurrentTime();
        Txtime = curtime + (uint32_t)(4000000*0.0001f); // (5ms but board cannot transmit RF and US at same time, so it sets RF to 1.75ms delay after US if Txtime is <=10ms)
        RF_cmdPropTx.startTime = Txtime; // delay RF packet transmission time so US square-wave emitted first

        /* Set PWM duty to 50% then back to 0 to generate a burst*/
        duty = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 50) / 100); // set duty cycle to 50%
        PWM_setDuty(pwm2, duty);
        _delay_cycles(47300-1); // 1ms delay to sustain burst
        PWM_setDuty(pwm2, 0); // set duty cycle to 0 (no signal)


        /******************** RF loop ********************/

        /* Create packet with incrementing sequence number and random payload */
        txPacket[0] = (uint8_t)(seqNumber >> 8);
        txPacket[1] = (uint8_t)(seqNumber++);
        uint8_t i;
        for (i = 2; i < PAYLOAD_LENGTH; i++)
        {
            txPacket[i] = rand();
        }


        /* Transmit a packet and wait for its echo.
         * - When the first of the two chained commands (TX) completes, the
         * RF_EventCmdDone event is raised but not RF_EventLastCmdDone
         * - The RF_EventLastCmdDone in addition to the RF_EventCmdDone events
         * are raised when the second, and therefore last, command (RX) in the
         * chain completes
         * -- If the RF core successfully receives the echo it will also raise
         * the RF_EventRxEntryDone event
         * -- If the RF core times out while waiting for the echo it does not
         * raise the RF_EventRxEntryDone event
         */
        RF_EventMask terminationReason =
                RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal,
                          echoCallback, (RF_EventCmdDone | RF_EventRxEntryDone |
                          RF_EventLastCmdDone));

        /********** Mapping RF signals to GPIO for debugging **********/
        // Map RFC_GPO0 to IO 24
        PINCC26XX_setMux(pinHandle, IOID_24, PINCC26XX_MUX_RFC_GPO0); // LNA radio signal (high in Rx mode)
        // Map IO 26 to RFC_GPI1
        PINCC26XX_setMux(pinHandle, IOID_26, PINCC26XX_MUX_RFC_GPO3); // transmission initiation radio signal (high when transmission initiated)


        switch(terminationReason)
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
                break;
            case RF_EventCmdCancelled:
                // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
                break;
            case RF_EventCmdAborted:
                // Abrupt command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            case RF_EventCmdStopped:
                // Graceful command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            default:
                // Uncaught error event
                while(1);
        }

        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
        switch(cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet transmitted successfully
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packet
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT while transmitting packet
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesizer being programmed
                break;
            case PROP_ERROR_TXUNF:
                // TX underflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while(1);
        }

        /****************** Added ADC code to occur after echoed packet received? *******************/

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

    }
}

static void echoCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
#ifdef LOG_RADIO_EVENTS
    eventLog[evIndex++ & 0x1F] = e;
#endif// LOG_RADIO_EVENTS

    if((e & RF_EventCmdDone) && !(e & RF_EventLastCmdDone))
    {
        /* Successful TX */
        /* Toggle LED1, clear LED2 to indicate TX */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,
                           !PIN_getOutputValue(Board_PIN_LED1));
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
    }
    else if(e & RF_EventRxEntryDone)
    {
        /* Successful RX */
        bRxSuccess = true;

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &(currentDataEntry->data):
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte
         */
        packetLength      = *(uint8_t *)(&(currentDataEntry->data));
        packetDataPointer = (uint8_t *)(&(currentDataEntry->data) + 1);

        /* Copy the payload + status byte to the rxPacket variable */
        memcpy(rxPacket, packetDataPointer, (packetLength + 1));

        /* Check the packet against what was transmitted */
        int16_t status = memcmp(txPacket, rxPacket, packetLength);

        if(status == 0)
        {
            /* Toggle LED1, clear LED2 to indicate RX */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1,
                               !PIN_getOutputValue(Board_PIN_LED1));
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        }
        else
        {
            /* Error Condition: set both LEDs */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
        }

        RFQueue_nextEntry();
    }
    else if((e & RF_EventLastCmdDone) && !(e & RF_EventRxEntryDone))
    {
        if(bRxSuccess == true)
        {
            /* Received packet successfully but RX command didn't complete at
             * the same time RX_ENTRY_DONE event was raised. Reset the flag
             */
            bRxSuccess = false;
        }
        else
        {
            /* RX timed out */
            /* Set LED2, clear LED1 to indicate TX */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
        }
    }
    else
    {
        /* Error Condition: set both LEDs */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
    }
}

/*
 * This function is called whenever an ADC buffer is full.
 * The content of the buffer is then converted into human-readable format and
 * sent to the PC via UART.
 */
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion,
    void *completedADCBuffer, uint32_t completedChannel) {

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
       uint32_t bin_average = 0;

       uint16_t run_number = 0;
       uint32_t run_max = 0;

       uint16_t bin_number = 0; // 0-39 bins, does not get reset
       uint16_t saved_bin_number = 0; // keep track of which bin the peak average occurs in
       uint32_t total_max = 0;

       // loop through 2.5 ms data four times to get 10 ms data
       while (run_number < 4) { // this loop does not actually do anything, it's just repeating the same data (from the microcontroller
        // output buffer) 4 times. Ignore this loop. We are actually only sampling for 2.5 ms
           // loop through the 10 bins
           while (a < (ADCBUFFERSIZE/50)) {
               // loop through the 50 microvoltbuffer values associated with each bin (b<50, 50<b<100, 100<b<150, etc.)
               while (b < (ADCBUFFERSIZE/10 + 50*a)) {
                   sum = sum + abs(microVoltBuffer[b]);
                   b++; // increment sample number (will do 50 samples for each bin)

                   // printf("%u\n", (unsigned int) microVoltBuffer[b]);
               }
               bin_average = sum / 500; // calculate average of the bin
               // to find max average value among the 10 bins
               if (bin_average > run_max) {
                   run_max = bin_average;
                   saved_bin_number = bin_number;
               }
               a++; // increment bin number that gets reset after each run
               bin_number++;

           }
           // reset values for the new run
           a = 0; // bin #
           b = 0; // sample #
           sum = 0;
           bin_average = 0;
           if (run_max > total_max) {
               total_max = run_max;
           }
           run_number++; // increment run # (does 4 runs for 10ms of data)
//               printf("%d %d\n", total_max, saved_bin_number);
       }


        // make pin go high if peak average value over the 10 ms sampling interval is greater than a threshold value
        // AND it occurs within 6ms of transmission
       if (total_max > 50000 && saved_bin_number <= 23) {
   //         digital pin 15 (buzzer) goes high
                PIN_setOutputValue(pinHandle, Board_DIO15, 1);
       }
       else {
          // digital pin 15 (buzzer) goes low
                PIN_setOutputValue(pinHandle, Board_DIO15, 0);
       }

   //    ADC_close(handle);

       ADCBuf_convertCancel(handle);

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

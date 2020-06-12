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
 *  ======== pwmled2.c ========
 */
/* For usleep() */
#include <unistd.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/PWM.h>

/* Example/Board Header files */
#include "Board.h"

/*
 *  ======== mainThread ========
 *  Task periodically increments the PWM duty for the on board LED.
 */
void *mainThread(void *arg0)
{
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

    /* Loop forever incrementing the PWM duty */
    while (1) {

        duty = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 50) / 100); // set duty cycle to 50%
        PWM_setDuty(pwm2, duty);
        _delay_cycles(47300-1); // 1ms delay to sustain burst
        PWM_setDuty(pwm2, 0); // set duty cycle to 0 (no signal)
        _delay_cycles(47300000-1); // 1000ms delay to sustain idle period
    }
}

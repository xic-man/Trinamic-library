/*
 * tmc2209hal.c - interface for Trinamic TMC2209 stepper driver
 *
 * v0.0.5 / 2021-11-22 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2021, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission..

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <stdlib.h>
#include <string.h>

#include "tmc2209.h"
#include "tmchal.h"

trinamic_config_t *getConfig(TMC2209_t *driver) {
    return &driver->config;
}

bool isValidMicrosteps(TMC2209_t *driver, uint16_t msteps) {
    return tmc_microsteps_validate(msteps);
}

void setMicrosteps(TMC2209_t *driver, uint16_t msteps) {
   TMC2209_SetMicrosteps(driver, (tmc2209_microsteps_t)msteps);
}

void setCurrent(TMC2209_t *driver, uint16_t mA, uint8_t hold_pct) {
    TMC2209_SetCurrent(driver, mA, hold_pct);
}

uint16_t getCurrent(TMC2209_t *driver) {
    return TMC2209_GetCurrent(driver);
}

TMC_chopconf_t getChopconf(TMC2209_t *driver) {
    TMC_chopconf_t chopconf;

    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->chopconf);

    chopconf.mres = driver->chopconf.reg.mres;
    chopconf.toff = driver->chopconf.reg.toff;
    chopconf.tbl = driver->chopconf.reg.tbl;
    chopconf.hend = driver->chopconf.reg.hend;
    chopconf.hstrt = driver->chopconf.reg.hstrt;

    return chopconf;
}

uint32_t getStallGuardResult(TMC2209_t *driver) {
    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->sg_result);

    return (uint32_t)driver->sg_result.reg.result;
}

TMC_drv_status_t getDriverStatus(TMC2209_t *driver) {
    TMC_drv_status_t drv_status = {0};
    TMC2209_status_t status;

    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->sg_result);
    status.value = TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->drv_status);

    drv_status.driver_error = status.driver_error;
    drv_status.sg_result = driver->sg_result.reg.result;
    drv_status.ot = driver->drv_status.reg.ot;
    drv_status.otpw = driver->drv_status.reg.otpw;
    drv_status.cs_actual = driver->drv_status.reg.cs_actual;
    drv_status.stst = driver->drv_status.reg.stst;
//    drv_status.fsactive = driver->drv_status.reg.fsactive;
    drv_status.ola = driver->drv_status.reg.ola;
    drv_status.olb = driver->drv_status.reg.olb;
    drv_status.s2ga = driver->drv_status.reg.s2ga;
    drv_status.s2gb = driver->drv_status.reg.s2gb;

    return drv_status;
}

TMC_ihold_irun_t getIholdIrun(TMC2209_t *driver) {
    TMC_ihold_irun_t ihold_irun;

    ihold_irun.ihold = driver->ihold_irun.reg.ihold;
    ihold_irun.irun = driver->ihold_irun.reg.irun;
    ihold_irun.iholddelay = driver->ihold_irun.reg.iholddelay;

    return ihold_irun;
}

uint32_t getDriverStatusRaw(TMC2209_t *driver) {
    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->drv_status);

    return driver->drv_status.reg.value;
}

uint32_t getTStep(TMC2209_t *driver) {
    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->tstep);

    return (uint32_t)driver->tstep.reg.tstep;
}

void setTCoolThrs(TMC2209_t *driver, float mm_sec, float steps_mm) {
    TMC2209_SetTCOOLTHRS(driver, mm_sec, steps_mm);
}

void setTCoolThrsRaw(TMC2209_t *driver, uint32_t value) {
    driver->tcoolthrs.reg.tcoolthrs = value;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tcoolthrs);
}

void stallGuardEnable(TMC2209_t *driver, float feed_rate, float steps_mm, int16_t sensitivity) {
    driver->gconf.reg.en_spreadcycle = false; // stealthChop on
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->gconf);

    driver->pwmconf.reg.pwm_autoscale = false;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->pwmconf);

    TMC2209_SetTCOOLTHRS(driver, feed_rate / (60.0f * 1.5f), steps_mm);

    driver->sgthrs.reg.threshold = (uint8_t)sensitivity;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->sgthrs);
}

void stealthChopEnable(TMC2209_t *driver) {
    driver->gconf.reg.en_spreadcycle = false; // stealthChop on
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->gconf);

    driver->pwmconf.reg.pwm_autoscale = true;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->pwmconf);

    setTCoolThrsRaw(driver, 0);
}

void coolStepEnable(TMC2209_t *driver) {
    driver->gconf.reg.en_spreadcycle = true; // stealthChop off
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->gconf);

    driver->pwmconf.reg.pwm_autoscale = false;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->pwmconf);

    setTCoolThrsRaw(driver, 0);
}

float getTPWMThrs(TMC2209_t *driver, float steps_mm) {
    return TMC2209_GetTPWMTHRS(driver, steps_mm);
}

uint32_t getTPWMThrsRaw(TMC2209_t *driver) {
    return driver->tpwmthrs.reg.tpwmthrs;
}

void setTPWMThrs(TMC2209_t *driver, float mm_sec, float steps_mm) {
    TMC2209_SetTPWMTHRS(driver, mm_sec, steps_mm);
}

void stealthChop(TMC2209_t *driver, bool on) {
    driver->config.mode = on ? TMCMode_StealthChop : TMCMode_CoolStep;

    if(on)
        stealthChopEnable(driver);
    else
        coolStepEnable(driver);
}

bool stealthChopGet(TMC2209_t *driver) {
    return !driver->gconf.reg.en_spreadcycle && driver->pwmconf.reg.pwm_autoscale;
}

// coolconf

void sg_filter(TMC2209_t *driver, bool val) {
//    driver->sgthrs.reg.threshold = val;
//    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->coolconf);
}

void sg_stall_value(TMC2209_t *driver, int16_t val) {
    driver->sgthrs.reg.threshold = (uint8_t)val;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->sgthrs);
}

int16_t get_sg_stall_value(TMC2209_t *driver) {
    return (int16_t)driver->sgthrs.reg.threshold;
}

void coolconf(TMC2209_t *driver, TMC_coolconf_t coolconf) {
    driver->coolconf.reg.semin = coolconf.semin;
    driver->coolconf.reg.semax = coolconf.semax;
    driver->coolconf.reg.sedn = coolconf.sedn;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->coolconf);
}

// chopconf

void chopper_timing(TMC2209_t *driver, TMC_chopper_timing_t timing) {
    driver->chopconf.reg.hstrt = timing.hstrt - 1;
    driver->chopconf.reg.hend = timing.hend + 3;
    driver->chopconf.reg.tbl = timing.tbl;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->chopconf);
}

uint8_t pwm_scale(TMC2209_t *driver) {
    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->pwm_scale);

    return driver->pwm_scale.reg.pwm_scale_sum;
}

bool vsense(TMC2209_t *driver) {
    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->chopconf);

    return driver->chopconf.reg.vsense;
}

void TMC2209_Create(TMC2209_t *driver, uint8_t address, uint16_t current, uint8_t microsteps, uint8_t r_sense) {
    TMC2209_SetDefaults(driver);
    driver->config.motor.id = address;
    driver->config.current = current;
    driver->config.microsteps = microsteps;
    driver->config.r_sense = r_sense;
    driver->chopconf.reg.mres = tmc_microsteps_to_mres(microsteps);
    TMC2209_Init(driver);
}

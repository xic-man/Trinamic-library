/*
 * tmc2209hal.h - interface for Trinamic TMC2209 stepper driver
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

#ifndef _TMC2209HAL_H_
#define _TMC2209HAL_H_

#include "tmchal.h"

trinamic_config_t *getConfig(TMC2209_t *driver);
bool isValidMicrosteps(TMC2209_t *driver, uint16_t msteps);
void setMicrosteps(TMC2209_t *driver, uint16_t msteps);
void setCurrent(TMC2209_t *driver, uint16_t mA, uint8_t hold_pct);
uint16_t getCurrent(TMC2209_t *driver);
TMC_chopconf_t getChopconf(TMC2209_t *driver);
uint32_t getStallGuardResult(TMC2209_t *driver);
TMC_drv_status_t getDriverStatus(TMC2209_t *driver);
TMC_ihold_irun_t getIholdIrun(TMC2209_t *driver);
uint32_t getDriverStatusRaw(TMC2209_t *driver);
uint32_t getTStep(TMC2209_t *driver);
void setTCoolThrs(TMC2209_t *driver, float mm_sec, float steps_mm);
void setTCoolThrsRaw(TMC2209_t *driver, uint32_t value);
void stallGuardEnable(TMC2209_t *driver, float feed_rate, float steps_mm, int16_t sensitivity);
void stealthChopEnable(TMC2209_t *driver);
void coolStepEnable(TMC2209_t *driver);
float getTPWMThrs(TMC2209_t *driver, float steps_mm);
uint32_t getTPWMThrsRaw(TMC2209_t *driver);
void setTPWMThrs(TMC2209_t *driver, float mm_sec, float steps_mm);
void stealthChop(TMC2209_t *driver, bool on);
bool stealthChopGet(TMC2209_t *driver);
void sg_filter(TMC2209_t *driver, bool val);
void sg_stall_value(TMC2209_t *driver, int16_t val);
int16_t get_sg_stall_value(TMC2209_t *driver);
void coolconf(TMC2209_t *driver, TMC_coolconf_t coolconf);
void chopper_timing(TMC2209_t *driver, TMC_chopper_timing_t timing);
uint8_t pwm_scale(TMC2209_t *driver);
bool vsense(TMC2209_t *driver);
void TMC2209_Create(TMC2209_t *driver, uint8_t address, uint16_t current, uint8_t microsteps, uint8_t r_sense);

#endif

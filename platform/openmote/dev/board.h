/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** \addtogroup openmote
 * @{
 *
 * \defgroup openmote Peripherals
 *
 * Defines related to the OpenMote
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the
 * OpenMote
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
#ifndef BOARD_H_
#define BOARD_H_

#include "dev/gpio.h"
#include "dev/nvic.h"
/*---------------------------------------------------------------------------*/
/** \name OpenMote LED configuration
 *
 * LEDs on the OpenMote are connected as follows:
 * - LED1 (Red)    -> PC4
 * - LED2 (Yellow) -> PC6
 * - LED3 (Green)  -> PC7
 * - LED4 (Orange) -> PC5
 *
 * @{
 */
/*---------------------------------------------------------------------------*/
/* Some files include leds.h before us, so we need to get rid of defaults in
 * leds.h before we provide correct definitions */
#undef LEDS_GREEN
#undef LEDS_YELLOW
#undef LEDS_RED
#undef LEDS_CONF_ALL

#define LEDS_RED       16 /**< LED1 (Red) -> PC4 */
#define LEDS_YELLOW    64 /**< LED2 (Yellow) -> PC6 */
#define LEDS_GREEN     128 /**< LED3 (Green)  -> PC7 */
#define LEDS_ORANGE    32 /**< LED4 (Orange) -> PC5 */
#define LEDS_CONF_ALL 240

/* Notify various examples that we have LEDs */
#define PLATFORM_HAS_LEDS        1
/** @} */
/*---------------------------------------------------------------------------*/
/** \name USB configuration
 *
 * The USB pullup is driven by PC0
 */
#define USB_PULLUP_PORT          GPIO_C_NUM
#define USB_PULLUP_PIN           0
/** @} */
/*---------------------------------------------------------------------------*/
/** \name UART configuration
 *
 * On the OpenMote, the UART is connected to the
 * following ports/pins
 * - RX:  PA0
 * - TX:  PA1
 * - CTS: PB0 (Can only be used with UART1)
 * - RTS: PD3 (Can only be used with UART1)
 *
 * We configure the port to use UART0. To use UART1, replace UART0_* with
 * UART1_* below.
 * @{
 */
#define UART0_RX_PORT            GPIO_A_NUM
#define UART0_RX_PIN             0

#define UART0_TX_PORT            GPIO_A_NUM
#define UART0_TX_PIN             1

#define UART1_CTS_PORT           GPIO_B_NUM
#define UART1_CTS_PIN            0

#define UART1_RTS_PORT           GPIO_D_NUM
#define UART1_RTS_PIN            3
/** @} */
/*---------------------------------------------------------------------------*/
/** \name OpenMote Button configuration
 *
 * Buttons on the OpenMote are connected as follows:
 * - BUTTON_USER -> PA3
 * @{
 */
/** BUTTON_USER -> PA3 */
#define BUTTON_USER_PORT       GPIO_A_NUM
#define BUTTON_USER_PIN        3
#define BUTTON_USER_VECTOR     NVIC_INT_GPIO_PORT_A
/* Notify various examples that we have Buttons */
#define PLATFORM_HAS_BUTTON      0
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SPI configuration
 *
 * These values configure which CC2538 pins to use for the SPI lines.
 * @{
 */
#define SPI_CLK_PORT             GPIO_A_NUM
#define SPI_CLK_PIN              2
#define SPI_MOSI_PORT            GPIO_A_NUM
#define SPI_MOSI_PIN             4
#define SPI_MISO_PORT            GPIO_A_NUM
#define SPI_MISO_PIN             5
#define SPI_SEL_PORT             GPIO_B_NUM
#define SPI_SEL_PIN              5
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "OpenMote-CC2538"
/** @} */

#endif /* BOARD_H_ */

/**
 * @}
 * @}
 */

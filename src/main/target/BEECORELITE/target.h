/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "SPEV"

#define TARGET_CONFIG

#define CONFIG_FASTLOOP_PREFERRED_ACC ACC_DEFAULT

#undef USE_DSHOT

// #define BRUSHED_ESC_AUTODETECT

#define NOMAIN

#define LED0                    PB8

#define USE_EXTI
#define MPU_INT_EXTI            PC13 // what?
#define EXTI15_10_CALLBACK_HANDLER_COUNT 2 // MPU_INT, SDCardDetect
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

// #define USE_ESC_SENSOR

#define GYRO
#define USE_GYRO_SPI_MPU6500

#define ACC
#define USE_ACC_SPI_MPU6500

#define ACC_MPU6500_ALIGN       CW180_DEG
#define GYRO_MPU6500_ALIGN      CW180_DEG

#define USE_VCP
#define USE_UART1
// #define USE_UART2
// #define USE_UART3

#define SERIAL_PORT_COUNT       2

// #define USE_ESCSERIAL
// #define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define UART2_TX_PIN            PA14 // PA14 / SWCLK
#define UART2_RX_PIN            PA15

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)

#define USE_SPI
#define USE_SPI_DEVICE_1 // PB9,3,4,5 on AF5 SPI1 (MPU)
// #define USE_SPI_DEVICE_2 // PB12,13,14,15 on AF5 SPI2 (SDCard)

#define SPI1_NSS_PIN            PB9
#define SPI1_SCK_PIN            PB3
#define SPI1_MISO_PIN           PB4
#define SPI1_MOSI_PIN           PB5

// #define SPI2_NSS_PIN            PB12
// #define SPI2_SCK_PIN            PB13
// #define SPI2_MISO_PIN           PB14
// #define SPI2_MOSI_PIN           PB15

#define MPU6500_CS_PIN                   PB9
#define MPU6500_SPI_INSTANCE             SPI1

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define ADC_INSTANCE            ADC2
#define RSSI_ADC_PIN            PB2
#define VBAT_ADC_PIN            PA4
#define CURRENT_METER_ADC_PIN   PA5

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM
#define DEFAULT_FEATURES        (FEATURE_RSSI_ADC | FEATURE_CURRENT_METER)

#define SPEKTRUM_BIND_PIN       UART3_RX_PIN

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// IO - stm32f303cc in 48pin package
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         (BIT(13)|BIT(14)|BIT(15))
#define TARGET_IO_PORTF         (BIT(0)|BIT(1)|BIT(4))

#define USABLE_TIMER_CHANNEL_COUNT 12 // PPM, 8 PWM, UART3 RX/TX, LED Strip
#define USED_TIMERS             (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8) | TIM_N(15) | TIM_N(16))
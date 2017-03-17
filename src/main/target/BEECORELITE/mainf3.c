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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

// #include "common/axis.h"
// #include "common/color.h"
// #include "common/maths.h"
#include "common/printf.h"

#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/feature.h"
// #include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

// #include "drivers/nvic.h"
// #include "drivers/sensor.h"
#include "drivers/system.h"
// #include "drivers/dma.h"
#include "drivers/io.h"
// #include "drivers/light_led.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
// #include "drivers/accgyro.h"
// #include "drivers/pwm_esc_detect.h"
// #include "drivers/rx_pwm.h"
// #include "drivers/pwm_output.h"
// #include "drivers/adc.h"
// #include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
// #include "drivers/inverter.h"
// #include "drivers/usb_io.h"
// #include "drivers/exti.h"
#include "drivers/serial_usb_vcp.h"

// #include "fc/config.h"
// #include "fc/fc_init.h"
// #include "fc/fc_msp.h"
// #include "fc/fc_tasks.h"
// #include "fc/rc_controls.h"
// #include "fc/runtime_config.h"
// #include "fc/cli.h"

// #include "msp/msp_serial.h"

// #include "rx/rx.h"
// #include "rx/spektrum.h"

// #include "io/serial.h"
// #include "io/motors.h"

// #include "scheduler/scheduler.h"

// #include "sensors/acceleration.h"
// #include "sensors/battery.h"
// #include "sensors/boardalignment.h"
// #include "sensors/gyro.h"
#include "sensors/initialisation.h"
// #include "sensors/sensors.h"

// #include "flight/failsafe.h"
// #include "flight/imu.h"
// #include "flight/mixer.h"
// #include "flight/pid.h"

// #include "build/build_config.h"
// #include "build/debug.h"

static serialPort_t *serial0 = NULL;
static serialPort_t *cereal = NULL;

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

void LEDInit() {
    gpio_config_t cfg;

    cfg.pin = GPIO_Pin_8;
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_50MHz;

    gpioInit(GPIOB, &cfg);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

void setup() {

    // common/printf.h
    printfSupportInit();

    // drivers/system.h (system_stm32f30x.c)
    systemInit();

    // drivers/io.h
    // initialize IO (needed for all IO operations)
    IOInitGlobal();

    // initEEPROM();

    // ensureEEPROMContainsValidData();
    // readEEPROM();

    ledInit(statusLedConfig());

    EXTIInit();

    delay(100);

    timerInit();  // timer must be initialized before any channel is allocated

    // this initializes some structure that helps manage USARTs
    serialInit(feature(FEATURE_SOFTSERIAL), SERIAL_PORT_NONE);

    // For MPU6500
    spiInit(SPIDEV_1);

    // Custom serial printer
    // cerealInit();

    // led GPIOB_Pin_8
    LEDInit();

    // open the USB VCP
    serial0 = usbVcpOpen();

    // Open USART1
    cereal = uartOpen(USART1, NULL, 115200, (MODE_TX|MODE_RX), SERIAL_NOT_INVERTED);
    setPrintfSerialPort(cereal);
    printf("printf Configured!\n");

    sensorsAutodetect();

}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

void loop() {
    static uint16_t i = 0;

    // print
    accUpdate(&accelerometerConfigMutable()->accelerometerTrims);

    printf("Current Time: %d\n", ++i);
    // printf("\t acc.smooth[0]: %d", acc.accSmooth[0]);
    // printf("\t acc.dev.ADCRaw[0]: %d\n", acc.dev.ADCRaw[0]);

    digitalHi(GPIOB, GPIO_Pin_8);   // turn the LED on (HIGH is the voltage level)
    delay(200);               // wait for a second
    digitalLo(GPIOB, GPIO_Pin_8);    // turn the LED off by making the voltage LOW
    delay(200);                // wait for a second
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

int main(void) {
    setup();
    while(1) {

        // allow reboot to happen for flashing
        while(serialRxBytesWaiting(serial0)) {
            const uint8_t c = serialRead(serial0);

            if (c == 'R') {
                systemResetToBootloader();
            }
        }

        loop();
    }
}
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
#include "drivers/accgyro.h"
#include "drivers/accgyro_mpu.h"
#include "drivers/accgyro_mpu6500.h"
#include "drivers/accgyro_spi_mpu6500.h"
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
// #include "sensors/initialisation.h"
// #include "sensors/sensors.h"

// #include "flight/failsafe.h"
// #include "flight/imu.h"
// #include "flight/mixer.h"
// #include "flight/pid.h"

// #include "build/build_config.h"
// #include "build/debug.h"

static serialPort_t *serial0 = NULL;
static serialPort_t *cereal = NULL;

static accDev_t accDev;
static gyroDev_t gyroDev;
static bool accInited = false;

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

void LEDInit() {
    gpio_config_t cfg;

    cfg.pin = GPIO_Pin_8;
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_50MHz;

    gpioInit(GPIOB, &cfg);
}

static const extiConfig_t *selectMPUIntExtiConfig(void)
{
    static const extiConfig_t mpuIntExtiConfig = { .tag = IO_TAG(MPU_INT_EXTI) };
    return &mpuIntExtiConfig;
}

bool myGyroDetect(gyroDev_t *dev) {
    // gyroSensor_e gyroHardware = GYRO_DEFAULT;

    dev->gyroAlign = ALIGN_DEFAULT;



    if (mpu6500SpiGyroDetect(dev)) {
        // gyroHardware = GYRO_MPU6500;

        dev->gyroAlign = GYRO_MPU6500_ALIGN;
        printf("Gyro SPI Found!\n");
        return true;
    }



    // if (gyroHardware != GYRO_NONE) {
    //     detectedSensors[SENSOR_INDEX_GYRO] = gyroHardware;
    //     sensorsSet(SENSOR_GYRO);
    // }


    return false;
}

bool myGyroInit(void) {
    memset(&gyroDev, 0, sizeof(gyroDev));

    gyroDev.mpuIntExtiConfig = selectMPUIntExtiConfig();
    mpuDetect(&gyroDev);
    // mpuResetFn = gyroDev.mpuConfiguration.resetFn;


    // const gyroSensor_e gyroHardware = gyroDetect(&gyroDev);
    // if (gyroHardware == GYRO_NONE) {
    //     return false;
    // }

    if (!myGyroDetect(&gyroDev))
        return false;

    // Must set gyro sample rate before initialisation
    // gyro.targetLooptime = gyroSetSampleRate(&gyroDev, gyroConfig()->gyro_lpf, gyroConfig()->gyro_sync_denom, gyroConfig()->gyro_use_32khz);
    // gyroDev.lpf = gyroConfig()->gyro_lpf;
    gyroDev.init(&gyroDev);
    if (gyroConfig()->gyro_align != ALIGN_DEFAULT) {
        gyroDev.gyroAlign = gyroConfig()->gyro_align;
    }
    // gyroInitFilters();

    return true;
}

// ============================================================================


bool myAccDetect(accDev_t *dev) {

    dev->accAlign = ALIGN_DEFAULT;

    if (mpu6500SpiAccDetect(dev)) {
        dev->accAlign = ACC_MPU6500_ALIGN;
        printf("SPI found\n");
        return true;
    }

    return false;

    // // Found anything? Check if error or ACC is really missing.
    // if (accHardware == ACC_NONE && accHardwareToUse != ACC_DEFAULT && accHardwareToUse != ACC_NONE) {
    //     // Nothing was found and we have a forced sensor that isn't present.
    //     accHardwareToUse = ACC_DEFAULT;
    //     goto retry;
    // }


    // if (accHardware == ACC_NONE) {
    //     return false;
    // }

    // detectedSensors[SENSOR_INDEX_ACC] = accHardware;
    // sensorsSet(SENSOR_ACC);
    // return true;
}

bool myAccInit() {
    memset(&accDev, 0, sizeof(accDev));

    // copy over the common gyro mpu settings
    accDev.mpuConfiguration = gyroDev.mpuConfiguration;
    accDev.mpuDetectionResult = gyroDev.mpuDetectionResult;

    if (!myAccDetect(&accDev)) {
        return false;
    }
    accDev.acc_1G = 256; // set default
    accDev.init(&accDev); // driver initialisation

    if (accelerometerConfig()->acc_align != ALIGN_DEFAULT) {
        accDev.accAlign = accelerometerConfig()->acc_align;
    }

    return true;
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

    // sensorsAutodetect();

    // -----------------------

    myGyroInit();
    accInited = myAccInit();

}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

void loop() {
    static uint16_t i = 0;

    // print
    // accUpdate(&accelerometerConfigMutable()->accelerometerTrims);

    gyroDev.read(&gyroDev);
    if (accInited)
        accDev.read(&accDev);

    printf("Current Time: %d", ++i);
    // printf("\t acc.smooth[0]: %d", acc.accSmooth[0]);
    printf("\t acc.dev.ADCRaw[0]: %d", accDev.ADCRaw[0]);
    printf("\t gyro.gyroADCRaw[0]: %d\n", gyroDev.gyroADCRaw[0]);

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
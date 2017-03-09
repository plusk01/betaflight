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

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"

#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/rx_pwm.h"
#include "drivers/pwm_output.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/usb_io.h"
#include "drivers/exti.h"

#ifdef USE_BST
#include "bus_bst.h"
#endif

#include "fc/config.h"
#include "fc/fc_init.h"
#include "fc/fc_msp.h"
#include "fc/fc_tasks.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/cli.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/serial.h"
#include "io/motors.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/initialisation.h"
#include "sensors/sensors.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "build/build_config.h"
#include "build/debug.h"

#ifdef TARGET_PREINIT
void targetPreInit(void);
#endif

#ifdef TARGET_BUS_INIT
void targetBusInit(void);
#endif

extern uint8_t motorControlEnable;

#ifdef SOFTSERIAL_LOOPBACK
serialPort_t *loopbackPort;
#endif

uint8_t systemState = SYSTEM_STATE_INITIALISING;

void processLoopback(void)
{
#ifdef SOFTSERIAL_LOOPBACK
    if (loopbackPort) {
        uint8_t bytesWaiting;
        while ((bytesWaiting = serialRxBytesWaiting(loopbackPort))) {
            uint8_t b = serialRead(loopbackPort);
            serialWrite(loopbackPort, b);
        };
    }
#endif
}

void init(void)
{
#ifdef USE_HAL_DRIVER
    HAL_Init();
#endif

    printfSupportInit();

    systemInit();

    // initialize IO (needed for all IO operations)
    IOInitGlobal();

#ifdef USE_HARDWARE_REVISION_DETECTION
    detectHardwareRevision();
#endif

#ifdef BRUSHED_ESC_AUTODETECT
    detectBrushedESC();
#endif

    initEEPROM();

    ensureEEPROMContainsValidData();
    readEEPROM();

    systemState |= SYSTEM_STATE_CONFIG_LOADED;

    //i2cSetOverclock(masterConfig.i2c_overclock);

    debugMode = systemConfig()->debug_mode;

    // Latch active features to be used for feature() in the remainder of init().
    latchActiveFeatures();

#ifdef TARGET_PREINIT
    targetPreInit();
#endif

    ledInit(statusLedConfig());
    LED2_ON;

#ifdef USE_EXTI
    EXTIInit();
#endif

#if defined(BUTTONS)
#ifdef BUTTON_A_PIN
    IO_t buttonAPin = IOGetByTag(IO_TAG(BUTTON_A_PIN));
    IOInit(buttonAPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(buttonAPin, IOCFG_IPU);
#endif

#ifdef BUTTON_B_PIN
    IO_t buttonBPin = IOGetByTag(IO_TAG(BUTTON_B_PIN));
    IOInit(buttonBPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(buttonBPin, IOCFG_IPU);
#endif

    // Check status of bind plug and exit if not active
    delayMicroseconds(10);  // allow configuration to settle

    if (!isMPUSoftReset()) {
#if defined(BUTTON_A_PIN) && defined(BUTTON_B_PIN)
        // two buttons required
        uint8_t secondsRemaining = 5;
        bool bothButtonsHeld;
        do {
            bothButtonsHeld = !IORead(buttonAPin) && !IORead(buttonBPin);
            if (bothButtonsHeld) {
                if (--secondsRemaining == 0) {
                    resetEEPROM();
                    systemReset();
                }
                delay(1000);
                LED0_TOGGLE;
            }
        } while (bothButtonsHeld);
#endif
    }
#endif

#ifdef SPEKTRUM_BIND
    if (feature(FEATURE_RX_SERIAL)) {
        switch (rxConfig()->serialrx_provider) {
        case SERIALRX_SPEKTRUM1024:
        case SERIALRX_SPEKTRUM2048:
            // Spektrum satellite binding if enabled on startup.
            // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
            // The rest of Spektrum initialization will happen later - via spektrumInit()
            spektrumBind(rxConfigMutable());
            break;
        }
    }
#endif

    delay(100);

    timerInit();  // timer must be initialized before any channel is allocated

#if defined(AVOID_UART1_FOR_PWM_PPM)
    serialInit(feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART1 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART2_FOR_PWM_PPM)
    serialInit(feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART2 : SERIAL_PORT_NONE);
#elif defined(AVOID_UART3_FOR_PWM_PPM)
    serialInit(feature(FEATURE_SOFTSERIAL),
            feature(FEATURE_RX_PPM) || feature(FEATURE_RX_PARALLEL_PWM) ? SERIAL_PORT_USART3 : SERIAL_PORT_NONE);
#else
    serialInit(feature(FEATURE_SOFTSERIAL), SERIAL_PORT_NONE);
#endif

    mixerInit(mixerConfig()->mixerMode);

    uint16_t idlePulse = motorConfig()->mincommand;
    if (feature(FEATURE_3D)) {
        idlePulse = flight3DConfig()->neutral3d;
    }

    if (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_BRUSHED) {
        featureClear(FEATURE_3D);
        idlePulse = 0; // brushed motors
    }

    mixerConfigureOutput();
    motorDevInit(&motorConfig()->dev, idlePulse, getMotorCount());

#if defined(USE_PWM) || defined(USE_PPM)
    if (feature(FEATURE_RX_PPM)) {
        ppmRxInit(ppmConfig(), motorConfig()->dev.motorPwmProtocol);
    } else if (feature(FEATURE_RX_PARALLEL_PWM)) {
        pwmRxInit(pwmConfig());
    }
#endif

    systemState |= SYSTEM_STATE_MOTORS_READY;

/* temp until PGs are implemented. */
#ifdef USE_INVERTER
    initInverters();
#endif

#ifdef USE_BST
    bstInit(BST_DEVICE);
#endif

#ifdef TARGET_BUS_INIT
    targetBusInit();
#else

#ifdef USE_SPI
#ifdef USE_SPI_DEVICE_1
    spiInit(SPIDEV_1);
#endif
#ifdef USE_SPI_DEVICE_2
    spiInit(SPIDEV_2);
#endif
#ifdef USE_SPI_DEVICE_3
    spiInit(SPIDEV_3);
#endif
#ifdef USE_SPI_DEVICE_4
    spiInit(SPIDEV_4);
#endif
#endif /* USE_SPI */

#ifdef USE_I2C
#ifdef USE_I2C_DEVICE_1
    i2cInit(I2CDEV_1);
#endif
#ifdef USE_I2C_DEVICE_2
    i2cInit(I2CDEV_2);
#endif
#ifdef USE_I2C_DEVICE_3
    i2cInit(I2CDEV_3);
#endif  
#ifdef USE_I2C_DEVICE_4
    i2cInit(I2CDEV_4);
#endif  
#endif /* USE_I2C */

#endif /* TARGET_BUS_INIT */

#ifdef USE_HARDWARE_REVISION_DETECTION
    updateHardwareRevision();
#endif

#ifdef USE_ADC
    /* these can be removed from features! */
    adcConfigMutable()->vbat.enabled = feature(FEATURE_VBAT);
    adcConfigMutable()->currentMeter.enabled = feature(FEATURE_CURRENT_METER);
    adcConfigMutable()->rssi.enabled = feature(FEATURE_RSSI_ADC);
    adcInit(adcConfig());
#endif

    initBoardAlignment(boardAlignment());

    if (!sensorsAutodetect()) {
        // if gyro was not detected due to whatever reason, we give up now.
        failureMode(FAILURE_MISSING_ACC);
    }

    systemState |= SYSTEM_STATE_SENSORS_READY;

    LED1_ON;
    LED0_OFF;
    LED2_OFF;

    for (int i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(50);

    }
    LED0_OFF;
    LED1_OFF;

    // gyro.targetLooptime set in sensorsAutodetect(), so we are ready to call pidInit()
    pidInit();

    imuInit();

    mspFcInit();
    mspSerialInit();

#ifdef USE_CLI
    cliInit(serialConfig());
#endif

    failsafeInit();

    rxInit();

#ifdef USE_ESC_SENSOR
    if (feature(FEATURE_ESC_SENSOR)) {
        escSensorInit();
    }
#endif

#ifdef USB_DETECT_PIN
    usbCableDetectInit();
#endif

    if (mixerConfig()->mixerMode == MIXER_GIMBAL) {
        accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
    }
    gyroSetCalibrationCycles();

    // start all timers
    // TODO - not implemented yet
    timerStart();

    ENABLE_STATE(SMALL_ANGLE);
    DISABLE_ARMING_FLAG(PREVENT_ARMING);

#ifdef SOFTSERIAL_LOOPBACK
    // FIXME this is a hack, perhaps add a FUNCTION_LOOPBACK to support it properly
    loopbackPort = (serialPort_t*)&(softSerialPorts[0]);
    if (!loopbackPort->vTable) {
        loopbackPort = openSoftSerial(0, NULL, 19200, SERIAL_NOT_INVERTED);
    }
    serialPrint(loopbackPort, "LOOPBACK\r\n");
#endif

    // Now that everything has powered up the voltage and cell count be determined.

    if (feature(FEATURE_VBAT | FEATURE_CURRENT_METER))
        batteryInit();

#ifdef CJMCU
    LED2_ON;
#endif

    // Latch active features AGAIN since some may be modified by init().
    latchActiveFeatures();
    motorControlEnable = true;

    fcTasksInit();
    systemState |= SYSTEM_STATE_READY;
}

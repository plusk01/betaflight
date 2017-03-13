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
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "config/config_master.h"
#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/accgyro.h"
#include "drivers/bus_i2c.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/system.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/fc_msp.h"
#include "fc/fc_rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/motors.h"
#include "io/serial.h"
#include "io/serial_4way.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "rx/msp.h"
#include "rx/rx.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

extern uint16_t cycleTime; // FIXME dependency on mw.c

static const char * const flightControllerIdentifier = BETAFLIGHT_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.
static const char * const boardIdentifier = TARGET_BOARD_IDENTIFIER;

typedef struct box_e {
    const uint8_t boxId;            // see boxId_e
    const char *boxName;            // GUI-readable box name
    const uint8_t permanentId;      //
} box_t;

// FIXME remove ;'s
static const box_t boxes[CHECKBOX_ITEM_COUNT + 1] = {
    { BOXARM, "ARM;", 0 },
    { BOXANGLE, "ANGLE;", 1 },
    { BOXHORIZON, "HORIZON;", 2 },
    { BOXBARO, "BARO;", 3 },
    //{ BOXVARIO, "VARIO;", 4 },
    { BOXMAG, "MAG;", 5 },
    { BOXHEADFREE, "HEADFREE;", 6 },
    { BOXHEADADJ, "HEADADJ;", 7 },
    { BOXCAMSTAB, "CAMSTAB;", 8 },
    { BOXCAMTRIG, "CAMTRIG;", 9 },
    { BOXGPSHOME, "GPS HOME;", 10 },
    { BOXGPSHOLD, "GPS HOLD;", 11 },
    { BOXPASSTHRU, "PASSTHRU;", 12 },
    { BOXBEEPERON, "BEEPER;", 13 },
    { BOXLEDMAX, "LEDMAX;", 14 },
    { BOXLEDLOW, "LEDLOW;", 15 },
    { BOXLLIGHTS, "LLIGHTS;", 16 },
    { BOXCALIB, "CALIB;", 17 },
    { BOXGOV, "GOVERNOR;", 18 },
    { BOXOSD, "OSD SW;", 19 },
    { BOXTELEMETRY, "TELEMETRY;", 20 },
    { BOXGTUNE, "GTUNE;", 21 },
    { BOXSONAR, "SONAR;", 22 },
    { BOXSERVO1, "SERVO1;", 23 },
    { BOXSERVO2, "SERVO2;", 24 },
    { BOXSERVO3, "SERVO3;", 25 },
    { BOXBLACKBOX, "BLACKBOX;", 26 },
    { BOXFAILSAFE, "FAILSAFE;", 27 },
    { BOXAIRMODE, "AIR MODE;", 28 },
    { BOX3DDISABLESWITCH, "DISABLE 3D SWITCH;", 29},
    { BOXFPVANGLEMIX, "FPV ANGLE MIX;", 30},
    { BOXBLACKBOXERASE, "BLACKBOX ERASE (>30s);", 31 },
    { CHECKBOX_ITEM_COUNT, NULL, 0xFF }
};

// this is calculated at startup based on enabled features.
static uint8_t activeBoxIds[CHECKBOX_ITEM_COUNT];
// this is the number of filled indexes in above array
static uint8_t activeBoxIdCount = 0;

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

typedef enum {
    MSP_SDCARD_STATE_NOT_PRESENT = 0,
    MSP_SDCARD_STATE_FATAL       = 1,
    MSP_SDCARD_STATE_CARD_INIT   = 2,
    MSP_SDCARD_STATE_FS_INIT     = 3,
    MSP_SDCARD_STATE_READY       = 4
} mspSDCardState_e;

typedef enum {
    MSP_SDCARD_FLAG_SUPPORTTED   = 1
} mspSDCardFlags_e;

#define RATEPROFILE_MASK (1 << 7)

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#define ESC_4WAY 0xff

uint8_t escMode;
uint8_t escPortIndex = 0;

static void mspFc4waySerialCommand(sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    const unsigned int dataSize = sbufBytesRemaining(src);
    if (dataSize == 0) {
        // Legacy format

        escMode = ESC_4WAY;
    } else {
        escMode = sbufReadU8(src);
        escPortIndex = sbufReadU8(src);
    }

    switch(escMode) {
    case ESC_4WAY:
        // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
        sbufWriteU8(dst, esc4wayInit());

        if (mspPostProcessFn) {
            *mspPostProcessFn = esc4wayProcess;
        }

        break;
    default:
        sbufWriteU8(dst, 0);
    }
}
#endif

static void mspRebootFn(serialPort_t *serialPort)
{
    UNUSED(serialPort);

    stopPwmAllMotors();
    systemReset();

    // control should never return here.
    while (true) ;
}

static const box_t *findBoxByActiveBoxId(uint8_t activeBoxId)
{
    for (uint8_t boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        const box_t *candidate = &boxes[boxIndex];
        if (candidate->boxId == activeBoxId) {
            return candidate;
        }
    }
    return NULL;
}

static const box_t *findBoxByPermenantId(uint8_t permenantId)
{
    for (uint8_t boxIndex = 0; boxIndex < sizeof(boxes) / sizeof(box_t); boxIndex++) {
        const box_t *candidate = &boxes[boxIndex];
        if (candidate->permanentId == permenantId) {
            return candidate;
        }
    }
    return NULL;
}

static void serializeBoxNamesReply(sbuf_t *dst)
{
    int flag = 1, count = 0;

reset:
    // in first run of the loop, we grab total size of junk to be sent
    // then come back and actually send it
    for (int i = 0; i < activeBoxIdCount; i++) {
        const int activeBoxId = activeBoxIds[i];
        const box_t *box = findBoxByActiveBoxId(activeBoxId);
        if (!box) {
            continue;
        }

        const int len = strlen(box->boxName);
        if (flag) {
            count += len;
        } else {
            for (int j = 0; j < len; j++) {
                sbufWriteU8(dst, box->boxName[j]);
            }
        }
    }

    if (flag) {
        flag = 0;
        goto reset;
    }
}

void initActiveBoxIds(void)
{
    // calculate used boxes based on features and fill availableBoxes[] array
    memset(activeBoxIds, 0xFF, sizeof(activeBoxIds));

    activeBoxIdCount = 0;
    activeBoxIds[activeBoxIdCount++] = BOXARM;

    if (!feature(FEATURE_AIRMODE)) {
        activeBoxIds[activeBoxIdCount++] = BOXAIRMODE;
    }

    if (sensors(SENSOR_ACC)) {
        activeBoxIds[activeBoxIdCount++] = BOXANGLE;
        activeBoxIds[activeBoxIdCount++] = BOXHORIZON;
        activeBoxIds[activeBoxIdCount++] = BOXHEADFREE;
    }

    if (feature(FEATURE_FAILSAFE)) {
        activeBoxIds[activeBoxIdCount++] = BOXFAILSAFE;
    }

    if (mixerConfig()->mixerMode == MIXER_FLYING_WING || mixerConfig()->mixerMode == MIXER_AIRPLANE) {
        activeBoxIds[activeBoxIdCount++] = BOXPASSTHRU;
    }

    activeBoxIds[activeBoxIdCount++] = BOXBEEPERON;

    activeBoxIds[activeBoxIdCount++] = BOXFPVANGLEMIX;

    if (feature(FEATURE_3D)) {
        activeBoxIds[activeBoxIdCount++] = BOX3DDISABLESWITCH;
    }

    if (feature(FEATURE_SERVO_TILT)) {
        activeBoxIds[activeBoxIdCount++] = BOXCAMSTAB;
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
        activeBoxIds[activeBoxIdCount++] = BOXCALIB;
    }

    activeBoxIds[activeBoxIdCount++] = BOXOSD;

}

#define IS_ENABLED(mask) (mask == 0 ? 0 : 1)

static uint32_t packFlightModeFlags(void)
{
    // Serialize the flags in the order we delivered them, ignoring BOXNAMES and BOXINDEXES
    // Requires new Multiwii protocol version to fix
    // It would be preferable to setting the enabled bits based on BOXINDEX.
    const uint32_t tmp = IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)) << BOXANGLE |
        IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)) << BOXHORIZON |
        IS_ENABLED(FLIGHT_MODE(BARO_MODE)) << BOXBARO |
        IS_ENABLED(FLIGHT_MODE(MAG_MODE)) << BOXMAG |
        IS_ENABLED(FLIGHT_MODE(HEADFREE_MODE)) << BOXHEADFREE |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXHEADADJ)) << BOXHEADADJ |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMSTAB)) << BOXCAMSTAB |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCAMTRIG)) << BOXCAMTRIG |
        IS_ENABLED(FLIGHT_MODE(GPS_HOME_MODE)) << BOXGPSHOME |
        IS_ENABLED(FLIGHT_MODE(GPS_HOLD_MODE)) << BOXGPSHOLD |
        IS_ENABLED(FLIGHT_MODE(PASSTHRU_MODE)) << BOXPASSTHRU |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBEEPERON)) << BOXBEEPERON |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDMAX)) << BOXLEDMAX |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLEDLOW)) << BOXLEDLOW |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXLLIGHTS)) << BOXLLIGHTS |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXCALIB)) << BOXCALIB |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGOV)) << BOXGOV |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXOSD)) << BOXOSD |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXTELEMETRY)) << BOXTELEMETRY |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXGTUNE)) << BOXGTUNE |
        IS_ENABLED(FLIGHT_MODE(SONAR_MODE)) << BOXSONAR |
        IS_ENABLED(ARMING_FLAG(ARMED)) << BOXARM |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBLACKBOX)) << BOXBLACKBOX |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE)) << BOXBLACKBOXERASE |
        IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)) << BOXFAILSAFE |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAIRMODE)) << BOXAIRMODE |
        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX)) << BOXFPVANGLEMIX;

    uint32_t ret = 0;
    for (int i = 0; i < activeBoxIdCount; i++) {
        const uint32_t flag = (tmp & (1 << activeBoxIds[i]));
        if (flag) {
            ret |= 1 << i;
        }
    }
    return ret;
}

static void serializeSDCardSummaryReply(sbuf_t *dst)
{
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU8(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
}

static void serializeDataflashSummaryReply(sbuf_t *dst)
{
    sbufWriteU8(dst, 0); // FlashFS is neither ready nor supported
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
    sbufWriteU32(dst, 0);
}

/*
 * Returns true if the command was processd, false otherwise.
 * May set mspPostProcessFunc to a function to be called once the command has been processed
 */
static bool mspFcProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
    switch (cmdMSP) {
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;

    case MSP_FC_VARIANT:
        sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;

    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO:
        sbufWriteData(dst, boardIdentifier, BOARD_IDENTIFIER_LENGTH);
#ifdef USE_HARDWARE_REVISION_DETECTION
        sbufWriteU16(dst, hardwareRevision);
#else
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection.
#endif
        break;

    case MSP_BUILD_INFO:
        sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
        sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
        sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
        break;

    // DEPRECATED - Use MSP_API_VERSION
    case MSP_IDENT:
        sbufWriteU8(dst, MW_VERSION);
        sbufWriteU8(dst, mixerConfig()->mixerMode);
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU32(dst, CAP_DYNBALANCE); // "capability"
        break;

    case MSP_STATUS_EX:
        sbufWriteU16(dst, getTaskDeltaTime(TASK_GYROPID));
#ifdef USE_I2C
        sbufWriteU16(dst, i2cGetErrorCounter());
#else
        sbufWriteU16(dst, 0);
#endif
        sbufWriteU16(dst, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
        sbufWriteU32(dst, packFlightModeFlags());
        sbufWriteU8(dst, getCurrentPidProfileIndex());
        sbufWriteU16(dst, constrain(averageSystemLoadPercent, 0, 100));
        sbufWriteU8(dst, MAX_PROFILE_COUNT);
        sbufWriteU8(dst, getCurrentControlRateProfileIndex());
        break;

    case MSP_NAME:
        {
            const int nameLen = strlen(systemConfig()->name);
            for (int i = 0; i < nameLen; i++) {
                sbufWriteU8(dst, systemConfig()->name[i]);
            }
        }
        break;

    case MSP_STATUS:
        sbufWriteU16(dst, getTaskDeltaTime(TASK_GYROPID));
#ifdef USE_I2C
        sbufWriteU16(dst, i2cGetErrorCounter());
#else
        sbufWriteU16(dst, 0);
#endif
        sbufWriteU16(dst, sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
        sbufWriteU32(dst, packFlightModeFlags());
        sbufWriteU8(dst, getCurrentPidProfileIndex());
        sbufWriteU16(dst, constrain(averageSystemLoadPercent, 0, 100));
        sbufWriteU16(dst, 0); // gyro cycle time
        break;

    case MSP_RAW_IMU:
        {
            // Hack scale due to choice of units for sensor data in multiwii
            const uint8_t scale = (acc.dev.acc_1G > 512) ? 4 : 1;
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, acc.accSmooth[i] / scale);
            }
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, gyroRateDps(i));
            }
            // for (int i = 0; i < 3; i++) {
            //     sbufWriteU16(dst, mag.magADC[i]);
            // }
        }
        break;

    case MSP_MOTOR:
        for (unsigned i = 0; i < 8; i++) {
            if (i >= MAX_SUPPORTED_MOTORS || !pwmGetMotors()[i].enabled) {
                sbufWriteU16(dst, 0);
                continue;
            }

            sbufWriteU16(dst, convertMotorToExternal(motor[i]));
        }
        break;

    case MSP_RC:
        for (int i = 0; i < rxRuntimeConfig.channelCount; i++) {
            sbufWriteU16(dst, rcData[i]);
        }
        break;

    case MSP_ATTITUDE:
        sbufWriteU16(dst, attitude.values.roll);
        sbufWriteU16(dst, attitude.values.pitch);
        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;

    case MSP_ALTITUDE:
        sbufWriteU32(dst, 0);
        sbufWriteU16(dst, vario);
        break;

    case MSP_SONAR_ALTITUDE:
        sbufWriteU32(dst, 0);
        break;

    case MSP_ANALOG:
        sbufWriteU8(dst, (uint8_t)constrain(getVbat(), 0, 255));
        sbufWriteU16(dst, (uint16_t)constrain(mAhDrawn, 0, 0xFFFF)); // milliamp hours drawn from battery
        sbufWriteU16(dst, rssi);
        if(batteryConfig()->multiwiiCurrentMeterOutput) {
            sbufWriteU16(dst, (uint16_t)constrain(amperage * 10, 0, 0xFFFF)); // send amperage in 0.001 A steps. Negative range is truncated to zero
        } else
            sbufWriteU16(dst, (int16_t)constrain(amperage, -0x8000, 0x7FFF)); // send amperage in 0.01 A steps, range is -320A to 320A
        break;

    case MSP_ARMING_CONFIG:
        sbufWriteU8(dst, armingConfig()->auto_disarm_delay);
        sbufWriteU8(dst, armingConfig()->disarm_kill_switch);
        break;

    case MSP_LOOP_TIME:
        sbufWriteU16(dst, (uint16_t)gyro.targetLooptime);
        break;

    case MSP_RC_TUNING:
        sbufWriteU8(dst, currentControlRateProfile->rcRate8);
        sbufWriteU8(dst, currentControlRateProfile->rcExpo8);
        for (int i = 0 ; i < 3; i++) {
            sbufWriteU8(dst, currentControlRateProfile->rates[i]); // R,P,Y see flight_dynamics_index_t
        }
        sbufWriteU8(dst, currentControlRateProfile->dynThrPID);
        sbufWriteU8(dst, currentControlRateProfile->thrMid8);
        sbufWriteU8(dst, currentControlRateProfile->thrExpo8);
        sbufWriteU16(dst, currentControlRateProfile->tpa_breakpoint);
        sbufWriteU8(dst, currentControlRateProfile->rcYawExpo8);
        sbufWriteU8(dst, currentControlRateProfile->rcYawRate8);
        break;

    case MSP_PID:
        for (int i = 0; i < PID_ITEM_COUNT; i++) {
            sbufWriteU8(dst, currentPidProfile->P8[i]);
            sbufWriteU8(dst, currentPidProfile->I8[i]);
            sbufWriteU8(dst, currentPidProfile->D8[i]);
        }
        break;

    case MSP_PIDNAMES:
        for (const char *c = pidnames; *c; c++) {
            sbufWriteU8(dst, *c);
        }
        break;

    case MSP_PID_CONTROLLER:
        sbufWriteU8(dst, PID_CONTROLLER_BETAFLIGHT);
        break;

    case MSP_MODE_RANGES:
        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            const modeActivationCondition_t *mac = modeActivationConditions(i);
            const box_t *box = &boxes[mac->modeId];
            sbufWriteU8(dst, box->permanentId);
            sbufWriteU8(dst, mac->auxChannelIndex);
            sbufWriteU8(dst, mac->range.startStep);
            sbufWriteU8(dst, mac->range.endStep);
        }
        break;

    case MSP_ADJUSTMENT_RANGES:
        for (int i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            const adjustmentRange_t *adjRange = adjustmentRanges(i);
            sbufWriteU8(dst, adjRange->adjustmentIndex);
            sbufWriteU8(dst, adjRange->auxChannelIndex);
            sbufWriteU8(dst, adjRange->range.startStep);
            sbufWriteU8(dst, adjRange->range.endStep);
            sbufWriteU8(dst, adjRange->adjustmentFunction);
            sbufWriteU8(dst, adjRange->auxSwitchChannelIndex);
        }
        break;

    case MSP_BOXNAMES:
        serializeBoxNamesReply(dst);
        break;

    case MSP_BOXIDS:
        for (int i = 0; i < activeBoxIdCount; i++) {
            const box_t *box = findBoxByActiveBoxId(activeBoxIds[i]);
            if (!box) {
                continue;
            }
            sbufWriteU8(dst, box->permanentId);
        }
        break;

    case MSP_MISC:
        sbufWriteU16(dst, rxConfig()->midrc);

        sbufWriteU16(dst, motorConfig()->minthrottle);
        sbufWriteU16(dst, motorConfig()->maxthrottle);
        sbufWriteU16(dst, motorConfig()->mincommand);

        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);
        sbufWriteU8(dst, 0); // gps_type
        sbufWriteU8(dst, 0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
        sbufWriteU8(dst, 0); // gps_ubx_sbas

        sbufWriteU8(dst, batteryConfig()->multiwiiCurrentMeterOutput);
        sbufWriteU8(dst, rxConfig()->rssi_channel);
        sbufWriteU8(dst, 0);

        sbufWriteU8(dst, batteryConfig()->vbatscale);
        sbufWriteU8(dst, batteryConfig()->vbatmincellvoltage);
        sbufWriteU8(dst, batteryConfig()->vbatmaxcellvoltage);
        sbufWriteU8(dst, batteryConfig()->vbatwarningcellvoltage);
        break;

    case MSP_MOTOR_PINS:
        // FIXME This is hardcoded and should not be.
        for (int i = 0; i < 8; i++) {
            sbufWriteU8(dst, i + 1);
        }
        break;

    case MSP_DEBUG:
        // output some useful QA statistics
        // debug[x] = ((hse_value / 1000000) * 1000) + (SystemCoreClock / 1000000);         // XX0YY [crystal clock : core clock]

        for (int i = 0; i < DEBUG16_VALUE_COUNT; i++) {
            sbufWriteU16(dst, debug[i]);      // 4 variables are here for general monitoring purpose
        }
        break;

    // Additional commands that are not compatible with MultiWii
    case MSP_ACC_TRIM:
        sbufWriteU16(dst, accelerometerConfig()->accelerometerTrims.values.pitch);
        sbufWriteU16(dst, accelerometerConfig()->accelerometerTrims.values.roll);
        break;

    case MSP_UID:
        sbufWriteU32(dst, U_ID_0);
        sbufWriteU32(dst, U_ID_1);
        sbufWriteU32(dst, U_ID_2);
        break;

    case MSP_FEATURE:
        sbufWriteU32(dst, featureMask());
        break;

    case MSP_BOARD_ALIGNMENT:
        sbufWriteU16(dst, boardAlignment()->rollDegrees);
        sbufWriteU16(dst, boardAlignment()->pitchDegrees);
        sbufWriteU16(dst, boardAlignment()->yawDegrees);
        break;

    case MSP_VOLTAGE_METER_CONFIG:
        sbufWriteU8(dst, batteryConfig()->vbatscale);
        sbufWriteU8(dst, batteryConfig()->vbatmincellvoltage);
        sbufWriteU8(dst, batteryConfig()->vbatmaxcellvoltage);
        sbufWriteU8(dst, batteryConfig()->vbatwarningcellvoltage);
        sbufWriteU8(dst, batteryConfig()->batteryMeterType);
        break;

    case MSP_CURRENT_METER_CONFIG:
        sbufWriteU16(dst, batteryConfig()->currentMeterScale);
        sbufWriteU16(dst, batteryConfig()->currentMeterOffset);
        sbufWriteU8(dst, batteryConfig()->currentMeterType);
        sbufWriteU16(dst, batteryConfig()->batteryCapacity);
        break;

    case MSP_MIXER:
        sbufWriteU8(dst, mixerConfig()->mixerMode);
        break;

    case MSP_RX_CONFIG:
        sbufWriteU8(dst, rxConfig()->serialrx_provider);
        sbufWriteU16(dst, rxConfig()->maxcheck);
        sbufWriteU16(dst, rxConfig()->midrc);
        sbufWriteU16(dst, rxConfig()->mincheck);
        sbufWriteU8(dst, rxConfig()->spektrum_sat_bind);
        sbufWriteU16(dst, rxConfig()->rx_min_usec);
        sbufWriteU16(dst, rxConfig()->rx_max_usec);
        sbufWriteU8(dst, rxConfig()->rcInterpolation);
        sbufWriteU8(dst, rxConfig()->rcInterpolationInterval);
        sbufWriteU16(dst, rxConfig()->airModeActivateThreshold);
        sbufWriteU8(dst, rxConfig()->rx_spi_protocol);
        sbufWriteU32(dst, rxConfig()->rx_spi_id);
        sbufWriteU8(dst, rxConfig()->rx_spi_rf_channel_count);
        sbufWriteU8(dst, rxConfig()->fpvCamAngleDegrees);
        break;

    case MSP_FAILSAFE_CONFIG:
        sbufWriteU8(dst, failsafeConfig()->failsafe_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_off_delay);
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle);
        sbufWriteU8(dst, failsafeConfig()->failsafe_kill_switch);
        sbufWriteU16(dst, failsafeConfig()->failsafe_throttle_low_delay);
        sbufWriteU8(dst, failsafeConfig()->failsafe_procedure);
        break;

    case MSP_RXFAIL_CONFIG:
        for (int i = 0; i < rxRuntimeConfig.channelCount; i++) {
            sbufWriteU8(dst, rxFailsafeChannelConfigs(i)->mode);
            sbufWriteU16(dst, RXFAIL_STEP_TO_CHANNEL_VALUE(rxFailsafeChannelConfigs(i)->step));
        }
        break;

    case MSP_RSSI_CONFIG:
        sbufWriteU8(dst, rxConfig()->rssi_channel);
        break;

    case MSP_RX_MAP:
        sbufWriteData(dst, rxConfig()->rcmap, MAX_MAPPABLE_RX_INPUTS);
        break;

    case MSP_BF_CONFIG:
        sbufWriteU8(dst, mixerConfig()->mixerMode);

        sbufWriteU32(dst, featureMask());

        sbufWriteU8(dst, rxConfig()->serialrx_provider);

        sbufWriteU16(dst, boardAlignment()->rollDegrees);
        sbufWriteU16(dst, boardAlignment()->pitchDegrees);
        sbufWriteU16(dst, boardAlignment()->yawDegrees);

        sbufWriteU16(dst, batteryConfig()->currentMeterScale);
        sbufWriteU16(dst, batteryConfig()->currentMeterOffset);
        break;

    case MSP_CF_SERIAL_CONFIG:
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                continue;
            };
            sbufWriteU8(dst, serialConfig()->portConfigs[i].identifier);
            sbufWriteU16(dst, serialConfig()->portConfigs[i].functionMask);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].msp_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].gps_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].telemetry_baudrateIndex);
            sbufWriteU8(dst, serialConfig()->portConfigs[i].blackbox_baudrateIndex);
        }
        break;


    case MSP_DATAFLASH_SUMMARY:
        serializeDataflashSummaryReply(dst);
        break;

    case MSP_SDCARD_SUMMARY:
        serializeSDCardSummaryReply(dst);
        break;

    case MSP_BF_BUILD_INFO:
        sbufWriteData(dst, buildDate, 11); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
        sbufWriteU32(dst, 0); // future exp
        sbufWriteU32(dst, 0); // future exp
        break;

    case MSP_3D:
        sbufWriteU16(dst, flight3DConfig()->deadband3d_low);
        sbufWriteU16(dst, flight3DConfig()->deadband3d_high);
        sbufWriteU16(dst, flight3DConfig()->neutral3d);
        break;

    case MSP_RC_DEADBAND:
        sbufWriteU8(dst, rcControlsConfig()->deadband);
        sbufWriteU8(dst, rcControlsConfig()->yaw_deadband);
        sbufWriteU8(dst, rcControlsConfig()->alt_hold_deadband);
        sbufWriteU16(dst, flight3DConfig()->deadband3d_throttle);
        break;

    case MSP_SENSOR_ALIGNMENT:
        sbufWriteU8(dst, gyroConfig()->gyro_align);
        sbufWriteU8(dst, accelerometerConfig()->acc_align);
        break;

    case MSP_ADVANCED_CONFIG:
        if (gyroConfig()->gyro_lpf) {
            sbufWriteU8(dst, 8); // If gyro_lpf != OFF then looptime is set to 1000
            sbufWriteU8(dst, 1);
        } else {
            sbufWriteU8(dst, gyroConfig()->gyro_sync_denom);
            sbufWriteU8(dst, pidConfig()->pid_process_denom);
        }
        sbufWriteU8(dst, motorConfig()->dev.useUnsyncedPwm);
        sbufWriteU8(dst, motorConfig()->dev.motorPwmProtocol);
        sbufWriteU16(dst, motorConfig()->dev.motorPwmRate);
        sbufWriteU16(dst, (uint16_t)lrintf(motorConfig()->digitalIdleOffsetPercent * 100));
        sbufWriteU8(dst, gyroConfig()->gyro_use_32khz);
        //!!TODO gyro_isr_update to be added pending decision
        //sbufWriteU8(dst, gyroConfig()->gyro_isr_update);
        sbufWriteU8(dst, motorConfig()->dev.motorPwmInversion);
        break;

    case MSP_FILTER_CONFIG :
        sbufWriteU8(dst, gyroConfig()->gyro_soft_lpf_hz);
        sbufWriteU16(dst, currentPidProfile->dterm_lpf_hz);
        sbufWriteU16(dst, currentPidProfile->yaw_lpf_hz);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_1);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_1);
        sbufWriteU16(dst, currentPidProfile->dterm_notch_hz);
        sbufWriteU16(dst, currentPidProfile->dterm_notch_cutoff);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_hz_2);
        sbufWriteU16(dst, gyroConfig()->gyro_soft_notch_cutoff_2);
        break;

    case MSP_PID_ADVANCED:
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0); // was pidProfile.yaw_p_limit
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, currentPidProfile->vbatPidCompensation);
        sbufWriteU8(dst, currentPidProfile->setpointRelaxRatio);
        sbufWriteU8(dst, currentPidProfile->dtermSetpointWeight);
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU8(dst, 0); // reserved
        sbufWriteU16(dst, (uint16_t)lrintf(currentPidProfile->rateAccelLimit * 10));
        sbufWriteU16(dst, (uint16_t)lrintf(currentPidProfile->yawRateAccelLimit * 10));
        sbufWriteU8(dst, currentPidProfile->levelAngleLimit);
        sbufWriteU8(dst, currentPidProfile->levelSensitivity);
        break;

    case MSP_SENSOR_CONFIG:
        sbufWriteU8(dst, accelerometerConfig()->acc_hardware);
        // sbufWriteU8(dst, barometerConfig()->baro_hardware);
        sbufWriteU8(dst, 0); // replaced above
        break;

    case MSP_REBOOT:
        if (mspPostProcessFn) {
            *mspPostProcessFn = mspRebootFn;
        }
        break;

    default:
        return false;
    }
    return true;
}

static mspResult_e mspFcProcessInCommand(uint8_t cmdMSP, sbuf_t *src)
{
    uint32_t i;
    uint8_t value;
    const unsigned int dataSize = sbufBytesRemaining(src);
#ifdef GPS
    uint8_t wp_no;
    int32_t lat = 0, lon = 0, alt = 0;
#endif
    switch (cmdMSP) {
    case MSP_SELECT_SETTING:
        value = sbufReadU8(src);
        if ((value & RATEPROFILE_MASK) == 0) {
            if (!ARMING_FLAG(ARMED)) {
                if (value >= MAX_PROFILE_COUNT) {
                    value = 0;
                }
                changePidProfile(value);
            }
        } else {
            value = value & ~RATEPROFILE_MASK;

            if (value >= CONTROL_RATE_PROFILE_COUNT) {
                value = 0;
            }
            changeControlRateProfile(value);
        }
        break;

    case MSP_SET_HEAD:
        magHold = sbufReadU16(src);
        break;

    case MSP_SET_RAW_RC:
#ifdef USE_RX_MSP
        {
            uint8_t channelCount = dataSize / sizeof(uint16_t);
            if (channelCount > MAX_SUPPORTED_RC_CHANNEL_COUNT) {
                return MSP_RESULT_ERROR;
            } else {
                uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
                for (int i = 0; i < channelCount; i++) {
                    frame[i] = sbufReadU16(src);
                }
                rxMspFrameReceive(frame, channelCount);
            }
        }
#endif
        break;
    case MSP_SET_ACC_TRIM:
        accelerometerConfigMutable()->accelerometerTrims.values.pitch = sbufReadU16(src);
        accelerometerConfigMutable()->accelerometerTrims.values.roll  = sbufReadU16(src);
        break;
    case MSP_SET_ARMING_CONFIG:
        armingConfigMutable()->auto_disarm_delay = sbufReadU8(src);
        armingConfigMutable()->disarm_kill_switch = sbufReadU8(src);
        break;

    case MSP_SET_LOOP_TIME:
        sbufReadU16(src);
        break;

    case MSP_SET_PID_CONTROLLER:
        break;

    case MSP_SET_PID:
        for (int i = 0; i < PID_ITEM_COUNT; i++) {
            currentPidProfile->P8[i] = sbufReadU8(src);
            currentPidProfile->I8[i] = sbufReadU8(src);
            currentPidProfile->D8[i] = sbufReadU8(src);
        }
        pidInitConfig(currentPidProfile);
        break;

    case MSP_SET_MODE_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            i = sbufReadU8(src);
            const box_t *box = findBoxByPermenantId(i);
            if (box) {
                mac->modeId = box->boxId;
                mac->auxChannelIndex = sbufReadU8(src);
                mac->range.startStep = sbufReadU8(src);
                mac->range.endStep = sbufReadU8(src);

                useRcControlsConfig(modeActivationConditions(0), currentPidProfile);
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_ADJUSTMENT_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *adjRange = adjustmentRangesMutable(i);
            i = sbufReadU8(src);
            if (i < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                adjRange->adjustmentIndex = i;
                adjRange->auxChannelIndex = sbufReadU8(src);
                adjRange->range.startStep = sbufReadU8(src);
                adjRange->range.endStep = sbufReadU8(src);
                adjRange->adjustmentFunction = sbufReadU8(src);
                adjRange->auxSwitchChannelIndex = sbufReadU8(src);
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RC_TUNING:
        if (dataSize >= 10) {
            currentControlRateProfile->rcRate8 = sbufReadU8(src);
            currentControlRateProfile->rcExpo8 = sbufReadU8(src);
            for (int i = 0; i < 3; i++) {
                value = sbufReadU8(src);
                currentControlRateProfile->rates[i] = MIN(value, i == FD_YAW ? CONTROL_RATE_CONFIG_YAW_RATE_MAX : CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            }
            value = sbufReadU8(src);
            currentControlRateProfile->dynThrPID = MIN(value, CONTROL_RATE_CONFIG_TPA_MAX);
            currentControlRateProfile->thrMid8 = sbufReadU8(src);
            currentControlRateProfile->thrExpo8 = sbufReadU8(src);
            currentControlRateProfile->tpa_breakpoint = sbufReadU16(src);
            if (dataSize >= 11) {
                currentControlRateProfile->rcYawExpo8 = sbufReadU8(src);
            }
            if (dataSize >= 12) {
                currentControlRateProfile->rcYawRate8 = sbufReadU8(src);
            }
            generateThrottleCurve();
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_MISC:
        rxConfigMutable()->midrc = sbufReadU16(src);
        motorConfigMutable()->minthrottle = sbufReadU16(src);
        motorConfigMutable()->maxthrottle = sbufReadU16(src);
        motorConfigMutable()->mincommand = sbufReadU16(src);

        failsafeConfigMutable()->failsafe_throttle = sbufReadU16(src);

        sbufReadU8(src); // gps_type
        sbufReadU8(src); // gps_baudrate
        sbufReadU8(src); // gps_ubx_sbas

        batteryConfigMutable()->multiwiiCurrentMeterOutput = sbufReadU8(src);
        rxConfigMutable()->rssi_channel = sbufReadU8(src);
        sbufReadU8(src);

        batteryConfigMutable()->vbatscale = sbufReadU8(src);           // actual vbatscale as intended
        batteryConfigMutable()->vbatmincellvoltage = sbufReadU8(src);  // vbatlevel_warn1 in MWC2.3 GUI
        batteryConfigMutable()->vbatmaxcellvoltage = sbufReadU8(src);  // vbatlevel_warn2 in MWC2.3 GUI
        batteryConfigMutable()->vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
        break;

    case MSP_SET_MOTOR:
        for (int i = 0; i < 8; i++) { // FIXME should this use MAX_MOTORS or MAX_SUPPORTED_MOTORS instead of 8
            motor_disarmed[i] = convertExternalToMotor(sbufReadU16(src));
        }
        break;

    case MSP_SET_3D:
        flight3DConfigMutable()->deadband3d_low = sbufReadU16(src);
        flight3DConfigMutable()->deadband3d_high = sbufReadU16(src);
        flight3DConfigMutable()->neutral3d = sbufReadU16(src);
        break;

    case MSP_SET_RC_DEADBAND:
        rcControlsConfigMutable()->deadband = sbufReadU8(src);
        rcControlsConfigMutable()->yaw_deadband = sbufReadU8(src);
        rcControlsConfigMutable()->alt_hold_deadband = sbufReadU8(src);
        flight3DConfigMutable()->deadband3d_throttle = sbufReadU16(src);
        break;

    case MSP_SET_RESET_CURR_PID:
        resetPidProfile(currentPidProfile);
        break;
    case MSP_SET_SENSOR_ALIGNMENT:
        gyroConfigMutable()->gyro_align = sbufReadU8(src);
        accelerometerConfigMutable()->acc_align = sbufReadU8(src);
        break;

    case MSP_SET_ADVANCED_CONFIG:
        gyroConfigMutable()->gyro_sync_denom = sbufReadU8(src);
        pidConfigMutable()->pid_process_denom = sbufReadU8(src);
        motorConfigMutable()->dev.useUnsyncedPwm = sbufReadU8(src);
#ifdef USE_DSHOT
        motorConfigMutable()->dev.motorPwmProtocol = constrain(sbufReadU8(src), 0, PWM_TYPE_MAX - 1);
#else
        motorConfigMutable()->dev.motorPwmProtocol = constrain(sbufReadU8(src), 0, PWM_TYPE_BRUSHED);
#endif
        motorConfigMutable()->dev.motorPwmRate = sbufReadU16(src);
        if (sbufBytesRemaining(src) >= 2) {
            motorConfigMutable()->digitalIdleOffsetPercent = sbufReadU16(src) / 100.0f;
        }
        if (sbufBytesRemaining(src)) {
            gyroConfigMutable()->gyro_use_32khz = sbufReadU8(src);
        }
        //!!TODO gyro_isr_update to be added pending decision
        /*if (sbufBytesRemaining(src)) {
            gyroConfigMutable()->gyro_isr_update = sbufReadU8(src);
        }*/
        validateAndFixGyroConfig();

        if (sbufBytesRemaining(src)) {        
            motorConfigMutable()->dev.motorPwmInversion = sbufReadU8(src);
        }
        break;

    case MSP_SET_FILTER_CONFIG:
        gyroConfigMutable()->gyro_soft_lpf_hz = sbufReadU8(src);
        currentPidProfile->dterm_lpf_hz = sbufReadU16(src);
        currentPidProfile->yaw_lpf_hz = sbufReadU16(src);
        if (dataSize > 5) {
            gyroConfigMutable()->gyro_soft_notch_hz_1 = sbufReadU16(src);
            gyroConfigMutable()->gyro_soft_notch_cutoff_1 = sbufReadU16(src);
            currentPidProfile->dterm_notch_hz = sbufReadU16(src);
            currentPidProfile->dterm_notch_cutoff = sbufReadU16(src);
        }
        if (dataSize > 13) {
            gyroConfigMutable()->gyro_soft_notch_hz_2 = sbufReadU16(src);
            gyroConfigMutable()->gyro_soft_notch_cutoff_2 = sbufReadU16(src);
        }
        // reinitialize the gyro filters with the new values
        validateAndFixGyroConfig();
        gyroInitFilters();
        // reinitialize the PID filters with the new values
        pidInitFilters(currentPidProfile);
        break;

    case MSP_SET_PID_ADVANCED:
        sbufReadU16(src);
        sbufReadU16(src);
        sbufReadU16(src); // was pidProfile.yaw_p_limit
        sbufReadU8(src); // reserved
        currentPidProfile->vbatPidCompensation = sbufReadU8(src);
        currentPidProfile->setpointRelaxRatio = sbufReadU8(src);
        currentPidProfile->dtermSetpointWeight = sbufReadU8(src);
        sbufReadU8(src); // reserved
        sbufReadU8(src); // reserved
        sbufReadU8(src); // reserved
        currentPidProfile->rateAccelLimit = sbufReadU16(src) / 10.0f;
        currentPidProfile->yawRateAccelLimit = sbufReadU16(src) / 10.0f;
        if (dataSize > 17) {
            currentPidProfile->levelAngleLimit = sbufReadU8(src);
            currentPidProfile->levelSensitivity = sbufReadU8(src);
        }
        pidInitConfig(currentPidProfile);
        break;

    case MSP_SET_SENSOR_CONFIG:
        accelerometerConfigMutable()->acc_hardware = sbufReadU8(src);
        // barometerConfigMutable()->baro_hardware = sbufReadU8(src);
        sbufReadU8(src); // replaced above
        break;

    case MSP_RESET_CONF:
        if (!ARMING_FLAG(ARMED)) {
            resetEEPROM();
            readEEPROM();
        }
        break;

    case MSP_ACC_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
        break;

    case MSP_MAG_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            ENABLE_STATE(CALIBRATE_MAG);
        break;

    case MSP_EEPROM_WRITE:
        if (ARMING_FLAG(ARMED)) {
            return MSP_RESULT_ERROR;
        }
        writeEEPROM();
        readEEPROM();
        break;

    case MSP_SET_FEATURE:
        featureClearAll();
        featureSet(sbufReadU32(src)); // features bitmap
        break;

    case MSP_SET_BOARD_ALIGNMENT:
        boardAlignmentMutable()->rollDegrees = sbufReadU16(src);
        boardAlignmentMutable()->pitchDegrees = sbufReadU16(src);
        boardAlignmentMutable()->yawDegrees = sbufReadU16(src);
        break;

    case MSP_SET_VOLTAGE_METER_CONFIG:
        batteryConfigMutable()->vbatscale = sbufReadU8(src);           // actual vbatscale as intended
        batteryConfigMutable()->vbatmincellvoltage = sbufReadU8(src);  // vbatlevel_warn1 in MWC2.3 GUI
        batteryConfigMutable()->vbatmaxcellvoltage = sbufReadU8(src);  // vbatlevel_warn2 in MWC2.3 GUI
        batteryConfigMutable()->vbatwarningcellvoltage = sbufReadU8(src);  // vbatlevel when buzzer starts to alert
        if (dataSize > 4) {
            batteryConfigMutable()->batteryMeterType = sbufReadU8(src);
        }
        break;

    case MSP_SET_CURRENT_METER_CONFIG:
        batteryConfigMutable()->currentMeterScale = sbufReadU16(src);
        batteryConfigMutable()->currentMeterOffset = sbufReadU16(src);
        batteryConfigMutable()->currentMeterType = sbufReadU8(src);
        batteryConfigMutable()->batteryCapacity = sbufReadU16(src);
        break;

#ifndef USE_QUAD_MIXER_ONLY
    case MSP_SET_MIXER:
        mixerConfigMutable()->mixerMode = sbufReadU8(src);
        break;
#endif

    case MSP_SET_RX_CONFIG:
        rxConfigMutable()->serialrx_provider = sbufReadU8(src);
        rxConfigMutable()->maxcheck = sbufReadU16(src);
        rxConfigMutable()->midrc = sbufReadU16(src);
        rxConfigMutable()->mincheck = sbufReadU16(src);
        rxConfigMutable()->spektrum_sat_bind = sbufReadU8(src);
        if (dataSize > 8) {
            rxConfigMutable()->rx_min_usec = sbufReadU16(src);
            rxConfigMutable()->rx_max_usec = sbufReadU16(src);
        }
        if (dataSize > 12) {
            rxConfigMutable()->rcInterpolation = sbufReadU8(src);
            rxConfigMutable()->rcInterpolationInterval = sbufReadU8(src);
            rxConfigMutable()->airModeActivateThreshold = sbufReadU16(src);
        }
        if (dataSize > 16) {
            rxConfigMutable()->rx_spi_protocol = sbufReadU8(src);
            rxConfigMutable()->rx_spi_id = sbufReadU32(src);
            rxConfigMutable()->rx_spi_rf_channel_count = sbufReadU8(src);
        }
        if (dataSize > 22) {
            rxConfigMutable()->fpvCamAngleDegrees = sbufReadU8(src);
        }
        break;

    case MSP_SET_FAILSAFE_CONFIG:
        failsafeConfigMutable()->failsafe_delay = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_off_delay = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_throttle = sbufReadU16(src);
        failsafeConfigMutable()->failsafe_kill_switch = sbufReadU8(src);
        failsafeConfigMutable()->failsafe_throttle_low_delay = sbufReadU16(src);
        failsafeConfigMutable()->failsafe_procedure = sbufReadU8(src);
        break;

    case MSP_SET_RXFAIL_CONFIG:
        i = sbufReadU8(src);
        if (i < MAX_SUPPORTED_RC_CHANNEL_COUNT) {
            rxConfigMutable()->failsafe_channel_configurations[i].mode = sbufReadU8(src);
            rxConfigMutable()->failsafe_channel_configurations[i].step = CHANNEL_VALUE_TO_RXFAIL_STEP(sbufReadU16(src));
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_RSSI_CONFIG:
        rxConfigMutable()->rssi_channel = sbufReadU8(src);
        break;

    case MSP_SET_RX_MAP:
        for (int i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
            rxConfigMutable()->rcmap[i] = sbufReadU8(src);
        }
        break;

    case MSP_SET_BF_CONFIG:
#ifdef USE_QUAD_MIXER_ONLY
        sbufReadU8(src); // mixerMode ignored
#else
        mixerConfigMutable()->mixerMode = sbufReadU8(src); // mixerMode
#endif

        featureClearAll();
        featureSet(sbufReadU32(src)); // features bitmap

        rxConfigMutable()->serialrx_provider = sbufReadU8(src); // serialrx_type

        boardAlignmentMutable()->rollDegrees = sbufReadU16(src); // board_align_roll
        boardAlignmentMutable()->pitchDegrees = sbufReadU16(src); // board_align_pitch
        boardAlignmentMutable()->yawDegrees = sbufReadU16(src); // board_align_yaw

        batteryConfigMutable()->currentMeterScale = sbufReadU16(src);
        batteryConfigMutable()->currentMeterOffset = sbufReadU16(src);
        break;

    case MSP_SET_CF_SERIAL_CONFIG:
        {
            uint8_t portConfigSize = sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(uint8_t) * 4);

            if (dataSize % portConfigSize != 0) {
                return MSP_RESULT_ERROR;
            }

            uint8_t remainingPortsInPacket = dataSize / portConfigSize;

            while (remainingPortsInPacket--) {
                uint8_t identifier = sbufReadU8(src);

                serialPortConfig_t *portConfig = serialFindPortConfiguration(identifier);
                if (!portConfig) {
                    return MSP_RESULT_ERROR;
                }

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU16(src);
                portConfig->msp_baudrateIndex = sbufReadU8(src);
                portConfig->gps_baudrateIndex = sbufReadU8(src);
                portConfig->telemetry_baudrateIndex = sbufReadU8(src);
                portConfig->blackbox_baudrateIndex = sbufReadU8(src);
            }
        }
        break;

    case MSP_SET_NAME:
        memset(systemConfigMutable()->name, 0, ARRAYLEN(systemConfig()->name));
        for (unsigned int i = 0; i < MIN(MAX_NAME_LENGTH, dataSize); i++) {
            systemConfigMutable()->name[i] = sbufReadU8(src);
        }
        break;

    default:
        // we do not know how to handle the (valid) message, indicate error MSP $M!
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const uint8_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (mspFcProcessOutCommand(cmdMSP, dst, mspPostProcessFn)) {
        ret = MSP_RESULT_ACK;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    } else if (cmdMSP == MSP_SET_4WAY_IF) {
        mspFc4waySerialCommand(dst, src, mspPostProcessFn);
        ret = MSP_RESULT_ACK;
#endif
    } else {
        ret = mspFcProcessInCommand(cmdMSP, src);
    }
    reply->result = ret;
    return ret;
}

/*
 * Return a pointer to the process command function
 */
void mspFcInit(void)
{
    initActiveBoxIds();
}

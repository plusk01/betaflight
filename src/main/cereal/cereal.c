#include "cereal/cereal.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/printf.h"

#include "io/serial.h"

#define CEREAL_INITIAL_PORT_MODE    MODE_TX

static serialPort_t *cerealPort = NULL;
static bool cerealPortEnabled = false;

static serialPortConfig_t *portConfig;

void cerealInit(void) {
    portConfig = findSerialPortConfig(FUNCTION_CEREAL);

    configureCerealPort();

    setPrintfSerialPort(cerealPort);
    printf("printf Configured!");
}

void cerealProcess(uint32_t currentTime) {
    if (cerealPortEnabled) {
        // serialPrint(cerealPort, "Hi\n");
        // setPrintfSerialPort(cerealPort);
        printf("Current Time: %d", currentTime);
    }
}

void configureCerealPort(void) {

    baudRate_e baudRateIndex = BAUD_115200;

    cerealPort = openSerialPort(portConfig->identifier, FUNCTION_CEREAL, NULL, baudRates[baudRateIndex], CEREAL_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);

    if (!cerealPort) {
        return;
    }

    cerealPortEnabled = true;
}
#include "messages.h"
#include "globals.h"
#include <string.h>

void report_error(ErrorCode_t errCode, const char *file, const char *func, int line) {
    UARTMsg_t errMsg = {.type = MSG_TYPE_ERROR, .errCode = errCode, .file = file, .func = func, .line = line};
    osMessageQueuePut(xUARTQueueHandle, &errMsg, 0U, 0);
}

void logMsgFromTask(ErrorCode_t errCode, MsgType_t type, const char *file, const char *func, int line, const char *message) {
    UARTMsg_t logMsg = {.type = type, .errCode = errCode, .file = file, .func = func, .line = line};
    strncpy(logMsg.msg, message, sizeof(logMsg.msg)-1);
    osMessageQueuePut(xUARTQueueHandle, &logMsg, 0U, 0);

}

void logAllHealthFlags(void) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
        "HealthFlags: uart=%d, alarm=%d, bmp180=%d, button=%d, env_mgr=%d, voltage_mgr=%d, watchdog=%d",
        healthFlags.uart,
        healthFlags.alarm,
        healthFlags.bmp180,
        healthFlags.button,
        healthFlags.env_mgr,
        healthFlags.voltage_mgr,
        healthFlags.watchdog
    );
    LOG_FROM_TASK(ERR_CODE_SUCCESS, MSG_TYPE_HEALTH, buffer);
}
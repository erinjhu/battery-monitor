#include "messages.h"
#include "globals.h"

void report_error(ErrorCode_t errCode, const char *file, const char *func, int line) {
    UARTMsg_t errMsg = {.type = MSG_TYPE_ERROR, .errCode = errCode, .file = file, .func = func, .line = line};
    osMessageQueuePut(xUARTQueueHandle, &errMsg, 0U, 0);
}

void logMsgFromTask(ErrorCode_t errCode, MsgType_t type, const char *file, const char *func, int line, const char *message) {
    UARTMsg_t logMsg = {.type = type, .errCode = errCode, .file = file, .func = func, .line = line};
    strncpy(logMsg.msg, message, sizeof(logMsg.msg)-1);
    osMessageQueuePut(xUARTQueueHandle, &logMsg, 0U, 0);

}
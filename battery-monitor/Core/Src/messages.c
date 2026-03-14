#include "messages.h"
#include "globals.h"

void report_error(ErrorCode_t errCode, const char *file, const char *func, int line) {
    UARTMsg_t errMsg = {.type = MSG_TYPE_ERROR, .errCode = errCode, .file = file, .func = func, .line = line};
    osMessageQueuePut(xUARTQueueHandle, &errMsg, 0U, 0);
}



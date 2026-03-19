#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>

typedef enum
{
  MSG_TYPE_VOLTAGE,
  MSG_TYPE_TEMP,
  MSG_TYPE_PRESSURE,
  MSG_TYPE_BUTTON,
  MSG_TYPE_HEALTH,
  MSG_TYPE_ERROR
} MsgType_t;

typedef enum
{
  ERR_CODE_SUCCESS = 0,
  ERR_CODE_UNKNOWN = 1,
  ERR_CODE_INVALID_ARG = 2,
  ERR_CODE_QUEUE_FULL = 3,
  ERR_CODE_MUTEX_TIMEOUT = 4,
  ERR_CODE_BUFF_TOO_SMALL = 5,
  ERR_CODE_LOG_MSG_SILENCED = 6,
  ERR_CODE_INVALID_STATE = 7,
  ERR_CODE_UNSUPPORTED_EVENT = 8,
  ERR_CODE_BUFF_OVERFLOW = 9,
  ERR_CODE_SEMAPHORE_TIMEOUT = 10,
  ERR_CODE_SEMAPHORE_FULL = 11,
  ERR_CODE_QUEUE_EMPTY = 12,
  ERR_CODE_NOT_MUTEX_OWNER = 13,
  ERR_CODE_PERSISTENT_CORRUPTED = 14,
  ERR_CODE_FAILED_UNPACK = 15,
  ERR_CODE_FAILED_PACK = 16,
  ERR_CODE_INVALID_STATE_TRANSITION = 17,
  ERR_CODE_FREERTOS_ASSERT_FAIL = 18,
  ERR_CODE_FAILED_STACK_CANARY = 19,
  ERR_CODE_ADC_TIMEOUT = 20,
  ERR_CODE_I2C_MEM_READ = 21,
  ERR_CODE_I2C_MEM_WRITE = 22,
  ERR_CODE_FAILED_BMP180_INIT = 23,
  ERR_CODE_FAILED_BMP180_CALIB = 24,
  ERR_CODE_WATCHDOG_HEALTHY = 25,
} ErrorCode_t;

typedef struct 
{
  MsgType_t type;  
  union {
    float value;  
    uint8_t str[8];
    ErrorCode_t errCode;
  };
  const char *file;
  const char *func;
  int line;
  char msg[64];
} UARTMsg_t;

void logMsgFromTask(ErrorCode_t errCode, MsgType_t type, const char *file, const char *func, int line, const char *message);

#define LOG_FROM_TASK(_errCode, _type, _message) logMsgFromTask((_errCode), (_type), __FILE__, __func__, __LINE__, (_message))

void report_error(ErrorCode_t errCode, const char *file, const char *func, int line);

#define REPORT_ERROR(errCode, _healthFlag) report_error((errCode), __FILE__, __func__, __LINE__)


#define RETURN_IF_ERROR_CODE_CMSIS(_ret, _healthFlag) \
  do { \
    cmsisErrCode = _ret; \
    if (cmsisErrCode != osOK) { \
      *_healthFlag = HEALTH_ERROR; /* or your error code */ \
      REPORT_ERROR(cmsisErrCode, _healthFlag); \
      return; \
    } \
  } while (0)

#define RETURN_IF_ERROR_CODE_HAL(_ret, _healthFlag) \
  do { \
    halErrCode = _ret; \
    if (halErrCode != HAL_OK) { \
      *_healthFlag = HEALTH_ERROR; /* or your error code */ \
      REPORT_ERROR(halErrCode, _heathFlag); \
      return; \
    } \
  } while (0)



  
#endif // MESSAGES_H
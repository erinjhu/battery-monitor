#include "interrupts.h"
#include "globals.h"
#include "gpio.h"
#include "messages.h"
#include "cmsis_os.h"
#include "iwdg.h"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
  osStatus_t cmsisErrCode;
  // HAL_UART_Transmit(&huart2, (uint8_t*)"ISR Started\r\n", 21, 100);
  currentTime = HAL_GetTick();
  if (GPIO_PIN == GPIO_BUTTON && (currentTime - previousTime > 5))
  {
    //HAL_GPIO_TogglePin(GPIOA, GPIO_GREEN_LED);
    UARTMsg_t msg = {.type = MSG_TYPE_BUTTON};
    RETURN_IF_ERROR_CODE_CMSIS(osMessageQueuePut(xUARTQueueHandle, &msg, 0U, 0), &healthFlags.button);
    previousTime = currentTime;
    healthFlags.button = HEALTH_OK;
  }
}
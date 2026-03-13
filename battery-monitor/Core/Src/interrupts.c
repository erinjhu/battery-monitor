#include "interrupts.h"
#include "globals.h"
#include "gpio.h"
#include "messages.h"
#include "cmsis_os.h"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
  // HAL_UART_Transmit(&huart2, (uint8_t*)"ISR Started\r\n", 21, 100);
  currentTime = HAL_GetTick();
  if (GPIO_PIN == GPIO_BUTTON && (currentTime - previousTime > 5))
  {
    //HAL_GPIO_TogglePin(GPIOA, GPIO_GREEN_LED);
    UARTMsg_t msg = {.type = MSG_TYPE_BUTTON};
    osMessageQueuePut(xUARTQueueHandle, &msg, 0U, 0);
    previousTime = currentTime;
  }
}
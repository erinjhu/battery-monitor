# External Interrupt

## STM32 EXTI Interrupts

Pins can be configured to trigger an interrupt
|Type|Description|Code|
|--|--|--|
|Rising edge|Pin transitions from low to high (0 --> 1)|`GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;`|
|Falling edge|Pin transitions from high to low (1 --> 0)|`GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;`|
|Either edge|Either|`GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;`|

## Button Debouncing

**Bouncing**
- When pressing/releasing a button
- **Contacts:** metal pieces inside that complete the circuit
- **Bouncing:** the contacts quickly make and break contact, causing multiple on/off signals

**Debouncing**
```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
  // Record current time
  currentTime = HAL_GetTick(); 
  // Check which interrupt pin triggered the interrupt
  // Ensure enough time passes since last button press (5 ms)
  if (GPIO_PIN == GPIO_BUTTON && (currentTime - previousTime > 5)) 
  {
    UARTMsg_t msg = {.type = MSG_TYPE_BUTTON};
    osMessageQueuePut(xUARTQueueHandle, &msg, 0U, 0);
    previousTime = currentTime;
  }
}
```
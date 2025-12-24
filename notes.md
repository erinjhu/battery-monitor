## PWM

### PWM Frequency

Since $f_{\text{clock}}$ is usually too fast for PWM, use the prescaler and period values to obtain the desired $f_{\text{PWM}}$.

$f_{\text{PWM}} = \frac{f_\text{clock}}{(1 + \text{Prescaler})(1 + \text{Period})}$

| Variable           | Description                                                                                                                                                                           |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| $f_{\text{PWM}}$   | number of full on-off cycles completed per second (desired clock to run PWM signal)                                                                                                   |
| $f_{\text{clock}}$ | number of full on-off cycles completed per second (given clock; usually depends on the device)                                                                                        |
| $\text{Prescaler}$ | factor to divide the system clock by to get a slower value. use 1 + prescaler because there is a tick when the counter is at 0.                                                       |
| $\text{Period}$    | number of timer counts (ticks). e.g. if $\text{Period}=99$, there will be 100 ticks (timer at 0 for one tick, timer at 1 for one tick, ... , timer at 99 for one tick, repeat from 0) |

### Duty Cycle

The compare register value is the number of ticks when the desired duty cycle will be completed for a certain period.

$\text{Compare Register Value}=\frac{\text{Duty Cycle} }{100} \cdot \text{Period}$

| Variable                        | Description                                                                           |
| ------------------------------- | ------------------------------------------------------------------------------------- |
| $\text{Compare Register Value}$ | when the counter reaches the compare register value, switch the signal from ON to OFF |
| $\text{Duty Cycle}$             | % of the period that the signal is ON                                                 |
| $\text{Period}$                 | number of ticks in one full cycle                                                     |

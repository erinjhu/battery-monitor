# Watchdog

# Watchdog Task (Kick Watchdog)

- **Why is there a delay at the end of the loop?**
  - Prevent it from hogging CPU power and starving other tasks
  - Put task to sleep so other tasks can run

## Parameters

**Prescaler:** 
- Analogy: a prescaler of 4 puts the frequency at 0.25x speed
- Division factor for LSI clock 

LSI clock: 
- Low-speed internal clock 
- Separate from the main clock
- Stays active in low-power modes
- Uses less power but is less stable

**Reload value:**
- Starting value for watchdog counter
- Count from reload value down to zero
- Max: 4095

**Timeout**

Timeout in seconds:
$\frac{\text{Reload Value} + 1}{\text{LSI Frequency / Prescaler}}$
# Watchdog

- **Why is there a delay at the end of the loop?**
  - Prevent it from hogging CPU power and starving other tasks
  - Put task to sleep so other tasks can run
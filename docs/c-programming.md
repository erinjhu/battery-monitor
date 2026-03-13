# C Programming

## Headers

### Include Guard

Prevents header file contents from being included multiple times when you compile. Otherwise, types or variables can be defined multiple times.

*If the macro isn't defined, compile the rest of the file. Then, define the macro, so that the next time you look at the file, the preprocessor won't recompile it.*

```c
#ifndef MESSAGES_H      // if macro MESSAGES_H isn't defined
#define MESSAGES_H      // define it to indicate you included the file

// header file contents

#endif // MESSAGES_H
```

### Extern for Global Variables

globals.h

```c
// declare variable that will be defined in another file
extern volatile float fBatteryVoltage;
```

globals.c

```c
// define the variable and allocate memory for it
extern volatile float fBatteryVoltage;
```
Functions are extern by default, so you don't need to use `extern` for them.



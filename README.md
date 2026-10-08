# Openwater Open-MOTION Console Firmware

## Disclaimer

CAUTION - Investigational device. Limited by Federal (or United States) law to investigational use. The system described here has not been evaluated by the FDA and is not designed for the treatment or diagnosis of any disease. It is provided AS-IS, with no warranties. User assumes all liability and responsibility for identifying and mitigating risks associated with using this software.

This repository contains the firmware for the Motion Console.


CPPCheck
```bash

  cppcheck --enable=warning,style,performance,portability \
                   --std=c11 \
                   --error-exitcode=1 \
                   --force \
                   --inline-suppr \
                   --std=c11
                   --output-file=cppcheck_report.txt
                   --template="[{severity}] {file}:{line} {id} - {message}" \
                   --suppress=missingIncludeSystem \
                   --suppress=*:Core/Src/lwrb.c \
                   --suppress=*:Core/Src/jsmn.c \
                   --suppress=*:Core/Src/system_stm32h7xx.c \
                   --suppress=*:Core/Src/stm32h7xx_hal_msp.c \
                   --suppress=*:Core/Src/syscalls.c \
                   ./Core/Src ./USB_DEVICE/App


```
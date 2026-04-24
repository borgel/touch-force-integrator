## HID_RTOS
HID_RTOS is a demo app showing reading from a USB HID device. 
* Make sure to follow the setup process in the README to build and flash (when you configure the external loader, uncheck init!)
* Keyboard doesn't seem to completely work out of the box, but mouse does
* By default works on USB1 (the top one) via a USB A-C converter, per docs

Booting/project format
* the LCD example wants Boot_XIP.hex. I added a copy to this repo but it can be found with the full ST examples here https://github.com/STMicroelectronics/STM32CubeH7RS/tree/main/Projects/STM32H7S78-DK/Templates/Template_XIP


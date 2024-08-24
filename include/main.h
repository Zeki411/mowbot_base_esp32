#ifndef __MAIN_H__
#define __MAIN_H__

#include <Arduino.h>
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "mbb_motor_controller.h"


extern HardwareSerial uROS_Serial;


extern int mbb_main(void);


#endif // __MAIN_H__
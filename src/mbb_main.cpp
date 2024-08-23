#include "main.h"
#include "mbb_motor_controller.h"

MBBMotorController MBB_MotorCtrl;


int mbb_main(void) {

    MBB_MotorCtrl.init();
    
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  
    }
}
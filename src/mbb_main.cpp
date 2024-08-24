#include "main.h"
#include "mbb_uros.h"
#include "mbb_motor_controller.h"

MBBMotorController MBB_MotorCtrl;


int mbb_main(void) {
    // init
    mbb_uros_task_init();
    // MBB_MotorCtrl.init();

    mbb_uros_task_start();
    
    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);  
    }
}
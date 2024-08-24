#include "main.h"
#include "mbb_uros.h"
#include "mbb_mc.h"


int mbb_main(void) {
    mbb_uros_task_init();
    mbb_mc_task_init();

    vTaskDelay(100 / portTICK_PERIOD_MS);

    mbb_uros_task_start();
    mbb_mc_task_start();
    
    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);  
    }
}
#ifndef __MBB_MOTOR_CONTROLLER_H__
#define __MBB_MOTOR_CONTROLLER_H__

#include <Arduino.h>

#include "driver/uart.h"
#include "freertos/task.h"

#include "mbb_motor_controller_define.h"



class MBBMotorController  {
public:

    // Singleton pattern
    static MBBMotorController& getInstance() {
        static MBBMotorController instance; // Guaranteed to be created only once
        return instance;
    }

    // Delete copy constructor and assignment operator to prevent copies
    MBBMotorController(const MBBMotorController&) = delete;
    MBBMotorController& operator=(const MBBMotorController&) = delete;


    TaskHandle_t uart_rx_task_handle = NULL;


    MBBMotorController();
    ~MBBMotorController();

    void init();
    void start();

    
private:
    static MBBMotorController_MsgRxRunData_t last_rx_msg_run_data;
    static QueueHandle_t uart_rx_queue;
    static MBBMotorController_RxMsgMode_t rx_msg_mode;
    
    void init_hw_uart(void);
    static void uart_rx_task(void *arg);
};


#endif // __MBB_MOTOR_CONTROLLER_H__
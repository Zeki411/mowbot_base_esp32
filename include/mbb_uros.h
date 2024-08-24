#ifndef __MBB_UROS_H__
#define __MBB_UROS_H__

//  Hardware configuration
#define MBB_UROS_UART_RX 26
#define MBB_UROS_UART_TX 27
#define MBB_UROS_UART_BAUDRATE 115200

// Logging configuration
#define MBB_UROS_LOG_LEVEL ESP_LOG_INFO
#define MBB_UROS_LOG_TAG "mbb-uros"

// uROS node configuration
#define MBB_UROS_NODE_NAME "uros_mbb_base"
#define MBB_UROS_NODE_NAMESPACE ""
#define MBB_UROS_SUB_TOPIC_NAME "/mowbot_base/cmd_vel_unstamped"
#define MBB_UROS_PUB_TOPIC_NAME "/mowbot_base/odom"



// Task configuration
#define MBB_UROS_TASK_STACK_SIZE 4096
#define MBB_UROS_TASK_PRIORITY 10


extern void mbb_uros_task_init(void);
extern void mbb_uros_task_start(void);

#endif // __MBB_UROS_H__
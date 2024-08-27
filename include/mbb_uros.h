#ifndef __MBB_UROS_H__
#define __MBB_UROS_H__

//  Hardware configuration
#define MBB_UROS_UART_RX 26
#define MBB_UROS_UART_TX 27
#define MBB_UROS_UART_BAUDRATE 115200

// Logging configuration
#define MBB_UROS_LOG_LEVEL ESP_LOG_ERROR
#define MBB_UROS_LOG_TAG "mbb-uros"

// uROS node configuration
#define MBB_UROS_NODE_NAME "uros_mbb_base"
#define MBB_UROS_NODE_NAMESPACE ""
#define MBB_UROS_SUB_TOPIC_NAME "/mowbot_base/cmd_vel_unstamped"
#define MBB_UROS_PUB_TOPIC_NAME "/mowbot_base/odom"

#define MBB_UROS_ODOM_FRAME_ID "odom"
#define MBB_UROS_ODOM_CHILD_FRAME_ID "base_link"

#define MBB_UROS_PUB_TIMER_PERIOD_MS 100 // 10 Hz


#define MBB_UROS_ODOM_QUEUE_SIZE 1
#define MBB_UROS_CMD_VEL_QUEUE_SIZE 1


// Task configuration
#define MBB_UROS_TASK_STACK_SIZE 4096
#define MBB_UROS_TASK_PRIORITY 5

// Ping agent configuration
#define MBB_UROS_PING_AGENT_ATTEMPT_TIMEOUT_MS 1000
#define MBB_UROS_PING_AGENT_ATTMEPT_NUM 5
#define MBB_UROS_SYNC_AGENT_TIME_TIMEOUT_MS 1000



extern QueueHandle_t mbb_uros_odom_queue;
extern QueueHandle_t mbb_uros_cmd_vel_queue;

extern void mbb_uros_task_init(void);
extern void mbb_uros_task_start(void);

extern int64_t mbb_uros_get_epoch_millis();
extern int64_t mbb_uros_get_epoch_nanos();

#endif // __MBB_UROS_H__
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/string.h>
#include <esp_log.h>
#include <esp_intr_alloc.h>

#include "main.h"
#include "mbb_uros.h"

HardwareSerial uROS_Serial(1);
QueueHandle_t mbb_uros_odom_queue;
QueueHandle_t mbb_uros_cmd_vel_queue;

rcl_allocator_t mbb_uros_allocator;
rclc_support_t mbb_uros_support;
rcl_node_t mbb_uros_node;
rcl_subscription_t mbb_uros_sub;
rcl_publisher_t mbb_uros_pub;
rclc_executor_t mbb_uros_executor;
rcl_timer_t mbb_uros_pub_timer;
geometry_msgs__msg__Twist mbb_uros_sub_allocated_msg;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
static void error_loop() {
  while(1) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

static void mbb_uros_sub_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    // ESP_LOGI(MBB_UROS_LOG_TAG, "%f, %f", msg->linear.x, msg->angular.z);

    // Send the message to the queue
    xQueueSend(mbb_uros_cmd_vel_queue, msg, 0);
}

static void mbb_uros_hw_init(void) 
{
    uROS_Serial.begin(MBB_UROS_UART_BAUDRATE, SERIAL_8N1, MBB_UROS_UART_RX, MBB_UROS_UART_TX);
    while (!uROS_Serial)
    {
        ESP_LOGE(MBB_UROS_LOG_TAG, "Serial port not available");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void mbb_uros_pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    nav_msgs__msg__Odometry odom_msg = {};  // Initialize the local odometry message

    if (pdTRUE == xQueueReceive(mbb_uros_odom_queue, &odom_msg, 0))
    {
        // ESP_LOGI(MBB_UROS_LOG_TAG, "Publishing odometry message...");
        RCSOFTCHECK(rcl_publish(&mbb_uros_pub, &odom_msg, NULL));
    }
}

int64_t mbb_uros_get_epoch_millis()
{
    if (rmw_uros_epoch_synchronized())
    {
        return rmw_uros_epoch_millis();
    }
    else
    {
        ESP_LOGE(MBB_UROS_LOG_TAG,\
            "Cannot get epoch millis. Time is not synchronized with the agent.");
        return -1; 
    }
}

int64_t mbb_uros_get_epoch_nanos()
{
    if (rmw_uros_epoch_synchronized())
    {
        return rmw_uros_epoch_nanos();
    }
    else
    {
        ESP_LOGE(MBB_UROS_LOG_TAG,\
            "Cannot get epoch nanos. Time is not synchronized with the agent.");
        return -1; 
    }
}


void mbb_uros_task_init(void)
{
    esp_log_level_set(MBB_UROS_LOG_TAG, MBB_UROS_LOG_LEVEL);

    mbb_uros_hw_init();

    // Setup the micro-ROS
    set_microros_serial_transports(uROS_Serial);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // ping the agent to test the connection
    ESP_LOGI(MBB_UROS_LOG_TAG, "Checking micro-ROS agent...");
    rmw_ret_t ping_result = rmw_uros_ping_agent(MBB_UROS_PING_AGENT_ATTEMPT_TIMEOUT_MS,\
                                                MBB_UROS_PING_AGENT_ATTMEPT_NUM);
    if (RMW_RET_OK != ping_result)
    {
        ESP_LOGE(MBB_UROS_LOG_TAG,\
            "micro-ROS agent not found. Please check and do reset to try again.");
        error_loop();
    }

    ESP_LOGI(MBB_UROS_LOG_TAG, "micro-ROS agent found. Initializing...");

    // Initialize the micro-ROS
    // create allocator
    mbb_uros_allocator = rcl_get_default_allocator();    
    // create init_options
    RCCHECK(rclc_support_init(&mbb_uros_support,\
                            0, NULL, &mbb_uros_allocator));
    // create node
    RCCHECK(rclc_node_init_default(&mbb_uros_node,\
                                MBB_UROS_NODE_NAME,\
                                MBB_UROS_NODE_NAMESPACE,\
                                &mbb_uros_support));   
    
    // create subscriber
    RCCHECK(rclc_subscription_init_best_effort(&mbb_uros_sub,\
                                            &mbb_uros_node,\
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),\
                                            MBB_UROS_SUB_TOPIC_NAME));

    // create publisher
    RCCHECK(rclc_publisher_init_default(&mbb_uros_pub,\
                                        &mbb_uros_node,\
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),\
                                        MBB_UROS_PUB_TOPIC_NAME));

    // create timer
    
    RCCHECK(rclc_timer_init_default(
        &mbb_uros_pub_timer,\
        &mbb_uros_support,\
        RCL_MS_TO_NS(MBB_UROS_PUB_TIMER_PERIOD_MS),\
        mbb_uros_pub_timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&mbb_uros_executor, &mbb_uros_support.context, 2, &mbb_uros_allocator));

    // add subscription to executor
    RCCHECK(rclc_executor_add_subscription(&mbb_uros_executor,\
                                        &mbb_uros_sub,\
                                        &mbb_uros_sub_allocated_msg,\
                                        &mbb_uros_sub_callback,\
                                        ON_NEW_DATA));
    // add timer to executor
    RCCHECK(rclc_executor_add_timer(&mbb_uros_executor, &mbb_uros_pub_timer));

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // synchronize time with the agent
    if (RMW_RET_OK != rmw_uros_sync_session(MBB_UROS_SYNC_AGENT_TIME_TIMEOUT_MS))
    {
        ESP_LOGE(MBB_UROS_LOG_TAG,\
            "Failed to synchronize time with the agent. Please check and do reset to try again.");
        error_loop();
    }
    
    ESP_LOGI(MBB_UROS_LOG_TAG, "micro-ROS initialized");

    // Create the queues
    mbb_uros_odom_queue = xQueueCreate(MBB_UROS_ODOM_QUEUE_SIZE, sizeof(nav_msgs__msg__Odometry));
    mbb_uros_cmd_vel_queue = xQueueCreate(MBB_UROS_CMD_VEL_QUEUE_SIZE, sizeof(geometry_msgs__msg__Twist));
}


// spin and pub
void mbb_uros_task(void *arg) 
{
    RCSOFTCHECK(rclc_executor_spin(&mbb_uros_executor));
    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Yield control to other tasks for 10 ms
        // RCSOFTCHECK(rclc_executor_spin_some(&mbb_uros_executor, RCL_MS_TO_NS(100)));
    }
}

void mbb_uros_task_start(void)
{
    xTaskCreate(mbb_uros_task,\
                "mbb_uros_task",\
                MBB_UROS_TASK_STACK_SIZE,\
                NULL,\
                MBB_UROS_TASK_PRIORITY,\
                NULL);
}
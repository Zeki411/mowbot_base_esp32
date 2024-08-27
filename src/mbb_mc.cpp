#include <esp32-hal-ledc.h> 
#include <driver/uart.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_intr_alloc.h>

#include <string.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include "mbb_mc.h"
#include "mbb_uros.h"
#include "main.h"

#include <rosidl_runtime_c/string_functions.h>

static QueueHandle_t mbb_mc_uart_rx_queue;
static TaskHandle_t mbb_mc_monitor_task_handle;
static TaskHandle_t mbb_mc_control_task_handle;

static mbb_mc_msg_rx_mode_t mbb_mc_rx_msg_mode = MBB_MC_RX_MSG_RUN_MODE;
static mbb_mc_run_state_t mbb_mc_run_state;

typedef struct {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
} mbb_mc_odom_t;

static mbb_mc_odom_t mbb_mc_odom;
static int64_t mbb_mc_last_odom_time;

// nav_msgs__msg__Odometry mbb_mc_last_odom;

static void mbb_mc_set_pwm_1(int pwm_val)
{
    ledcWrite(MBB_MC_PWM_1_CHANNEL, pwm_val);
}

static void mbb_mc_set_pwm_2(int pwm_val)
{
    ledcWrite(MBB_MC_PWM_2_CHANNEL, pwm_val);
}

static void mbb_mc_process_run_data(mbb_mc_msg_rx_run_data_t *run_data)
{

    // Process run data
    mbb_mc_run_state.current_m1 = (run_data->m1current[0] << 8 | run_data->m1current[1]) * 0.1;
    mbb_mc_run_state.current_m2 = (run_data->m2current[0] << 8 | run_data->m2current[1]) * 0.1;
    mbb_mc_run_state.temp_m1 = (run_data->m1temp[0] << 8 | run_data->m1temp[1]) * 0.1;
    mbb_mc_run_state.temp_m2 = (run_data->m2temp[0] << 8 | run_data->m2temp[1]) * 0.1;
    mbb_mc_run_state.supply_vol = (run_data->supply_vol[0] << 8 | run_data->supply_vol[1]) * 0.1;

    // ESP_LOGI(MBB_MC_LOG_TAG, "M1 Current: %.2f, M2 Current: %.2f, M1 Temp: %.2f, M2 Temp: %.2f, Supply Voltage: %.2f",\
    //         mbb_mc_run_state.current_m1,\
    //         mbb_mc_run_state.current_m2,\
    //         mbb_mc_run_state.temp_m1,\
    //         mbb_mc_run_state.temp_m2,\
    //         mbb_mc_run_state.supply_vol);

    mbb_mc_run_state.m1_rpm = (run_data->m1data[0] << 24 | run_data->m1data[1] << 16 | run_data->m1data[2] << 8 | run_data->m1data[3]);
    mbb_mc_run_state.m2_rpm = (run_data->m2data[0] << 24 | run_data->m2data[1] << 16 | run_data->m2data[2] << 8 | run_data->m2data[3]);

    // ESP_LOGI(MBB_MC_LOG_TAG, "M1 RPM: %d, M2 RPM: %d", mbb_mc_run_state.m1_rpm, mbb_mc_run_state.m2_rpm);

    // Calculate odom

    // get delta time in microseconds
    // int64_t delta_time = esp_timer_get_time() - mbb_mc_odom_prev_time;
    // mbb_mc_odom_prev_time = esp_timer_get_time();
    
    int64_t current_time_ms = mbb_uros_get_epoch_millis();

    // ESP_LOGI(MBB_MC_LOG_TAG, "Odom sec: %d, Odom ns: %d", mbb_mc_last_odom.header.stamp.sec, mbb_mc_last_odom.header.stamp.nanosec);
    // int64_t last_time_ms = (int64_t)mbb_mc_last_odom.header.stamp.sec * 1000 +\
    //             (int64_t)mbb_mc_last_odom.header.stamp.nanosec / 1000000;
    int64_t delta_time_ms = current_time_ms - mbb_mc_last_odom_time;
    if (delta_time_ms < 0)
    {
        ESP_LOGE(MBB_MC_LOG_TAG, "Negative delta time");
        return;
    }
    mbb_mc_last_odom_time = current_time_ms;
    
    // update odom
    double delta_time_s = (double)delta_time_ms / 1000;
    double m1_vel = ( (double)mbb_mc_run_state.m1_rpm / MBB_MC_MOTOR_GEAR_RATIO ) * MBB_MC_WHEEL_RADIUS * 2 * M_PI / 60; // m/s
    double m2_vel = ( (double)mbb_mc_run_state.m2_rpm / MBB_MC_MOTOR_GEAR_RATIO ) * MBB_MC_WHEEL_RADIUS * 2 * M_PI / 60; // m/s

    double delta_s = delta_time_s * (m1_vel + m2_vel) / 2;
    double delta_theta = delta_time_s * (m2_vel - m1_vel) / MBB_MC_DISTANCE_BETWEEN_WHEEL_SIDES;

    mbb_mc_odom.x += delta_s * cos(mbb_mc_odom.yaw + delta_theta / 2); // trapzoidal integration
    mbb_mc_odom.y += delta_s * sin(mbb_mc_odom.yaw + delta_theta / 2); // trapzoidal integration
    mbb_mc_odom.yaw += delta_theta;
    mbb_mc_odom.yaw = fmod(mbb_mc_odom.yaw, 2 * M_PI);
    mbb_mc_odom.pitch = 0;
    mbb_mc_odom.roll = 0;

    ESP_LOGI(MBB_MC_LOG_TAG, "X: %f, Y: %f, Yaw: %f", mbb_mc_odom.x, mbb_mc_odom.y, mbb_mc_odom.yaw);

    // prepare odom message
    nav_msgs__msg__Odometry odom_msg = {};  // Initialize the local odometry message

    // Initialize strings to avoid any garbage data
    rosidl_runtime_c__String__init(&odom_msg.header.frame_id);
    rosidl_runtime_c__String__init(&odom_msg.child_frame_id);
    // Assign values to string fields
    rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, MBB_UROS_ODOM_FRAME_ID);
    rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, MBB_UROS_ODOM_CHILD_FRAME_ID);

    odom_msg.header.stamp.sec = (int32_t)(current_time_ms / 1000); // convert to seconds
    odom_msg.header.stamp.nanosec = (int32_t)((current_time_ms % 1000) * 1000000); // convert to nanoseconds

    odom_msg.pose.pose.position.x = mbb_mc_odom.x;
    odom_msg.pose.pose.position.y = mbb_mc_odom.y;

    // convert yaw to quaternion
    double cy = cos(mbb_mc_odom.yaw * 0.5);
    double sy = sin(mbb_mc_odom.yaw * 0.5);
    double cp = cos(mbb_mc_odom.pitch * 0.5);
    double sp = sin(mbb_mc_odom.pitch * 0.5);
    double cr = cos(mbb_mc_odom.roll * 0.5);
    double sr = sin(mbb_mc_odom.roll * 0.5);

    odom_msg.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr;
    odom_msg.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr;
    odom_msg.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr;
    odom_msg.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr;


    // ESP_LOGI(MBB_MC_LOG_TAG, "pos x: %f, pos y: %f, pos z: %f, ori x: %f, ori y: %f, ori z: %f, ori w: %f",\
    //         odom_msg.pose.pose.position.x,\
    //         odom_msg.pose.pose.position.y,\
    //         odom_msg.pose.pose.position.z,\
    //         odom_msg.pose.pose.orientation.x,\
    //         odom_msg.pose.pose.orientation.y,\
    //         odom_msg.pose.pose.orientation.z,\
    //         odom_msg.pose.pose.orientation.w);


    odom_msg.twist.twist.linear.x = delta_s / delta_time_s; // m/s
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = delta_theta / delta_time_s; // rad/s


    // ESP_LOGI(MBB_MC_LOG_TAG, "lin x: %f, lin y: %f, lin z: %f, ang x: %f, ang y: %f, ang z: %f",\
    //         odom_msg.twist.twist.linear.x,\
    //         odom_msg.twist.twist.linear.y,\
    //         odom_msg.twist.twist.linear.z,\
    //         odom_msg.twist.twist.angular.x,\
    //         odom_msg.twist.twist.angular.y,\
    //         odom_msg.twist.twist.angular.z);

    memset(&odom_msg.pose.covariance, 0, sizeof(odom_msg.pose.covariance));

    // Send odom to uROS, using overwrite to make sure the latest data for the topic is sent
    xQueueOverwrite(mbb_uros_odom_queue, &odom_msg);

}


static void mbb_mc_monitor_task(void *arg)
{
    uart_event_t event;
    uint8_t* data = (uint8_t*) malloc(MBB_MC_UART_RX_BUFF_SIZE);
    while(1) 
    {
        if (xQueueReceive(mbb_mc_uart_rx_queue, (void *)&event, (TickType_t)portMAX_DELAY)) 
        {
            bzero(data, MBB_MC_UART_RX_BUFF_SIZE);
            switch(event.type) 
            {
                case UART_DATA:
                    uart_read_bytes(MBB_MC_UART_NUM, data, event.size, portMAX_DELAY);
                    mbb_mc_msg_header_t header;
                    memcpy(&header, data, MBB_MC_UART_MSG_HEADER_LEN);
                    if (header.header1 == MBB_MC_UART_MSG_RX_H1 &&\
                        header.header2 == MBB_MC_UART_MSG_RX_H2 &&\
                        header.cmd == MBB_MC_UART_MSG_RX_CMD) 
                    {
                        if (mbb_mc_rx_msg_mode == MBB_MC_RX_MSG_RUN_MODE) 
                        {
                            mbb_mc_msg_rx_run_data_t decoded_data;
                            memcpy(&decoded_data, data + MBB_MC_UART_MSG_HEADER_LEN, MBB_MC_UART_MSG_RX_RUN_DATA_LEN);
                            mbb_mc_process_run_data(&decoded_data);
                        }

                        if (mbb_mc_rx_msg_mode == MBB_MC_RX_MSG_SET_MODE)
                        {
                            //TODO: Implement set mode
                        }
                        
                    }

            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    free(data);
}

/*
 * Function to map the velocity to the corresponding PWM value.
 *
 */
static int mbb_mc_vel_to_pwm(double v, double max_rpm)
{
    if (v > MBB_MC_VELOCITY_LIMIT) {
        v = MBB_MC_VELOCITY_LIMIT;
    } else if (v < (-1 * MBB_MC_VELOCITY_LIMIT)) {
        v = (-1 * MBB_MC_VELOCITY_LIMIT);
    }

    // Map the limit RPM to the corresponding PWM value
    int pwm_limit_pos = MAP_RANGE(MBB_MC_MOTOR_RPM_LIMIT, -1 * max_rpm, max_rpm, MBB_MC_PWM_VAL_RPM_MAX_NEG, MBB_MC_PWM_VAL_RPM_MAX_POS);
    
    int pwm_limit_neg = MAP_RANGE(-1 * MBB_MC_MOTOR_RPM_LIMIT, -1 * max_rpm, max_rpm, MBB_MC_PWM_VAL_RPM_MAX_NEG, MBB_MC_PWM_VAL_RPM_MAX_POS);

    // Map the velocity to the corresponding PWM value in the range of the limit PWM values
    int pwm_value = MAP_RANGE(v, -1 * MBB_MC_VELOCITY_LIMIT, MBB_MC_VELOCITY_LIMIT, pwm_limit_neg, pwm_limit_pos);

    return pwm_value;
}

static void mbb_mc_process_cmd_vel(geometry_msgs__msg__Twist *cmd_vel_data)
{
    // Process cmd_vel
    double v_left = cmd_vel_data->linear.x - (cmd_vel_data->angular.z * MBB_MC_DISTANCE_BETWEEN_WHEEL_SIDES / 2);
    double v_right = cmd_vel_data->linear.x + (cmd_vel_data->angular.z * MBB_MC_DISTANCE_BETWEEN_WHEEL_SIDES / 2);

    int pwm_left = mbb_mc_vel_to_pwm(v_left, MBB_MC_MOTOR_1_RPM_MEASURE_MAX);
    int pwm_right = mbb_mc_vel_to_pwm(v_right, MBB_MC_MOTOR_2_RPM_MEASURE_MAX);

    // ESP_LOGI(MBB_MC_LOG_TAG, "PWM Left: %d, PWM Right: %d", pwm_left, pwm_right);

    mbb_mc_set_pwm_1(pwm_left);
    mbb_mc_set_pwm_2(pwm_right);
}

static void mbb_mc_control_task(void *arg)
{
    geometry_msgs__msg__Twist cmd_vel_data;
    while(1) 
    {
        if(xQueueReceive(mbb_uros_cmd_vel_queue, &cmd_vel_data, portMAX_DELAY))
        {
            // Process cmd_vel
            // ESP_LOGI(MBB_MC_LOG_TAG, "Received cmd_vel: %f, %f", cmd_vel_data.linear.x, cmd_vel_data.angular.z);
            mbb_mc_process_cmd_vel(&cmd_vel_data);
            
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void mbb_mc_hw_init(void)
{
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = MBB_MC_UART_BAUDRATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(MBB_MC_UART_NUM, &uart_config));

    // Set UART pins(TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(MBB_MC_UART_NUM,\
                                MBB_MC_UART_TX_IO,\
                                MBB_MC_UART_RX_IO,\
                                UART_PIN_NO_CHANGE,\
                                UART_PIN_NO_CHANGE));

    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(MBB_MC_UART_NUM,\
                                        MBB_MC_UART_RX_BUFF_SIZE,\
                                        MBB_MC_UART_TX_BUFF_SIZE,\
                                        MBB_MC_UART_QUEUE_SIZE,\
                                        &mbb_mc_uart_rx_queue,\
                                        0));    

    // Config PWM
    ledcSetup(MBB_MC_PWM_1_CHANNEL, MBB_MC_PWM_FREQ, MBB_MC_PWM_RESOLUTION);
    ledcSetup(MBB_MC_PWM_2_CHANNEL, MBB_MC_PWM_FREQ, MBB_MC_PWM_RESOLUTION);

    // Attach the channel to the GPIO to be controlled
    ledcAttachPin(MBB_MC_PWM_1_IO, MBB_MC_PWM_1_CHANNEL);
    ledcAttachPin(MBB_MC_PWM_2_IO, MBB_MC_PWM_2_CHANNEL);

    // Set the PWM duty cycle to the default value
    ledcWrite(MBB_MC_PWM_1_CHANNEL, MBB_MC_PWM_VAL_RPM_ZERO);
    ledcWrite(MBB_MC_PWM_2_CHANNEL, MBB_MC_PWM_VAL_RPM_ZERO);

    vTaskDelay(500 / portTICK_PERIOD_MS);
    // ledcWrite(MBB_MC_PWM_1_CHANNEL, MBB_MC_PWM_VAL_RPM_MAX_NEG);
    // ledcWrite(MBB_MC_PWM_2_CHANNEL, MBB_MC_PWM_VAL_RPM_MAX_POS);
}

void mbb_mc_task_init(void)
{
    mbb_mc_hw_init();

    esp_log_level_set(MBB_MC_LOG_TAG, MBB_MC_LOG_LEVEL);

    ESP_LOGI(MBB_MC_LOG_TAG, "MBB MC initialized");
}

void mbb_mc_task_start(void)
{
    xTaskCreate(mbb_mc_monitor_task,\
                "mbb_mc_monitor_task",\
                MBB_MC_MONITOR_TASK_STACK_SIZE,\
                NULL,\
                MBB_UROS_TASK_PRIORITY,\
                &mbb_mc_monitor_task_handle);
    
    xTaskCreate(mbb_mc_control_task,\
                "mbb_mc_control_task",\
                MBB_MC_CONTROL_TASK_STACK_SIZE,\
                NULL,\
                MBB_UROS_TASK_PRIORITY,\
                &mbb_mc_control_task_handle);
}

void mbb_mc_task_stop(void)
{
    vTaskDelete(mbb_mc_monitor_task_handle);
    vTaskDelete(mbb_mc_control_task_handle);
}

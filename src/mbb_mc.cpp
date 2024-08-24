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

static QueueHandle_t mbb_mc_uart_rx_queue;
static TaskHandle_t mbb_mc_monitor_task_handle;
static TaskHandle_t mbb_mc_control_task_handle;

static mbb_mc_msg_rx_mode_t mbb_mc_rx_msg_mode = MBB_MC_RX_MSG_RUN_MODE;
static mbb_mc_run_state_t mbb_mc_run_state;

static void mbb_mc_process_run_data(mbb_mc_msg_rx_run_data_t *run_data)
{
    // Process run data
    mbb_mc_run_state.current_m1 = (run_data->m1current[0] << 8 | run_data->m1current[1]) * 0.1;
    mbb_mc_run_state.current_m2 = (run_data->m2current[0] << 8 | run_data->m2current[1]) * 0.1;
    mbb_mc_run_state.temp_m1 = (run_data->m1temp[0] << 8 | run_data->m1temp[1]) * 0.1;
    mbb_mc_run_state.temp_m2 = (run_data->m2temp[0] << 8 | run_data->m2temp[1]) * 0.1;
    mbb_mc_run_state.supply_vol = (run_data->supply_vol[0] << 8 | run_data->supply_vol[1]) * 0.1;

    ESP_LOGI(MBB_MC_LOG_TAG, "M1 Current: %.2f, M2 Current: %.2f, M1 Temp: %.2f, M2 Temp: %.2f, Supply Voltage: %.2f",\
            mbb_mc_run_state.current_m1,\
            mbb_mc_run_state.current_m2,\
            mbb_mc_run_state.temp_m1,\
            mbb_mc_run_state.temp_m2,\
            mbb_mc_run_state.supply_vol);

    // Calculate odom

    // Send odom to uROS
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
        // vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    free(data);
}

static void mbb_mc_control_task(void *arg)
{
    geometry_msgs__msg__Twist cmd_vel_data;
    while(1) 
    {
        if(xQueueReceive(mbb_uros_cmd_vel_queue, &cmd_vel_data, portMAX_DELAY))
        {
            // Process cmd_vel
            ESP_LOGI(MBB_MC_LOG_TAG, "Received cmd_vel: %f, %f", cmd_vel_data.linear.x, cmd_vel_data.angular.z);
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

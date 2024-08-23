#include "mbb_motor_controller.h"

QueueHandle_t MBBMotorController::uart_rx_queue = nullptr;
MBBMotorController_RxMsgMode_t MBBMotorController::rx_msg_mode = MBB_MC_RX_MSG_RUN_MODE;
MBBMotorController_MsgRxRunData_t MBBMotorController::last_rx_msg_run_data = {0};

// static void uart_rx_task(void *arg) {
//     ESP_LOGI("uart_rx_task", "Task started");
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
// }


MBBMotorController::MBBMotorController() {
    // Constructor code here
}

MBBMotorController::~MBBMotorController() {
    // Destructor code here
}

void MBBMotorController::init_hw_uart(void) {
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
    ESP_ERROR_CHECK(uart_set_pin(MBB_MC_UART_NUM, MBB_MC_UART_TX_IO, MBB_MC_UART_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(MBB_MC_UART_NUM, MBB_MC_UART_RX_BUFF_SIZE, 0, 20, &uart_rx_queue, 0));    
}

void MBBMotorController::uart_rx_task(void *arg) {
    char *TAG = "uart_rx_task";
    uart_event_t event;
    uint8_t* data = (uint8_t*) malloc(MBB_MC_UART_RX_BUFF_SIZE);
    while (1)
    {
        if (xQueueReceive(uart_rx_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(data, MBB_MC_UART_RX_BUFF_SIZE);
            // ESP_LOGI(TAG, "uart[%d] event:", MBB_UART_NUM);
            switch(event.type) {
                case UART_DATA:
                    uart_read_bytes(MBB_MC_UART_NUM, data, event.size, portMAX_DELAY);
                    // ESP_LOGI(TAG, "Read %d bytes: %s", event.size, data);

                    MBBMotorController_MsgHeader_t header;
                    memcpy(&header, data, MBB_MC_UART_MSG_HEADER_LEN);

                    ESP_LOGI(TAG, "header1: 0x%02X, header2: 0x%02X, data_len: %d, cmd: 0x%02X", \
                        header.header1, header.header2, header.data_len, header.cmd);
                    
                    if (header.header1 == MBB_MC_UART_MSG_RX_H1 && \
                        header.header2 == MBB_MC_UART_MSG_RX_H2 && \
                        header.cmd == MBB_MC_UART_MSG_RX_CMD
                    ) {
                        
                        ESP_LOGI(TAG, "Received a valid message");

                        if (rx_msg_mode ==MBB_MC_RX_MSG_RUN_MODE) {

                            MBBMotorController_MsgRxRunData_t rx_msg_temp;

                            // Copy the data to the struct
                            memcpy(&rx_msg_temp, data + MBB_MC_UART_MSG_HEADER_LEN, MBB_MC_UART_MSG_RX_RUN_DATA_LEN);

                            // Reorder the bytes
                            last_rx_msg_run_data.fix1 = rx_msg_temp.fix1;
                            last_rx_msg_run_data.fix2 = rx_msg_temp.fix2;
                            last_rx_msg_run_data.m1data = SWAP_MSB_LSB_32(rx_msg_temp.m1data);
                            last_rx_msg_run_data.m2data = SWAP_MSB_LSB_32(rx_msg_temp.m2data);
                            last_rx_msg_run_data.m1current = SWAP_MSB_LSB_16(rx_msg_temp.m1current);
                            last_rx_msg_run_data.m2current = SWAP_MSB_LSB_16(rx_msg_temp.m2current);
                            last_rx_msg_run_data.m1temp = SWAP_MSB_LSB_16(rx_msg_temp.m1temp);
                            last_rx_msg_run_data.m2temp = SWAP_MSB_LSB_16(rx_msg_temp.m2temp);
                            last_rx_msg_run_data.supply_vol = SWAP_MSB_LSB_16(rx_msg_temp.supply_vol);
                            last_rx_msg_run_data.m1status = SWAP_MSB_LSB_16(rx_msg_temp.m1status);
                            last_rx_msg_run_data.m2status = SWAP_MSB_LSB_16(rx_msg_temp.m2status);
                            last_rx_msg_run_data.unknown1 = rx_msg_temp.unknown1;
                            last_rx_msg_run_data.unknown2 = rx_msg_temp.unknown2;

                            ESP_LOGI(TAG, "fix1: 0x%02X, fix2: 0x%02X, m1data: 0x%08X, m2data: 0x%08X, m1current: 0x%04X, m2current: 0x%04X, m1temp: 0x%04X, m2temp: 0x%04X, supply_vol: 0x%04X, m1status: 0x%04X, m2status: 0x%04X", \
                                last_rx_msg_run_data.fix1, last_rx_msg_run_data.fix2, \
                                last_rx_msg_run_data.m1data, last_rx_msg_run_data.m2data, \
                                last_rx_msg_run_data.m1current, last_rx_msg_run_data.m2current, 
                                last_rx_msg_run_data.m1temp, last_rx_msg_run_data.m2temp, \
                                last_rx_msg_run_data.supply_vol, \
                                last_rx_msg_run_data.m1status, last_rx_msg_run_data.m2status);
                        } 
                        else if (rx_msg_mode == MBB_MC_RX_MSG_SET_MODE) {
                            //TODO: Implement the set mode message
                        }

                    }

                    break;
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "UART_FIFO_OVF");
                    uart_flush_input(MBB_MC_UART_NUM);
                    xQueueReset(uart_rx_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "UART_BUFFER_FULL");
                    uart_flush_input(MBB_MC_UART_NUM);
                    xQueueReset(uart_rx_queue);
                    break;
                case UART_BREAK:
                    ESP_LOGI(TAG, "UART_BREAK");
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "UART_PARITY_ERR");
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "UART_FRAME_ERR");
                    break;
                case UART_PATTERN_DET:
                    ESP_LOGI(TAG, "UART_PATTERN_DET");
                    break;
                default:
                    ESP_LOGI(TAG, "Unknown event");
                    break;
            }

        }
    }
    free(data);
}

void MBBMotorController::init() {
    init_hw_uart();

    xTaskCreate(
        uart_rx_task,
        "uart_rx_task", 
        1024 * 2, 
        NULL, 
        10, 
        &uart_rx_task_handle);
    

}

void MBBMotorController::start() {

}





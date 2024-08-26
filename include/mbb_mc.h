#ifndef __MBB_MC_H__
#define __MBB_MC_H__

#include <stdint.h>

#define MAP_RANGE(x, xmin, xmax, ymin, ymax) \
    ((ymin) + (((x) - (xmin)) * ((ymax) - (ymin)) / ((xmax) - (xmin))))

// Logging configuration
#define MBB_MC_LOG_TAG "mbb-mc"
#define MBB_MC_LOG_LEVEL ESP_LOG_INFO

// UART configuration
#define MBB_MC_UART_NUM UART_NUM_2
#define MBB_MC_UART_RX_IO 16
#define MBB_MC_UART_TX_IO 17
#define MBB_MC_UART_BAUDRATE 19200
#define MBB_MC_UART_RX_BUFF_SIZE 1024
#define MBB_MC_UART_TX_BUFF_SIZE 0
#define MBB_MC_UART_QUEUE_SIZE 10

// UART RX message info
#define MBB_MC_UART_MSG_HEADER_LEN 4

#define MBB_MC_UART_MSG_RX_H1 0x5A
#define MBB_MC_UART_MSG_RX_H2 0xA5
#define MBB_MC_UART_MSG_RX_CMD 0x82

#define MBB_MC_UART_MSG_RX_RUN_DATA_LEN 26
#define MBB_MC_UART_MSG_RX_RUN_DATA_FIX1 0x11
#define MBB_MC_UART_MSG_RX_RUN_DATA_FIX2 0x00

// PWM configuration
#define MBB_MC_PWM_1_IO 19
#define MBB_MC_PWM_2_IO 18
#define MBB_MC_PWM_1_CHANNEL 0
#define MBB_MC_PWM_2_CHANNEL 1
#define MBB_MC_PWM_FREQ 50
#define MBB_MC_PWM_RESOLUTION 16

//TODO: Need to measure and replace with actual values
#define MBB_MC_WHEEL_RADIUS 0.125 // meters
#define MBB_MC_DISTANCE_BETWEEN_WHEEL_SIDES 0.5 // meters
#define MBB_MC_MOTOR_GEAR_RATIO 36 // 1:36

constexpr int MBB_MC_PWM_VAL_RPM_ZERO = ((1 << MBB_MC_PWM_RESOLUTION) - 1) * 7.5  / 100; // 7.5% duty cycle ~ 1.5ms pulse width at 50Hz
// constexpr int MBB_MC_PWM_VAL_RPM_MAX_POS = ((1 << MBB_MC_PWM_RESOLUTION) - 1) * 5.25 / 100; // 5.25% duty cycle ~ 1.05ms pulse width at 50Hz
// constexpr int MBB_MC_PWM_VAL_RPM_MAX_NEG = ((1 << MBB_MC_PWM_RESOLUTION) - 1) * 9.75 / 100; // 9.75% duty cycle ~ 1.95ms pulse width at 50Hz

constexpr int MBB_MC_PWM_VAL_RPM_MAX_POS = ((1 << MBB_MC_PWM_RESOLUTION) - 1) * 5/ 100; // 5% duty cycle ~ 1.0ms pulse width at 50Hz
constexpr int MBB_MC_PWM_VAL_RPM_MAX_NEG = ((1 << MBB_MC_PWM_RESOLUTION) - 1) * 10 / 100; // 10% duty cycle ~ 2.0ms pulse width at 50Hz

/*
 * These value are measured from the motor corresponding to the max/min PWM values, max is 3000 RPM (in specs).
 * They may be needed to be calibrated again.
 */
#define MBB_MC_MOTOR_1_RPM_MEASURE_MAX 2745 // 2745 RPM at 5% duty cycle, -2745 RPM at 10% duty cycle
#define MBB_MC_MOTOR_2_RPM_MEASURE_MAX 2870 // 2870 RPM at 5% duty cycle, -2870 RPM at 10% duty cycle

/*
 * This value is the user chosen limits to sync 2 motors at same max RPM.
 * It need to be smaller than the smallest max RPM value of the 2 motors.
 */
#define MBB_MC_MOTOR_RPM_LIMIT 2700 // RPM
#define MBB_MC_VELOCITY_LIMIT ((2 * (MBB_MC_MOTOR_RPM_LIMIT / MBB_MC_MOTOR_GEAR_RATIO) * 3.14159 * MBB_MC_WHEEL_RADIUS) / 60) // m/s









// MBB MC task
#define MBB_MC_MONITOR_TASK_STACK_SIZE 4096
#define MBB_MC_MONITOR_TASK_PRIORITY 5
#define MBB_MC_CONTROL_TASK_STACK_SIZE 4096
#define MBB_MC_CONTROL_TASK_PRIORITY 5



typedef enum {
    MBB_MC_RX_MSG_RUN_MODE = 0,
    MBB_MC_RX_MSG_SET_MODE = 1,
} mbb_mc_msg_rx_mode_t;

typedef union{
    struct {
        uint8_t header1 : 8;
        uint8_t header2 : 8;
        uint8_t data_len : 8;
        uint8_t cmd: 8;
    }__attribute__((packed));  // Ensures no padding between fields
    uint8_t raw[MBB_MC_UART_MSG_HEADER_LEN];
} mbb_mc_msg_header_t;

typedef union{
    struct{
        uint8_t current_overload: 1; 
        uint8_t load_abnomal: 1;
        uint8_t temperature_protection: 1;
        uint8_t voltage_too_high: 1;
        uint8_t voltage_too_low: 1;
        uint8_t blocking_protection: 1;
        uint8_t hall_signal_abnormal: 1;
        uint8_t abnomal: 1;
        uint8_t reserved: 8;
    }__attribute__((packed));  // Ensures no padding between fields

    uint8_t raw[2];
} mbb_mc_msg_rx_motor_status_t;

typedef union{
    struct{
        // data
        uint8_t fix1: 8;
        uint8_t fix2: 8;
        uint8_t m1data[4];
        uint8_t m2data[4];
        uint8_t m1current[2];
        uint8_t m2current[2];
        uint8_t m1temp[2];
        uint8_t m2temp[2];
        uint8_t supply_vol[2];
        uint8_t m1status[2];
        uint8_t m2status[2];

        // unknown
        uint8_t unknown1: 8;
        uint8_t unknown2: 8;
    }__attribute__((packed));  // Ensures no padding between fields

    uint8_t raw[MBB_MC_UART_MSG_RX_RUN_DATA_LEN];
} mbb_mc_msg_rx_run_data_t;

typedef struct{
    double m1_rpm;
    double m2_rpm;
    double current_m1;
    double current_m2;
    double temp_m1;
    double temp_m2;
    double supply_vol;
    mbb_mc_msg_rx_motor_status_t status_m1;
    mbb_mc_msg_rx_motor_status_t status_m2;
} mbb_mc_run_state_t;





extern void mbb_mc_task_init(void);
extern void mbb_mc_task_start(void);
extern void mbb_mc_task_stop(void);

#endif // __MBB_MC_H__  
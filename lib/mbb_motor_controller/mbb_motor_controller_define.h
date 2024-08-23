#ifndef __MBB_MOTOR_CONTROLLER_DEFINE_H__
#define __MBB_MOTOR_CONTROLLER_DEFINE_H__

#include <Arduino.h>

// Macro to swap bytes in a 16-bit value
#define SWAP_MSB_LSB_16(val) (((val) >> 8) | ((val) << 8))

// Macro to swap bytes in a 32-bit value
#define SWAP_MSB_LSB_32(val) ((((val) >> 24) & 0x000000FF) | \
                            (((val) >> 8) & 0x0000FF00)  | \
                            (((val) << 8) & 0x00FF0000)  | \
                            (((val) << 24) & 0xFF000000))

// UART configuration
#define MBB_MC_UART_RX_BUFF_SIZE 1024
#define MBB_MC_UART_NUM UART_NUM_2
#define MBB_MC_UART_RX_IO 16
#define MBB_MC_UART_TX_IO 17
#define MBB_MC_UART_BAUDRATE 19200

// UART RX message info
#define MBB_MC_UART_MSG_HEADER_LEN 4

#define MBB_MC_UART_MSG_RX_H1 0x5A
#define MBB_MC_UART_MSG_RX_H2 0xA5
#define MBB_MC_UART_MSG_RX_CMD 0x82

#define MBB_MC_UART_MSG_RX_RUN_DATA_LEN 26
#define MBB_MC_UART_MSG_RX_RUN_DATA_FIX1 0x11
#define MBB_MC_UART_MSG_RX_RUN_DATA_FIX2 0x00


// PWM configuration
#define MBB_MC_PWM_1_IO 18
#define MBB_MC_PWM_2_IO 19
#define MBB_MC_PWM_1_CHANNEL 0
#define MBB_MC_PWM_2_CHANNEL 1
#define MBB_MC_PWM_FREQ 50
#define MBB_MC_PWM_RESOLUTION 16

constexpr int MBB_MC_PWM_VAL_SPEED_ZERO = ((1 << MBB_MC_PWM_RESOLUTION) - 1) * 7.5  / 100; // 7.5% duty cycle ~ 1.5ms pulse width at 50Hz
constexpr int MBB_MC_PWM_VAL_SPEED_MAX_NEG = ((1 << MBB_MC_PWM_RESOLUTION) - 1) * 5.25 / 100; // 5.25% duty cycle ~ 1.05ms pulse width at 50Hz
constexpr int MBB_MC_PWM_VAL_SPEED_MAX_POS = ((1 << MBB_MC_PWM_RESOLUTION) - 1) * 9.75 / 100; // 9.75% duty cycle ~ 1.95ms pulse width at 50Hz


typedef enum {
    MBB_MC_RX_MSG_RUN_MODE = 0,
    MBB_MC_RX_MSG_SET_MODE = 1,
} MBBMotorController_RxMsgMode_t;

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
} MBBMotorController_MotorStatus_t;

typedef union{
    struct {
        uint8_t header1 : 8;
        uint8_t header2 : 8;
        uint8_t data_len : 8;
        uint8_t cmd: 8;
    }__attribute__((packed));  // Ensures no padding between fields
    uint8_t raw[MBB_MC_UART_MSG_HEADER_LEN];
} MBBMotorController_MsgHeader_t;

typedef union{
    struct{
        // data
        uint8_t fix1: 8;
        uint8_t fix2: 8;
        int32_t m1data: 32;
        int32_t m2data: 32;
        uint16_t m1current: 16;
        uint16_t m2current: 16;
        uint16_t m1temp: 16;
        uint16_t m2temp: 16;
        uint16_t supply_vol: 16;
        uint16_t m1status : 16;
        uint16_t m2status : 16;

        // unknown
        uint8_t unknown1: 8;
        uint8_t unknown2: 8;
    }__attribute__((packed));  // Ensures no padding between fields

    uint8_t raw[MBB_MC_UART_MSG_RX_RUN_DATA_LEN];
} MBBMotorController_MsgRxRunData_t;

/* -------------- */

typedef struct {
    //TODO

} MBBMotorController_RunningStatus_t;

#endif // __MBB_MOTOR_CONTROLLER_DEFINE_H__
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>



/* ==================== DEFINES ==================== */
#define CAN_ID 0x01                 // Standard ID for AK80-8 actuator
#define CAN_DLC 8                   // Data Length Code (8 bytes)
#define AK80_POLE_PAIRS 21          // AK80-8 motor pole pairs


#define CURRENT_LIMIT 4.0f  // Set below power supply max (10A)
#define SAFE_ACCELERATION 2.0f  // rad/s²
#define DEFAULT_KP 50.0f
#define DEFAULT_KD 1.0f


// Special Commands for MIT Mode
#define CMD_ENTER_MOTOR_MODE 0xFC
#define CMD_EXIT_MOTOR_MODE  0xFD
#define CMD_SET_ZERO         0xFE
#define CMD_DATA_FILL        0xFF


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    float position;     // Motor position in radians
    float velocity;     // Motor velocity in rad/s
    float torque;       // Motor torque in N.m
    float kp;           // Position gain (0-500)
    float kd;           // Velocity gain (0-5)
    uint8_t motor_id;   // Motor CAN ID
    float temperature;  // Motor temperature in °C
    uint8_t error_code; // Error status
} AK80_MIT_Data_t;

/* ==================== ENUMS & STRUCTS ==================== */
typedef enum {
    CAN_TX_SUCCESS = 0,
    CAN_TX_ERROR_QUEUE_FULL,
    CAN_TX_ERROR_TIMEOUT,
    CAN_TX_ERROR_HAL_FAIL
} can_tx_result_t;

// Add this new structure for position control
typedef struct {
    float target_angle_rad;      // Target position in radians
    float current_angle_rad;     // Current position in radians  
    float max_velocity;          // Maximum velocity limit (rad/s)
    float kp;                   // Position gain
    float kd;                   // Derivative gain
    float prev_error;           // Previous error for derivative
    uint8_t control_enabled;    // Position control enable flag
    uint32_t last_control_time; // For control loop timing
} PositionController_t;


typedef struct {
    uint8_t data[8];  // CAN message payload (8 bytes)
} CANMessage;


// MIT Mode Special Commands
#define MIT_MODE_ENTER      0xFC
#define MIT_MODE_EXIT       0xFD
#define MIT_MODE_ZERO_POS   0xFE

// AK80-8 Parameter Limits
#define AK80_P_MIN      -12.5f
#define AK80_P_MAX       12.5f
#define AK80_V_MIN      -37.5f
#define AK80_V_MAX       37.5f
#define AK80_T_MIN      -32.0f
#define AK80_T_MAX       32.0f
#define AK80_KP_MIN      0.0f
#define AK80_KP_MAX    500.0f
#define AK80_KD_MIN      0.0f
#define AK80_KD_MAX      5.0f
#define AK80_I_MIN     -40.0f
#define AK80_I_MAX      40.0f



/* ==================== FUNCTION PROTOTYPES ==================== */
void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_USART1_UART_Init(void);
void CAN_Config(void);
void Error_Handler(void);

// CAN transmission functions
can_tx_result_t TX_CAN_Extended(uint8_t* data, uint8_t dlc, uint32_t can_id, bool use_extended_id, uint32_t timeout_ms);
void TX_CAN(uint32_t can_id, uint8_t *data, bool ext_id);

// Command processing
void process_command(char* cmd);
void handle_mit_mode_command(char* cmd, float position, float speed);

// Motor control functions
void send_position_speed_command(uint8_t controller_id, float pos_deg, int16_t spd, int16_t acc);
void pack_cmd(CANMessage *msg, float p_des, float v_des, float kp, float kd, float t_ff);
void unpack_reply(CANMessage msg);
void update_position_control(void);

// Utility functions
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float calculate_velocity_command(void);

// Callback functions
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);


void Error_Handler(void);









#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

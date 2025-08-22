/**
******************************************************************************
* @file : main.c
* @brief : STM32 Blue Pill CAN Communication with AK80-8 Actuator
* Supports MIT Mode with Smooth Position Control
******************************************************************************
*/

#include "main.h"

/* ==================== DEFINES ==================== */
#define CAN_ID 0x01 // Standard ID for AK80-8 actuator
#define CAN_DLC 8 // Data Length Code (8 bytes)
#define AK80_POLE_PAIRS 21 // AK80-8 motor pole pairs
#define CURRENT_LIMIT 4.0f // Set below power supply max (10A)
#define SAFE_ACCELERATION 2.0f // rad/s²
#define DEFAULT_KP 50.0f
#define DEFAULT_KD 1.0f

// Special Commands for MIT Mode
#define CMD_ENTER_MOTOR_MODE 0xFC
#define CMD_EXIT_MOTOR_MODE 0xFD
#define CMD_SET_ZERO 0xFE
#define CMD_DATA_FILL 0xFF

/* ==================== GLOBAL VARIABLES ==================== */
UART_HandleTypeDef huart1;
CAN_HandleTypeDef hcan;
uint8_t rx_data;
CANMessage can_msg;

// Motor state variables
float current_position = 0.0f;
float current_velocity = 0.0f;
float current_torque = 0.0f;
float current_temp = 0.0f;
uint8_t motor_mode = 0; // 0 = off, 1 = MIT mode on

// Position controller global variable
PositionController_t pos_ctrl = {
    .target_angle_rad = 0.0f,
    .current_angle_rad = 0.0f,
    .max_velocity = 3.0f,        // Start with conservative 3 rad/s
    .kp = 10.0f,                 // Position gain
    .kd = 0.8f,                  // Derivative gain
    .prev_error = 0.0f,
    .control_enabled = 0,
    .last_control_time = 0
};

// MIT protocol limits
const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float V_MIN = -37.5f;
const float V_MAX = 37.5f;
const float T_MIN = -32.0f;
const float T_MAX = 32.0f;
const float Kp_MIN = 0.0f;
const float Kp_MAX = 500.0f;
const float Kd_MIN = 0.0f;
const float Kd_MAX = 5.0f;
const float I_MAX = 40.0f;
const float I_MIN = -40.0f;

/* ==================== FUNCTION PROTOTYPES ==================== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void CAN_Config(void);
void Error_Handler(void);

// Command processing
void process_command(char* cmd);
void handle_mit_mode_command(char* cmd, float position, float speed);

// Utility functions
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

// Callback functions
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

/* ==================== UTILITY FUNCTIONS ==================== */
/**
* @brief Append 32-bit integer to buffer in big-endian order
*/
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = (number >> 24) & 0xFF;
    buffer[(*index)++] = (number >> 16) & 0xFF;
    buffer[(*index)++] = (number >> 8) & 0xFF;
    buffer[(*index)++] = number & 0xFF;
}

/**
* @brief Append 16-bit integer to buffer in big-endian order
*/
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = (number >> 8) & 0xFF;
    buffer[(*index)++] = number & 0xFF;
}

/**
* @brief Convert float to unsigned integer with specified bit width
*/
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (int)((x - x_min) * ((float)((1 << bits) - 1) / span));
}

/**
* @brief Convert unsigned integer to float within specified range
*/
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* ==================== CAN TRANSMISSION FUNCTIONS ==================== */
/**
* @brief Transmit standard CAN message (for MIT mode)
* @param can_id: CAN identifier
* @param data: Pointer to 8-byte data array
* @param ext_id: true for extended ID, false for standard
*/
void TX_CAN(uint32_t can_id, uint8_t *data, bool ext_id) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    
    TxHeader.DLC = CAN_DLC;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    if (ext_id) {
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.ExtId = can_id;
    } else {
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.StdId = (uint16_t)can_id;
    }
    
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
        Error_Handler();
    }
    
    // Wait for transmission with timeout
    uint32_t timeout = HAL_GetTick() + 50;
    while(HAL_CAN_IsTxMessagePending(&hcan, TxMailbox)) {
        if (HAL_GetTick() > timeout) {
            return;
        }
    }
}

/* ==================== MOTOR CONTROL FUNCTIONS ==================== */
/**
* @brief Pack MIT protocol command into CAN message
*/
void pack_cmd(CANMessage *msg, float p_des, float v_des, float kp, float kd, float t_ff) {
    // Clamp values to safe limits
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
    kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    
    // Convert to scaled integers
    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
    int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    
    // Pack into CAN message according to MIT protocol
    msg->data[0] = p_int >> 8;
    msg->data[1] = p_int & 0xFF;
    msg->data[2] = v_int >> 4;
    msg->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg->data[4] = kp_int & 0xFF;
    msg->data[5] = kd_int >> 4;
    msg->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg->data[7] = t_int & 0xFF;
}

/**
* @brief Unpack motor response (MIT protocol)
*/
void unpack_reply(CANMessage msg) {
    int id = msg.data[0];
    int p_int = (msg.data[1] << 8) | msg.data[2];
    int v_int = (msg.data[3] << 4) | (msg.data[4] >> 4);
    int i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5];
    int T_int = msg.data[6];

    // Convert to engineering units
    current_position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    current_velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
    current_torque = uint_to_float(i_int, I_MIN, I_MAX, 12);
    current_temp = (float)T_int;

    // Print status
    char buffer[120];
    snprintf(buffer, sizeof(buffer),
             "Motor Response - ID:%d, Pos:%.3f rad, Vel:%.2f rad/s, Curr:%.2f A, Temp:%.1f°C\r\n",
             id, current_position, current_velocity, current_torque, current_temp);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/* ==================== POSITION CONTROL FUNCTIONS ==================== */
/**
 * @brief Calculate velocity command using PD controller
 * @return Velocity command in rad/s
 */
float calculate_velocity_command(void) {
    if (!pos_ctrl.control_enabled) return 0.0f;
    
    // Calculate position error
    float error = pos_ctrl.target_angle_rad - pos_ctrl.current_angle_rad;
    
    // Handle angle wrapping for continuous rotation
    while (error > 3.14159f) error -= 2.0f * 3.14159f;
    while (error < -3.14159f) error += 2.0f * 3.14159f;
    
    // Calculate derivative term (error rate)
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pos_ctrl.last_control_time) / 1000.0f;
    
    float error_dot = 0.0f;
    if (dt > 0.001f) { // Avoid division by zero
        error_dot = (error - pos_ctrl.prev_error) / dt;
        pos_ctrl.prev_error = error;
        pos_ctrl.last_control_time = current_time;
    }
    
    // PD controller calculation
    float velocity_cmd = pos_ctrl.kp * error + pos_ctrl.kd * error_dot;
    
    // Apply velocity limits for smooth motion
    if (velocity_cmd > pos_ctrl.max_velocity) {
        velocity_cmd = pos_ctrl.max_velocity;
    } else if (velocity_cmd < -pos_ctrl.max_velocity) {
        velocity_cmd = -pos_ctrl.max_velocity;
    }
    
    return velocity_cmd;
}

/**
 * @brief Update position control loop - call this regularly from main()
 */
void update_position_control(void) {
    if (!pos_ctrl.control_enabled || !motor_mode) return;
    
    // Update current position from motor feedback
    pos_ctrl.current_angle_rad = current_position;
    
    // Calculate velocity command
    float velocity_cmd = calculate_velocity_command();
    
    // Send command to motor using existing pack_cmd function
    pack_cmd(&can_msg, 
             pos_ctrl.target_angle_rad,  // Position setpoint
             velocity_cmd,               // Calculated velocity
             5.0f,                      // Low position gain (we're doing external control)
             pos_ctrl.kd,               // Use our derivative gain
             0.0f);                     // No feedforward torque
    
    TX_CAN(CAN_ID, can_msg.data, false);
}

/* ==================== COMMAND HANDLERS ==================== */
/**
* @brief Handle MIT mode position command
*/
void handle_mit_mode_command(char* cmd, float position, float speed) {
    char response[200];
    static float last_speed = 0.0f;
    static uint32_t last_time = 0;
    
    // Limit acceleration
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f;
    if (dt > 0) {
        float max_speed_change = SAFE_ACCELERATION * dt;
        if (fabs(speed - last_speed) > max_speed_change) {
            speed = last_speed + (speed > last_speed ? max_speed_change : -max_speed_change);
        }
    }
    
    pack_cmd(&can_msg, position, speed, DEFAULT_KP, DEFAULT_KD, CURRENT_LIMIT);
    TX_CAN(CAN_ID, can_msg.data, false);
    
    last_speed = speed;
    last_time = current_time;
    
    snprintf(response, sizeof(response),
             "MIT: Pos=%.2f rad, Speed=%.2f rad/s (limited)\r\n",
             position, speed);
    HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
}

/**
* @brief Process UART commands with position control
*/
void process_command(char* cmd) {
    char response[300];
    float angle_deg, velocity_limit;
    
    // Motor control commands
    if (strcmp(cmd, "m") == 0) {
        memcpy(can_msg.data, (uint8_t[]){CMD_DATA_FILL, CMD_DATA_FILL, CMD_DATA_FILL, CMD_DATA_FILL,
               CMD_DATA_FILL, CMD_DATA_FILL, CMD_DATA_FILL, CMD_ENTER_MOTOR_MODE}, 8);
        TX_CAN(CAN_ID, can_msg.data, false);
        motor_mode = 1;
        snprintf(response, sizeof(response), "Motor MIT mode activated\r\n");
    }
    else if (strcmp(cmd, "e") == 0) {
        memcpy(can_msg.data, (uint8_t[]){CMD_DATA_FILL, CMD_DATA_FILL, CMD_DATA_FILL, CMD_DATA_FILL,
               CMD_DATA_FILL, CMD_DATA_FILL, CMD_DATA_FILL, CMD_EXIT_MOTOR_MODE}, 8);
        TX_CAN(CAN_ID, can_msg.data, false);
        motor_mode = 0;
        pos_ctrl.control_enabled = 0; // Disable position control
        snprintf(response, sizeof(response), "Motor mode deactivated\r\n");
    }
    else if (strcmp(cmd, "z") == 0) {
        memcpy(can_msg.data, (uint8_t[]){CMD_DATA_FILL, CMD_DATA_FILL, CMD_DATA_FILL, CMD_DATA_FILL,
               CMD_DATA_FILL, CMD_DATA_FILL, CMD_DATA_FILL, CMD_SET_ZERO}, 8);
        TX_CAN(CAN_ID, can_msg.data, false);
        pos_ctrl.current_angle_rad = 0.0f; // Reset position tracking
        snprintf(response, sizeof(response), "Zero position set\r\n");
    }
    // NEW: Angle command - "a 90.5" sets target to 90.5 degrees
    else if (sscanf(cmd, "a %f", &angle_deg) == 1) {
        if (motor_mode) {
            pos_ctrl.target_angle_rad = angle_deg * 3.14159f / 180.0f; // Convert to radians
            pos_ctrl.control_enabled = 1; // Enable position control
            snprintf(response, sizeof(response), 
                     "Target angle set to %.2f degrees (%.3f rad)\r\n", 
                     angle_deg, pos_ctrl.target_angle_rad);
        } else {
            snprintf(response, sizeof(response), "Error: Enable MIT mode first ('m')\r\n");
        }
    }
    // NEW: Set maximum velocity - "v 5.0" sets max velocity to 5 rad/s  
    else if (sscanf(cmd, "v %f", &velocity_limit) == 1) {
        if (velocity_limit >= 0.1f && velocity_limit <= 10.0f) {
            pos_ctrl.max_velocity = velocity_limit;
            snprintf(response, sizeof(response), 
                     "Max velocity set to %.2f rad/s\r\n", velocity_limit);
        } else {
            snprintf(response, sizeof(response), "Error: Velocity must be 0.1-10.0 rad/s\r\n");
        }
    }
    // NEW: Stop position control
    else if (strcmp(cmd, "stop") == 0) {
        pos_ctrl.control_enabled = 0;
        snprintf(response, sizeof(response), "Position control stopped\r\n");
    }
    // Original MIT position command (kept for compatibility)
    else if (sscanf(cmd, "p %f %f", &angle_deg, &velocity_limit) == 2) {
        if (motor_mode) {
            HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
            handle_mit_mode_command(cmd, angle_deg * 3.14159f / 180.0f, velocity_limit);
            return;
        } else {
            snprintf(response, sizeof(response), "Error: Enable MIT mode first ('m')\r\n");
        }
    }
    // Enhanced status command
    else if (strcmp(cmd, "s") == 0) {
        float current_deg = pos_ctrl.current_angle_rad * 180.0f / 3.14159f;
        float target_deg = pos_ctrl.target_angle_rad * 180.0f / 3.14159f;
        snprintf(response, sizeof(response),
                 "Mode: %s, Control: %s\r\n"
                 "Position: %.2f° (target: %.2f°)\r\n" 
                 "Velocity: %.2f rad/s, Torque: %.2f A, Temp: %.1f°C\r\n",
                 motor_mode ? "MIT" : "Servo",
                 pos_ctrl.control_enabled ? "ON" : "OFF",
                 current_deg, target_deg,
                 current_velocity, current_torque, current_temp);
    }
    else if (strcmp(cmd, "help") == 0) {
        snprintf(response, sizeof(response),
                 "Commands:\r\n"
                 "m - Enable MIT mode\r\n"
                 "e - Disable MIT mode\r\n"  
                 "z - Set zero position\r\n"
                 "a [angle_deg] - Set target angle (servo mode)\r\n"
                 "v [vel_rad/s] - Set max velocity (0.1-10.0)\r\n"
                 "stop - Stop position control\r\n"
                 "p [pos_rad] [speed_rpm] - MIT direct command\r\n"
                 "s - Get status\r\n");
    }
    else {
        snprintf(response, sizeof(response), "Unknown command. Type 'help'\r\n");
    }
    
    HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
}

/* ==================== CALLBACK FUNCTIONS ==================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        static char command_buffer[32];
        static uint8_t buffer_index = 0;
        
        if (rx_data == '\b' && buffer_index > 0) {
            buffer_index--;
            char backspace_msg[] = "\b \b";
            HAL_UART_Transmit(&huart1, (uint8_t*)backspace_msg, strlen(backspace_msg), HAL_MAX_DELAY);
        }
        else if (rx_data == '\r' || rx_data == '\n') {
            if (buffer_index > 0) {
                command_buffer[buffer_index] = '\0';
                process_command(command_buffer);
                buffer_index = 0;
            }
        }
        else if (buffer_index < sizeof(command_buffer) - 1) {
            command_buffer[buffer_index++] = rx_data;
            HAL_UART_Transmit(&huart1, &rx_data, 1, HAL_MAX_DELAY);
        }
        
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    CANMessage msg;
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, msg.data) == HAL_OK) {
        char uart_buffer[150];
        int length = snprintf(uart_buffer, sizeof(uart_buffer), "RX CAN ID=0x%lx, Data=", RxHeader.StdId);
        
        for (int i = 0; i < RxHeader.DLC; i++) {
            length += snprintf(&uart_buffer[length], sizeof(uart_buffer) - length, "%02X ", msg.data[i]);
        }
        
        length += snprintf(&uart_buffer[length], sizeof(uart_buffer) - length, "\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, length, HAL_MAX_DELAY);
        
        if (RxHeader.StdId == CAN_ID && RxHeader.DLC == 8) {
            unpack_reply(msg);
        }
    }
}

/* ==================== MAIN FUNCTION ==================== */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    CAN_Config();

    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

    const char welcome_msg[] = "\r\n=== AK80-8 Smooth Position Controller ===\r\n"
                              "Type 'help' for commands\r\n"
                              "Use: m -> a 90.5 (enable MIT mode, then set 90.5°)\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)welcome_msg, strlen(welcome_msg), HAL_MAX_DELAY);

    uint32_t last_control_time = 0;
    const uint32_t CONTROL_PERIOD = 20; // 20ms = 50Hz control loop
    
    while (1) {
        uint32_t current_time = HAL_GetTick();
        
        // Run position control at fixed intervals
        if (current_time - last_control_time >= CONTROL_PERIOD) {
            update_position_control();
            last_control_time = current_time;
        }
        
        // Small delay to prevent excessive CPU usage
        HAL_Delay(1);
    }
}

/* ==================== SYSTEM CONFIGURATION ==================== */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

void CAN_Config(void) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 4;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    
    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        Error_Handler();
    }
    
    CAN_FilterTypeDef sFilterConfig = {
        .FilterBank = 0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterFIFOAssignment = CAN_FILTER_FIFO1,
        .FilterActivation = ENABLE
    };
    
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        Error_Handler();
    }
    
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    
    const char msg[] = "CAN configured successfully\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void Error_Handler(void) {
    const char errormsg[] = "CRITICAL ERROR\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)errormsg, strlen(errormsg), HAL_MAX_DELAY);
    NVIC_SystemReset();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
    // Debug assertion failed
}
#endif

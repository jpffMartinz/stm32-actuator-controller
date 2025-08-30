/**
******************************************************************************
* @file : main.c
* @brief : STM32 Blue Pill CAN Communication with AK80-8 Actuator
* Supports MIT Mode with Smooth Position Control
******************************************************************************
*/

#include "main.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* ==================== DEFINES ==================== */
#define CAN_ID 0x01 // Standard ID for AK80-8 actuator



/* ==================== GLOBAL VARIABLES ==================== */
// TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
CAN_HandleTypeDef hcan;
uint8_t rx_data;
CANMessage can_msg;


// Servo simulation variables
static uint32_t simulation_counter = 0;
static float target_position = 0.0f;
static float current_position = 0.0f;
static uint8_t uart_tx_buffer[8];

/* ==================== AK80-8 SERVO SIMULATION ==================== */
typedef struct {
    int16_t position;     // Position in 0.1° units (-32000 to 32000)
    int16_t speed;        // Speed in 10 RPM units (-32000 to 32000) 
    int16_t current;      // Current in 0.01A units (-6000 to 6000)
    int8_t temperature;   // Temperature in °C (-20 to 127)
    uint8_t error_code;   // Error code (0 = no error)
} AK80_ServoData_t;


static AK80_ServoData_t servo_data = {0};

/* ==================== FUNCTION PROTOTYPES ==================== */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
// static void MX_TIM2_Init(void);
void CAN_Config(void);
void Error_Handler(void);
void Generate_Mock_ServoData(void);
void Pack_And_Send_ServoData(void);



// /* ==================== TIMER CONFIGURATION ==================== */
// static void MX_TIM2_Init(void) {
//     __HAL_RCC_TIM2_CLK_ENABLE();
    
//     htim2.Instance = TIM2;
//     htim2.Init.Prescaler = 7199;      // 72MHz / 7200 = 10kHz
//     htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//     htim2.Init.Period = 499;          // 10kHz / 500 = 20Hz (50ms period)
//     htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//     htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
//     if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
//         Error_Handler();
//     }

//     HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);  // Set interrupt priority
//     HAL_NVIC_EnableIRQ(TIM2_IRQn);          // Enable TIM2 interrupt
    
//     // Start timer with interrupt
//     if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
//         Error_Handler();
//     }
    
//     const char msg[] = "Timer configured for 20Hz servo simulation\r\n";
//     HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
// }


/* ==================== SERVO SIMULATION FUNCTIONS ==================== */
void Generate_Mock_ServoData(void) {
    simulation_counter++;
    
    // Simulate smooth sinusoidal movement between -180° and +180°
    target_position = 180.0f * sinf(simulation_counter * 0.01f);
    
    // Simple first-order position tracking (simulates servo response)
    float position_error = target_position - current_position;
    current_position += position_error * 0.1f; // 10% tracking per update
    
    // Convert current position to protocol units (0.1° resolution)
    servo_data.position = (int16_t)(current_position * 10.0f);
    
    // Simulate speed based on position change
    static float prev_position = 0.0f;
    float speed_deg_per_sec = (current_position - prev_position) * 20.0f; // 20Hz update rate
    prev_position = current_position;
    
    // Convert to electrical RPM (simplified conversion)
    float speed_rpm = speed_deg_per_sec * 60.0f / 360.0f * 100.0f; // Rough conversion
    servo_data.speed = (int16_t)(speed_rpm / 10.0f); // Protocol units: 10 RPM
    
    // Simulate current based on position error (higher error = more current)
    float current_amps = fabsf(position_error) * 0.05f; // 0.05A per degree of error
    servo_data.current = (int16_t)(current_amps * 100.0f); // Protocol units: 0.01A
    
    // Simulate temperature (varies slowly between 25-35°C)
    servo_data.temperature = 30 + (int8_t)(5.0f * sinf(simulation_counter * 0.001f));
    
    // Simulate occasional errors for testing
    if (simulation_counter % 1000 == 0) {
        servo_data.error_code = 1; // Simulate over-temperature warning
    } else if (simulation_counter % 500 == 0) {
        servo_data.error_code = 2; // Simulate over-current warning  
    } else {
        servo_data.error_code = 0; // No error
    }
    
    // Clamp values to valid ranges
    if (servo_data.position < -32000) servo_data.position = -32000;
    if (servo_data.position > 32000) servo_data.position = 32000;
    if (servo_data.speed < -32000) servo_data.speed = -32000;
    if (servo_data.speed > 32000) servo_data.speed = 32000;
    if (servo_data.current < -6000) servo_data.current = -6000;
    if (servo_data.current > 6000) servo_data.current = 6000;
}

void Pack_And_Send_ServoData(void) {
    // Pack data according to AK80-8 protocol (8 bytes)
    uart_tx_buffer[0] = (servo_data.position >> 8) & 0xFF;    // Position High
    uart_tx_buffer[1] = servo_data.position & 0xFF;           // Position Low
    uart_tx_buffer[2] = (servo_data.speed >> 8) & 0xFF;       // Speed High  
    uart_tx_buffer[3] = servo_data.speed & 0xFF;              // Speed Low
    uart_tx_buffer[4] = (servo_data.current >> 8) & 0xFF;     // Current High
    uart_tx_buffer[5] = servo_data.current & 0xFF;            // Current Low
    uart_tx_buffer[6] = (uint8_t)servo_data.temperature;      // Temperature
    uart_tx_buffer[7] = servo_data.error_code;                // Error Code
    
    // Send via UART (non-blocking)
    HAL_UART_Transmit(&huart1, uart_tx_buffer, 8, 10);
}

// // Timer interrupt callback (called at 20Hz)
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim->Instance == TIM2) {

//         static uint32_t debug_counter = 0;
//         debug_counter++;


//         Generate_Mock_ServoData();
//         Pack_And_Send_ServoData();

//         if (debug_counter % 100 == 0) {
//             const char debug[] = "Timer ISR working\r\n";
//             HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), 10);
//         }

//     }
// }

/* ==================== UART COMMAND PROCESSING ==================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Simple command processing for testing
        switch (rx_data) {
            case 'h':
                {
                    const char help[] = "Commands: h=help, s=start, p=stop, r=reset, t=test\r\n";
                    HAL_UART_Transmit(&huart1, (uint8_t*)help, strlen(help), HAL_MAX_DELAY);
                }
                break;
                
            // case 's':
            //     HAL_TIM_Base_Start_IT(&htim2);
            //     {
            //         const char msg[] = "Servo simulation started\r\n";
            //         HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            //     }
            //     break;
                
            // case 'p':
            //     HAL_TIM_Base_Stop_IT(&htim2);
            //     {
            //         const char msg[] = "Servo simulation paused\r\n";
            //         HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            //     }
            //     break;
                
            case 'r':
                simulation_counter = 0;
                current_position = 0.0f;
                {
                    const char msg[] = "Servo simulation reset\r\n";
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                }
                break;
                
            case 't':
                {
                    char test_msg[100];
                    snprintf(test_msg, sizeof(test_msg), 
                        "Pos: %.1f°, Spd: %d, Cur: %.2fA, Temp: %d°C, Err: %d\r\n",
                        servo_data.position * 0.1f, 
                        servo_data.speed * 10,
                        servo_data.current * 0.01f,
                        servo_data.temperature,
                        servo_data.error_code);
                    HAL_UART_Transmit(&huart1, (uint8_t*)test_msg, strlen(test_msg), HAL_MAX_DELAY);
                }
                break;
        }
        
        // Re-enable UART receive interrupt
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}


// void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//     CAN_RxHeaderTypeDef RxHeader;
//     CANMessage msg;
    
//     if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, msg.data) == HAL_OK) {
//         char uart_buffer[150];
//         int length = snprintf(uart_buffer, sizeof(uart_buffer), "RX CAN ID=0x%lx, Data=", RxHeader.StdId);
        
//         for (int i = 0; i < RxHeader.DLC; i++) {
//             length += snprintf(&uart_buffer[length], sizeof(uart_buffer) - length, "%02X ", msg.data[i]);
//         }
        
//         length += snprintf(&uart_buffer[length], sizeof(uart_buffer) - length, "\r\n");
//         HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, length, HAL_MAX_DELAY);
        
//         if (RxHeader.StdId == CAN_ID && RxHeader.DLC == 8) {
//             unpack_reply(msg);
//         }
//     }
// }



/* ==================== MAIN FUNCTION ==================== */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    // MX_TIM2_Init();        // Add this line
    CAN_Config();

    const char startup[] = "STM32 AK80-8 Servo Simulator Ready\r\nSend 'h' for help\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)startup, strlen(startup), HAL_MAX_DELAY);
    
    // Initialize servo simulation
    servo_data.temperature = 25;
    servo_data.error_code = 0;
    
    while (1) {
        Generate_Mock_ServoData();
        Pack_And_Send_ServoData();
        HAL_Delay(50);
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

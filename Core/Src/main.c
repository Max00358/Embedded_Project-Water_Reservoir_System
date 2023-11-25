/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

#include "stdio.h"

#include <inttypes.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_D 30;
#define MAX_D 100;

// max PWM for the DC motor
const uint32_t MAX_PWM = 2000;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
// Mode of the Reservoir Project: 0 = SETUP, 1 = RUNNING
int mode = 0;
// timer interrupt
int time_changed = 1;
// keep track of zone
int zones[3] = {
  0
};

//for receiving terminal input
uint8_t byte[2];
int rpm_tick_count = 0;
uint8_t us100_Rx_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int rcv_intpt_flag = 0;
int t_curr = 0;
int counter;
uint8_t us100_buffer[2] = {
  0
};
uint8_t cmd_dist = 0x55;
volatile uint16_t distance = 0;
// Buffer to write to terminal
uint8_t msg_buffer[512] = {
  0
};
// For potentiometer
void ADC_Select_CH(int CH) {
  ADC_ChannelConfTypeDef sConfig = {
    0
  };
  switch (CH) {
  case 0:
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 1:
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 2:
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 3:
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 4:
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 5:
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 6:
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 7:
    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 8:
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 9:
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 10:
    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 11:
    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 12:
    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 13:
    sConfig.Channel = ADC_CHANNEL_13;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 14:
    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  case 15:
    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = 1;
    if (HAL_ADC_ConfigChannel( & hadc1, & sConfig) != HAL_OK) {
      Error_Handler();
    }
    break;
  }
}

// Displays digits in the timer board
void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B) {
  uint8_t DIGITA_VAL = 0x0F & DIGIT_A; //mask off higher4 bits
  int Abit0 = (DIGITA_VAL) & 1; // extract Abit0 of the 4-bit value
  int Abit1 = (DIGITA_VAL >> 1) & 1; // extract Abit1 of the 4-bit value
  int Abit2 = (DIGITA_VAL >> 2) & 1; // extract Abit2 of the 4-bit value
  int Abit3 = (DIGITA_VAL >> 3) & 1; // extract Abit3 of the 4-bit value

  uint8_t DIGITB_VAL = 0x0F & DIGIT_B; //mask off higher4 bits
  int Bbit0 = (DIGITB_VAL) & 1; // extract Bbit0 of the 4-bit value
  int Bbit1 = (DIGITB_VAL >> 1) & 1; // extract Bbit1 of the 4-bit value
  int Bbit2 = (DIGITB_VAL >> 2) & 1; // extract Bbit2 of the 4-bit value
  int Bbit3 = (DIGITB_VAL >> 3) & 1; // extract Bbit3 of the 4-bit value

  if (Abit0 == (0)) {
    HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);

  }
  if (Abit1 == (0)) {
    HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET);

  }
  if (Abit2 == (0)) {
    HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);

  }
  if (Abit3 == (0)) {
    HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_SET);

  }

  if (Bbit0 == (0)) {
    HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_SET);

  }
  if (Bbit1 == (0)) {
    HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_SET);

  }
  if (Bbit2 == (0)) {
    HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_SET);

  }
  if (Bbit3 == (0)) {
    HAL_GPIO_WritePin(GPIOB, DIGIT_B3_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOB, DIGIT_B3_Pin, GPIO_PIN_SET);

  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  // store user input in here
  int inlet_choice;
  int z1_choice;
  int z2_choice;
  int z3_choice;

  int t_in_start;
  int t_in_stop;
  int t_z1_start;
  int t_z1_stop;
  int t_z2_start;
  int t_z2_stop;
  int t_z3_start;
  int t_z3_stop;

  // For SERVO MOTOR
  int TIM2_Ch1_DCVAL = 500;
  int TIM2_CH1_STEP = 125;

  HAL_TIM_Base_Start( & htim2);
  HAL_TIM_PWM_Start( & htim2, TIM_CHANNEL_1);
  TIM2 -> PSC = 16 - 1;
  TIM2 -> ARR = 20000 - 1;
  TIM2 -> CCR1 = TIM2_Ch1_DCVAL;

  // For DC MOTOR
  int TIM3_Ch1_DCVAL = 0;
  int TIM3_Ch3_DCVAL = 0;
  int TIM3_Ch1_STEP = 200;
  TIM3 -> PSC = 16 - 1;
  TIM3 -> ARR = 2000 - 1;
  TIM3 -> CCR1 = 0;
  HAL_TIM_Base_Init( & htim3);
  HAL_TIM_PWM_Start( & htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start( & htim3, TIM_CHANNEL_3);

  // start up TIMER 5 for one second interrupts
  HAL_TIM_Base_Start_IT( & htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Set Green LED on Nucleo Board to OFF
  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);

  //here DC motor needs to be off

  // Display SETUP mode on terminal
  sprintf((char * ) msg_buffer, "SETUP MODE");
  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

  // ZONE/INLET PWM INFO
  sprintf((char * ) msg_buffer, "\r\nUse the following list for entering the PWM option for the MOTOR SPEED for a ZONE or INLET:");
  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
  sprintf((char * ) msg_buffer, "\r\n0) Manual Control (the Potentiometer setting in Run Mode for the chosen Zone or Inlet);");
  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
  sprintf((char * ) msg_buffer, "\r\n1) 60% PWM;");
  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
  sprintf((char * ) msg_buffer, "\r\n2) 80% PWM;");
  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
  sprintf((char * ) msg_buffer, "\r\n3) 99% PWM;\r\n");
  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

  DIGITS_Display(0, 0);

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // PART A DISPLAY OPTIONS
    // Provide message to cover INLET choice
    rcv_intpt_flag = 00; // this flag is used to see if an interrupt has occurred
    HAL_UART_Receive_IT( & huart6, & byte, 1); // enables the receiver to create an interrupt
    sprintf((char * ) msg_buffer, "\r\nINLET MOTOR SPEED PWM (option 0-3):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 1, 100);
    inlet_choice = byte[0] - 48;

    // Provide message to cover ZONE 1 choice
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 1);
    sprintf((char * ) msg_buffer, "\r\nZONE 1 MOTOR SPEED PWM (option 0-3):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 1, 100);
    z1_choice = byte[0] - 48;

    // Provide message to cover ZONE 2 choice
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 1);
    sprintf((char * ) msg_buffer, "\r\nZONE 2 MOTOR SPEED PWM (option 0-3):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 1, 100);
    z2_choice = byte[0] - 48;

    // Provide message to cover ZONE 3 choice
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 1);
    sprintf((char * ) msg_buffer, "\r\nZONE 3 MOTOR SPEED PWM (option 0-3):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 1, 100);
    z3_choice = byte[0] - 48;

    // PART B DISPLAY OPTIONS

    // GET CURRENT CLOCK TIME
    sprintf((char * ) msg_buffer, "\r\nCURRENT WALL CLOCK TIME (0-23):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 2);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 2, 100);
    t_curr = (byte[0] - 48) * 10 + (byte[1] - 48);

    // GET INLET WALL CLOCK TIME
    sprintf((char * ) msg_buffer, "\r\nINLET WALL CLOCK START TIME (0-23):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 2);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 2, 100);
    t_in_start = (byte[0] - 48) * 10 + (byte[1] - 48);

    sprintf((char * ) msg_buffer, "\r\nINLET WALL CLOCK STOP TIME (0-23):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 2);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 2, 100);
    t_in_stop = (byte[0] - 48) * 10 + (byte[1] - 48);

    // GET ZONE 1 WALL CLOCK TIME
    sprintf((char * ) msg_buffer, "\r\nZONE 1 WALL CLOCK START TIME (0-23):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 2);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 2, 100);
    t_z1_start = (byte[0] - 48) * 10 + (byte[1] - 48);

    sprintf((char * ) msg_buffer, "\r\nZONE 1 WALL CLOCK STOP TIME (0-23):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 2);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 2, 100);
    t_z1_stop = (byte[0] - 48) * 10 + (byte[1] - 48);

    // GET ZONE 2 WALL CLOCK TIME
    sprintf((char * ) msg_buffer, "\r\nZONE 2 WALL CLOCK START TIME (0-23):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 2);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 2, 100);
    t_z2_start = (byte[0] - 48) * 10 + (byte[1] - 48);

    sprintf((char * ) msg_buffer, "\r\nZONE 2 WALL CLOCK STOP TIME (0-23):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 2);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 2, 100);
    t_z2_stop = (byte[0] - 48) * 10 + (byte[1] - 48);

    // GET ZONE 3 WALL CLOCK TIME
    sprintf((char * ) msg_buffer, "\r\nZONE 3 WALL CLOCK START TIME (0-23):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 2);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 2, 100);
    t_z3_start = (byte[0] - 48) * 10 + (byte[1] - 48);

    sprintf((char * ) msg_buffer, "\r\nZONE 3 WALL CLOCK STOP TIME (0-23):");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);
    rcv_intpt_flag = 00;
    HAL_UART_Receive_IT( & huart6, & byte, 2);
    while (rcv_intpt_flag == 00) {};
    HAL_UART_Transmit( & huart6, & byte, 2, 100);
    t_z3_stop = (byte[0] - 48) * 10 + (byte[1] - 48);

    while ((HAL_GPIO_ReadPin(GPIOC, B1_Pin))) {
      HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
      HAL_Delay(250);
      HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
      HAL_Delay(250);
    }

    // as soon as it comes out of the while loop - BLUEe button has been pushed
    // start run mode
    // Display RUN mode on terminal
    sprintf((char * ) msg_buffer, "\r\nRUN MODE");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

    // the green led stays on in run mode
    HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);

    // Let's compare all the zone times to see which one is going to go first
    if (t_z1_start < t_z2_start && t_z1_start < t_z3_start) {
      zones[0] = 1;
      if (t_z2_start < t_z3_start) {
        zones[1] = 2;
        zones[2] = 3;
      } else {
        zones[1] = 3;
        zones[2] = 2;
      }
    }

    if (t_z2_start < t_z1_start && t_z2_start < t_z3_start) {
      zones[0] = 2;
      if (t_z1_start < t_z3_start) {
        zones[1] = 1;
        zones[2] = 3;
      } else {
        zones[1] = 3;
        zones[2] = 1;
      }
    }

    if (t_z3_start < t_z1_start && t_z3_start < t_z2_start) {
      zones[0] = 3;
      if (t_z1_start < t_z2_start) {
        zones[1] = 1;
        zones[2] = 2;
      } else {
        zones[1] = 2;
        zones[2] = 1;
      }
    }

    //keep track of which zone we are in
    int zone = zones[0];
    // run mode starts with setting the wall clock times
    mode = 1;

    sprintf((char * ) msg_buffer, "\r\nWall-Clock Time | Zone/Inlet | Motor Speed %%PWM | Motor RPM | Water Reservoir Depth\n\r");
    HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

    // While inlet start time has not occurred
    while (t_curr != t_in_start) {};

    // Okay, so inlet time has now started
    // Turn on the LED to indicate PURPLE_PIN
    HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET);

    // SET SERVO FOR INLET ZONE HERE
    TIM3 -> CCR1 = 0;
    TIM2 -> CCR1 = 2500;
    TIM3 -> CCR3 = 0;
    HAL_Delay(2000);
    time_changed = 1;
    rpm_tick_count = 0;

    while ((t_in_start <= t_curr) && (t_curr <= t_in_stop)) {
      if (time_changed == 1) {
        int motor_speed = 0;

        // MOTOR PWM
        if (inlet_choice == 0) {
          ADC_Select_CH(9);
          HAL_ADC_Start( & hadc1);
          HAL_ADC_PollForConversion( & hadc1, 1000);
          uint8_t ADC_CH9 = HAL_ADC_GetValue( & hadc1) * 100 / 255;
          HAL_ADC_Stop( & hadc1);
          motor_speed = ADC_CH9;
        }
        if (inlet_choice == 1) motor_speed = 60;
        if (inlet_choice == 2) motor_speed = 80;
        if (inlet_choice == 3) motor_speed = 99;

        //change TIM3->CCR1 between 1200 and 2000
        TIM3 -> CCR1 = 20 * (motor_speed);
        HAL_Delay(500);

        HAL_UART_Receive_IT( & huart1, us100_buffer, 2);
        HAL_UART_Transmit( & huart1, & cmd_dist, 1, 500);
        while (us100_Rx_flag == (00)) {};
        distance = (us100_buffer[0] << 8) | (us100_buffer[1]);

        if (distance > 250) {
          uint8_t display_distance = 0;
          DIGITS_Display(display_distance, display_distance);
          //sprintf((char * ) msg_buffer, "\r\nRESERVOIR IS EMPTY\n\r");
          HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

          	// the green led stays on in run mode
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
			while (1) {
			  HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);

			  HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_SET);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_RESET);

			  HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_SET);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_RESET);
			}
        } else if (distance > 30) {
          uint8_t display_distance = (100 - ((float)(distance)-30.0)*(100.0/220.0) );
          DIGITS_Display(display_distance / 10, display_distance % 10);
        } else {
          uint8_t display_distance = 9;
          HAL_UART_Transmit( & huart6, msg_buffer, strlen((char*) msg_buffer), 1000);
          DIGITS_Display(display_distance, display_distance);
        }

        us100_Rx_flag = 00;

        uint8_t rpm_int = rpm_tick_count;
        uint32_t rpm_val = (rpm_int) * 6 / 20;
        sprintf((char * ) msg_buffer, "\r        %d        |    Inlet   |        %d%%       |     %d    |         %d     \n", t_curr, motor_speed, rpm_val, distance);

        HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

        time_changed = 0;
      }
    };

    HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    TIM3 -> CCR1 = 0;
    int i = 0;
    while (i < 3) {
      zone = zones[i];

      if (zone == 1) {
        // SET SERVO MOTOR FOR ZONE 1
    	TIM2 -> CCR1 = 0;
        TIM2 -> CCR1 = 2000;
        HAL_Delay(2000);

        // LED
        HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET);
        HAL_Delay(500);
        rpm_tick_count = 0;

        while ((t_z1_start <= t_curr) && (t_curr <= t_z1_stop)) {
          if (time_changed == 1) {
            int motor_speed = 0;

            // MOTOR PWM
            if (z1_choice == 0) {
              ADC_Select_CH(9);
              HAL_ADC_Start( & hadc1);
              HAL_ADC_PollForConversion( & hadc1, 1000);
              uint8_t ADC_CH9 = HAL_ADC_GetValue( & hadc1) * 100 / 255;
              HAL_ADC_Stop( & hadc1);
              motor_speed = ADC_CH9;
            }
            if (z1_choice == 1) motor_speed = 60;
            if (z1_choice == 2) motor_speed = 80;
            if (z1_choice == 3) motor_speed = 99;

            //change TIM3->CCR1 between 1200 and 2000
            TIM3 -> CCR3 = 20 * (motor_speed);
            HAL_Delay(500);

            HAL_UART_Receive_IT( & huart1, us100_buffer, 2);
            HAL_UART_Transmit( & huart1, & cmd_dist, 1, 500);
            while (us100_Rx_flag == (00)) {};
            distance = (us100_buffer[0] << 8) | (us100_buffer[1]);
            if (distance > 250) {
			  uint8_t display_distance = 0;
			  DIGITS_Display(display_distance, display_distance);

			  //Stop DC Motor
			  TIM3 -> CCR3 = 0;
			  TIM3 -> CCR1 = 0;
			  sprintf((char * ) msg_buffer, "\r\nRESERVOIR IS EMPTY\n\r");
			  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

				// the green led stays on in run mode
				HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
				while (1) {
				  HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);

				  HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_RESET);

				  HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_RESET);
				}
			} else if (distance > 30) {
				uint8_t display_distance = (100 - ((float)(distance)-30.0)*(100.0/220.0) );
			  DIGITS_Display(display_distance / 10, display_distance % 10);
			} else {
			  uint8_t display_distance = 9;
			  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char*) msg_buffer), 1000);
			  DIGITS_Display(display_distance, display_distance);
			}
            us100_Rx_flag = 00;

            uint32_t rpm_val = (rpm_tick_count) * 6 / 20;
            sprintf((char * ) msg_buffer, "\r        %d        |    Zone 1  |        %d%%       |     %d    |         %d     \n", t_curr, motor_speed, rpm_val, distance);
            rpm_tick_count = 0;

            HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

            time_changed = 0;
          }
        }

        i++;

        HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
      }

      if (zone == 2) {
        // SET SERVO MOTOR FOR ZONE 2
    	TIM2 -> CCR1 = 0;
        TIM2 -> CCR1 = 1500;
        HAL_Delay(2000);
        // LED
        HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_SET);
        HAL_Delay(500);
        rpm_tick_count = 0;

        while ((t_z2_start <= t_curr) && (t_curr <= t_z2_stop)) {
          if (time_changed == 1) {
            int motor_speed = 0;

            // MOTOR PWM
            if (z2_choice == 0) {
              ADC_Select_CH(9);
              HAL_ADC_Start( & hadc1);
              HAL_ADC_PollForConversion( & hadc1, 1000);
              uint8_t ADC_CH9 = HAL_ADC_GetValue( & hadc1) * 100 / 255;
              HAL_ADC_Stop( & hadc1);
              motor_speed = ADC_CH9;
            }
            if (z2_choice == 1) motor_speed = 60;
            if (z2_choice == 2) motor_speed = 80;
            if (z2_choice == 3) motor_speed = 99;

            //change TIM3->CCR1 between 1200 and 2000
            TIM3 -> CCR3 = 20 * (motor_speed);
            HAL_Delay(500);

            HAL_UART_Receive_IT( & huart1, us100_buffer, 2);
            HAL_UART_Transmit( & huart1, & cmd_dist, 1, 500);
            while (us100_Rx_flag == (00)) {};
            distance = (us100_buffer[0] << 8) | (us100_buffer[1]);
            if (distance > 250) {
			  uint8_t display_distance = 0;
			  DIGITS_Display(display_distance, display_distance);

			  //Stop DC Motor
			  TIM3 -> CCR3 = 0;
			  TIM3 -> CCR1 = 0;
			  sprintf((char * ) msg_buffer, "\r\nRESERVOIR IS EMPTY\n\r");
			  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

			  //Stop DC Motor
			  TIM3 -> CCR3 = 0;
			  TIM3 -> CCR1 = 0;

				// the green led stays on in run mode
				HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
				while (1) {
				  HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);

				  HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_RESET);

				  HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_RESET);
				}
			} else if (distance > 30) {
			  uint8_t display_distance = (100 - ((float)(distance)-30.0)*(100.0/220.0) );
			  DIGITS_Display(display_distance / 10, display_distance % 10);
			} else {
			  uint8_t display_distance = 9;
			  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char*) msg_buffer), 1000);
			  DIGITS_Display(display_distance, display_distance);
			}
            us100_Rx_flag = 00;

            uint8_t rpm_int = rpm_tick_count;
            uint32_t rpm_val = (rpm_int) * 6 / 20;
            sprintf((char * ) msg_buffer, "\r        %d        |    Zone 2  |        %d%%       |     %d    |       %d      \n", t_curr, motor_speed, rpm_val, distance);
            rpm_tick_count = 0;

            HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

            time_changed = 0;
          }
        }

        i++;
        HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
      }

      if (zone == 3) {
        // SET SERVO MOTOR FOR ZONE 3
    	TIM2 -> CCR1 = 0;
        TIM2 -> CCR1 = 1000;
        HAL_Delay(2000);
        // LED
        HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_SET);
        HAL_Delay(500);
        rpm_tick_count = 0;

        while ((t_z3_start <= t_curr) && (t_curr <= t_z3_stop)) {
          if (time_changed == 1) {
            int motor_speed = 0;

            // MOTOR PWM
            if (z3_choice == 0) {
              ADC_Select_CH(9);
              HAL_ADC_Start( & hadc1);
              HAL_ADC_PollForConversion( & hadc1, 1000);
              uint8_t ADC_CH9 = HAL_ADC_GetValue( & hadc1) * 100 / 255;
              HAL_ADC_Stop( & hadc1);
              motor_speed = ADC_CH9;
            }
            if (z3_choice == 1) motor_speed = 60;
            if (z3_choice == 2) motor_speed = 80;
            if (z3_choice == 3) motor_speed = 99;

            //change TIM3->CCR1 between 1200 and 2000
            TIM3 -> CCR3 = 20 * (motor_speed);
            HAL_Delay(500);

            HAL_UART_Receive_IT( & huart1, us100_buffer, 2);
            HAL_UART_Transmit( & huart1, & cmd_dist, 1, 500);
            while (us100_Rx_flag == (00)) {};
            distance = (us100_buffer[0] << 8) | (us100_buffer[1]);
            if (distance > 250) {
			  uint8_t display_distance = 0;
			  DIGITS_Display(display_distance, display_distance);

			  //Stop DC Motor
			  TIM3 -> CCR3 = 0;
			  TIM3 -> CCR1 = 0;
			  sprintf((char * ) msg_buffer, "\r\nRESERVOIR IS EMPTY\n\r");
			  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

			  //Stop DC Motor
			  TIM3 -> CCR3 = 0;
			  TIM3 -> CCR1 = 0;

				// the green led stays on in run mode
				HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
				while (1) {
				  HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);

				  HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(GPIOA, BLUE_Pin, GPIO_PIN_RESET);

				  HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_RESET);
				}
			} else if (distance > 30) {
			  uint8_t display_distance = (100 - ((float)(distance)-30.0)*(100.0/220.0) );
			  DIGITS_Display(display_distance / 10, display_distance % 10);
			} else {
			  uint8_t display_distance = 9;
			  HAL_UART_Transmit( & huart6, msg_buffer, strlen((char*) msg_buffer), 1000);
			  DIGITS_Display(display_distance, display_distance);
			}
            us100_Rx_flag = 00;

            uint32_t rpm_val = (rpm_tick_count) * 6 / 20;
            sprintf((char * ) msg_buffer, "\r         %d       |    Zone 3  |        %d%%       |     %d    |           %d     \n", t_curr, motor_speed, rpm_val, distance);
            rpm_tick_count = 0;

            HAL_UART_Transmit( & huart6, msg_buffer, strlen((char * ) msg_buffer), 1000);

            time_changed = 0;
          }
        }
        i++;
        HAL_GPIO_WritePin(GPIOA, GREEN_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
      }
    }

    TIM3 -> CCR3 = 0;
    TIM3 -> CCR1 = 0;
    while (1) {};
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|BLUE_Pin|GREEN_Pin|RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin|DIGIT_B1_Pin|DIGIT_B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin|DIGIT_A1_Pin|DIGIT_B3_Pin|DIGIT_A2_Pin
                          |DIGIT_A3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin BLUE_Pin GREEN_Pin RED_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|BLUE_Pin|GREEN_Pin|RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_TICK_Pin */
  GPIO_InitStruct.Pin = RPM_TICK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_TICK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_B0_Pin DIGIT_B1_Pin DIGIT_B2_Pin */
  GPIO_InitStruct.Pin = DIGIT_B0_Pin|DIGIT_B1_Pin|DIGIT_B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_A0_Pin DIGIT_A1_Pin DIGIT_B3_Pin DIGIT_A2_Pin
                           DIGIT_A3_Pin */
  GPIO_InitStruct.Pin = DIGIT_A0_Pin|DIGIT_A1_Pin|DIGIT_B3_Pin|DIGIT_A2_Pin
                          |DIGIT_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
  if (huart -> Instance == USART6) {
    rcv_intpt_flag = 1;
  }

  if (huart -> Instance == USART1) {
    us100_Rx_flag = 01; // this flag is set to show that an receiver interrupt has occurred
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == RPM_TICK_Pin) {
    rpm_tick_count += 1;
  }
}

/*
 * timer interrupt
 * total_seconds += 1
 * if total_seconds % delta_t (6s) == 0
 * calculate_rpm from rpm_tick_count over delta t
 * tick per 2s / 30 2s per minute / 20 ticks per rotation = rotations per minute
 * rpm_tick_count = 0
 * */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
//void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
  if (htim -> Instance == TIM5 && mode == 1) {
    counter++; //num of seconds

    if (counter % 6 == 0) {
      time_changed = 1;
      t_curr += 1;
      if (t_curr == 24) {
        t_curr = 0;
      }
    };
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

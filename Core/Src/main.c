/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_mode.h"
#include "ms5837_dma.h"
#include "PID.h"
#include "mpu6050.h"
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MS5837_t depth_sensor;            // Sensor instance
PID_t pid_depth;
PID_t pid_pitch;
PID_t pid_roll;
MPU6050_t mpu6050;                // MPU6050 IMU instance



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Task 1 bench mode: no sensors, ESC PWM, or ROV control are started. */
#define COMM_TEST_MODE  1

// ---- VERTICAL MOTORS (depth control) ----
//   V_FL = Front Left  = TIM4 CH3 = PB8
//   V_FR = Front Right = TIM4 CH4 = PB9
//   V_RL = Rear Left   = TIM2 CH1 = PA0
//   V_RR = Rear Right  = TIM3 CH4 = PB1
#define MOTOR_V_FL  TIM_CHANNEL_3   // htim4 CH3, PB8
#define MOTOR_V_FR  TIM_CHANNEL_4   // htim4 CH4, PB9
#define MOTOR_V_RL  TIM_CHANNEL_1   // htim2 CH1, PA0
#define MOTOR_V_RR  TIM_CHANNEL_4   // htim3 CH4, PB1

// ---- HORIZONTAL MOTORS ----
//   O_FL = Front Left  = TIM3 CH1 = PA6
//   O_FR = Front Right = TIM2 CH2 = PA1
//   O_RL = Rear Left   = TIM3 CH2 = PA7
//   O_RR = Rear Right  = TIM3 CH3 = PB0
#define MOTOR_O_FL  TIM_CHANNEL_1   // htim3 CH1, PA6
#define MOTOR_O_FR  TIM_CHANNEL_2   // htim2 CH2, PA1
#define MOTOR_O_RL  TIM_CHANNEL_2   // htim3 CH2, PA7
#define MOTOR_O_RR  TIM_CHANNEL_3   // htim3 CH3, PB0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float pressure, temp, depth;
float err_depth, out_depth, err_pitch , out_pitch, err_roll, out_roll;
int time = 0;
float target_depth = 0.1f;
float current_depth = 0.0f;
//float target_pitch = 4.7f;   // target pitch angle in degrees (0 = level)
//float target_roll = -2.3f;      // target roll angle in degrees (0 = level)

// --- Horizontal (omnidirectional) movement ---
// forward_input: -500 to +500  (positive = forward, negative = backward)
//int forward_input = 150;

// --- Depth stabilization gate for horizontal motors ---
// Horizontal thrust is DISABLED until depth stabilizes.
// Depth must stay within DEPTH_STABLE_TOL of target for
// DEPTH_STABLE_COUNT consecutive loop iterations.
#define DEPTH_STABLE_TOL    0.05f   // metres – how close is "stable"
#define DEPTH_STABLE_COUNT  50      // iterations (~1 s at 50 Hz loop)
int  stable_counter = 0;            // consecutive-in-range counter
int  depth_stable   = 0;            // 1 = stable, horizontal enabled
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

void CommTest_UART_RxCpltCallback(void);
void CommTest_UART_TxCpltCallback(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int system_armed = 0;      // Safety Flag

/* HC-05 update command receiver. USART1 is PA9 (TX) / PA10 (RX). */
static uint8_t update_rx_byte;
static char update_command[16];
static uint8_t update_command_index;

/* Stop every ESC signal before handing control to the STM32 ROM bootloader. */
void ROV_ForceNeutralOutputs(void)
{
    system_armed = 0;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1500);
}

static void ROV_StopMotors(void)
{
    GPIO_InitTypeDef gpio = {0};

    ROV_ForceNeutralOutputs();

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);

    /* Sensor-only safety lock: ESC signal pins are ordinary low outputs,
       not timer alternate-function outputs. */
    HAL_GPIO_WritePin(GPIOA,
                      GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9,
                      GPIO_PIN_RESET);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* Jump to the factory STM32 USART bootloader at system-memory address. */
static void JumpToSystemBootloader(void)
{
    const uint32_t bootloader_base = 0x1FFF0000UL; /* STM32F411 system memory */
    uint32_t boot_msp;
    uint32_t boot_reset_handler;

    ROV_StopMotors();
    HAL_UART_DeInit(&huart1);

    /* The watchdog returns to the ROV firmware if a flashing session aborts. */
    IWDG->KR  = 0xCCCC;
    IWDG->KR  = 0x5555;
    IWDG->PR  = 0x03;
    IWDG->RLR = 0xAA0;
    IWDG->KR  = 0xAAAA;

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0) { }
    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != 0) { }
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON);
    RCC->CFGR = 0;

    __disable_irq();
    for (uint32_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    for (volatile uint32_t i = 0; i < 16000; i++) { }

    boot_msp = *(volatile uint32_t *)bootloader_base;
    boot_reset_handler = *(volatile uint32_t *)(bootloader_base + 4U);
    SCB->VTOR = bootloader_base;
    __DSB();
    __ISB();
    __set_MSP(boot_msp);
    ((void (*)(void))boot_reset_handler)();

    while (1) { }
}

void Set_Motor_PWM(TIM_HandleTypeDef *htim, uint32_t channel, int pwm) {
    __HAL_TIM_SET_COMPARE(htim, channel, pwm);
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
#if (!APP_MOTOR_OUTPUTS_ENABLED) && (!APP_MOTOR_NEUTRAL_TEST)
  ROV_StopMotors();
#endif

  /* Keep this in a USER CODE section: CubeMX regeneration will not remove it. */
  if (HAL_UART_Receive_IT(&huart1, &update_rx_byte, 1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
#if !COMM_TEST_MODE
  if (MS5837_Init(&depth_sensor, &hi2c3, MS5837_OSR_4096) != MS5837_OK) {
    Error_Handler();
    } // Initalize the depth sensor

    /***********************************************************************
     * !!! IMPORTANT – IMU I2C BUS SELECTION !!!
     * The IMU (MPU6050) is initialised below using hi2c1.
     * Make sure this matches the actual I2C peripheral the IMU is wired to.
     * If your IMU is on I2C2, change &hi2c1 → &hi2c2 (and declare hi2c2).
     * If your IMU is on I2C3, change &hi2c1 → &hi2c3.
//     ***********************************************************************/
//    if (MPU6050_Init(&mpu6050, &hi2c1, MPU6050_ADDR) != HAL_OK) {
//        // IMU init failed – optionally handle error
//    }

  	PIDSourceInit(&err_depth, &out_depth, &pid_depth);
  	PIDGainInit(0.02, 1.0, 1.0, 100.0, 0.2, 0.0, 0.0, 10.0, &pid_depth);
    PIDDelayInit(&pid_depth); // initialize depth PID
//
//    // Initialize pitch PID (tune gains as needed)
//    PIDSourceInit(&err_pitch, &out_pitch, &pid_pitch);
//    PIDGainInit(0.02, 1.0, 1.0, 200.0, 0.3, 0.0, 0.0, 10.0, &pid_pitch);
//    PIDDelayInit(&pid_pitch); // initialize pitch PID
//
//    PIDSourceInit(&err_roll, &out_roll, &pid_roll);
//    PIDGainInit(0.02, 1.0, 1.0, 200.0, 0.3, 0.0, 0.0, 10.0, &pid_roll);
//    PIDDelayInit(&pid_roll); // initialize roll PID



    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // PA0
//    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // PA1
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PA6
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PA7
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // PB0
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // PB1
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // PB0
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // PB1
#endif


    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		  // --- IMU reading (DMA-based MPU6050) ---
//		  MPU6050_ReadAll_DMA(&mpu6050);   // kick off non-blocking DMA read
//		  MPU6050_ProcessData(&mpu6050);   // process if DMA data is ready

		  static uint32_t last_time = 0;
		  	    if ((HAL_GetTick() - last_time) >= 100) {
		  	        if (MS5837_GetState(&depth_sensor) == MS5837_STATE_IDLE) {
		  	            MS5837_StartReading(&depth_sensor);
		  	            last_time = HAL_GetTick();
		  	        }
		  	    }

		  	    MS5837_Process(&depth_sensor);

		  	    if (MS5837_IsDataReady(&depth_sensor)) {
		  	         pressure = MS5837_GetPressure(&depth_sensor);
		  	         temp = MS5837_GetTemperature(&depth_sensor);
		  	         depth = MS5837_GetDepth(&depth_sensor);
		  	    	 current_depth = MS5837_GetDepth(&depth_sensor);
		  	    }
		  	  if (system_armed)
		  	  	    {

//		  	  	    	err_depth = target_depth - current_depth;
//		  	  	    	PID(&pid_depth);
		  	  	    // ============================
		  	  	    // DEPTH PID CONTROL ONLY
		  	  	    // ============================

		  	  	    // Calculate depth error
		  	  	    err_depth = target_depth - current_depth;

		  	  	    // PID calculation
		  	  	    PID(&pid_depth);


		  	  	    // Convert PID output to PWM correction
		  	  	    int u_depth = (int)(out_depth * 2);


		  	  	    // Dead zone compensation
		  	  	    if(u_depth > 0)
		  	  	    {
		  	  	        u_depth += 80;
		  	  	    }
		  	  	    else if(u_depth < 0)
		  	  	    {
		  	  	        u_depth -= 80;
		  	  	    }


		  	  	    // Limit PID output
		  	  	    if(u_depth > 300)
		  	  	        u_depth = 300;

		  	  	    if(u_depth < -300)
		  	  	        u_depth = -300;


		  	  	    int base = 1500;


		  	  	    // ============================
		  	  	    // VERTICAL MOTOR MIXING
		  	  	    // DEPTH ONLY
		  	  	    // ============================

		  	  	    int pwm_FL = base + u_depth;
		  	  	    int pwm_FR = base + u_depth;
		  	  	    int pwm_RL = base + u_depth;
		  	  	    int pwm_RR = base + u_depth;


		  	  	    // PWM safety limit
		  	  	    if(pwm_FL > 1800) pwm_FL = 1800;
		  	  	    if(pwm_FL < 1200) pwm_FL = 1200;

		  	  	    if(pwm_FR > 1800) pwm_FR = 1800;
		  	  	    if(pwm_FR < 1200) pwm_FR = 1200;

		  	  	    if(pwm_RL > 1800) pwm_RL = 1800;
		  	  	    if(pwm_RL < 1200) pwm_RL = 1200;

		  	  	    if(pwm_RR > 1800) pwm_RR = 1800;
		  	  	    if(pwm_RR < 1200) pwm_RR = 1200;


		  	  	    // Send PWM
		  	  	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_FL);
		  	  	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_FR);

		  	  	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_RL);

		  	  	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_RR);

//		  	  	    	// --- Pitch PID (uses MPU6050 pitch) ---
//		  	  	    	err_pitch = target_pitch - MPU6050_GetPitch(&mpu6050);
//		  	  	    	PID(&pid_pitch);
//
//		  	  	    	// --- Roll PID (uses MPU6050 roll) ---
//		  	  	    	err_roll = target_roll - MPU6050_GetRoll(&mpu6050);
//		  	  	    	PID(&pid_roll);

//		  	  		      int base = 1500;
//
//		  	  		      // --- Depth PID output ---
//		  	  		      int u_depth = 200;
//
//		  	  		      //   Dead zone compensation
//		  	  		      if (u_depth > 0) u_depth += 100;
//		  	  		      if (u_depth < 0) u_depth -= 100;
//		  	  		      // Limit
//		  	  		      if (u_depth > 300) u_depth = 300;
//		  	  		      if (u_depth < -300) u_depth = -300;

//		  	  		      // --- Pitch PID output ---
//		  	  		      int u_pitch = (int)(out_pitch * 2);
//
//		  	  		      // Pitch dead zone compensation
//		  	  		      if (u_pitch > 0) u_pitch += 50;
//		  	  		      if (u_pitch < 0) u_pitch -= 50;
//		  	  		      // Pitch limit
//		  	  		      if (u_pitch > 200) u_pitch = 200;
//		  	  		      if (u_pitch < -200) u_pitch = -200;
//
//                           // ----- roll PID output -----
//                          int u_roll = (int)(out_roll * 2);
//                          // Roll dead zone compensation
//                          if (u_roll > 0) u_roll += 50;
//                          if (u_roll < 0) u_roll -= 50;
//                               // Roll limit
//                          if (u_roll > 200) u_roll = 200;
//                          if (u_roll < -200) u_roll = -200;


		  	  		      /***********************************************************************
		  	  		       * VERTICAL MOTOR MIXING: depth + pitch + roll
		  	  		       *
		  	  		       *   u_depth : common-mode (all motors together for heave)
		  	  		       *   u_pitch : differential  (front vs rear for pitch torque)
		  	  		       *    u_roll  : differential  (left vs right for roll torque)
		  	  		       *   Positive pitch error = nose up  → push front down, rear up
		  	  		       *     Front motors: subtract u_pitch (more thrust downward)
		  	  		       *     Rear  motors: add u_pitch      (less thrust downward)
		  	  		       *
		  	  		       * !!! FIRST TEST: If pitch correction is reversed, swap the
		  	  		       *     +/- signs on u_pitch below. !!!
		  	  		       ***********************************************************************/
		  	  		      // FR (Front Right)


//		  	  		      int pwm_FR = base + u_depth ;
//
//		  	  		      // FL (Front Left)
//		  	  		      int pwm_FL = base + u_depth ;
//
//		  	  		      // RL (Rear Left)
//		  	  		      int pwm_RL = base - u_depth ;
//
//		  	  		      // RR (Rear Right)
//		  	  		      int pwm_RR = base - u_depth ;
//		  	  	//
//		  	  	//	      // --- E. SAFETY CLAMP ---
//		  	  		      if(pwm_FL > 1900) pwm_FL = 1900;
//		  	  		      if(pwm_FL < 1100) pwm_FL = 1100;
//		  	  		      if(pwm_FR > 1900) pwm_FR = 1900;
//		  	  		      if(pwm_FR < 1100) pwm_FR = 1100;
//		  	  		      if(pwm_RL > 1900) pwm_RL = 1900;
//		  	  		      if(pwm_RL < 1100) pwm_RL = 1100;
//		  	  		      if(pwm_RR > 1900) pwm_RR = 1900;
//		  	  		      if(pwm_RR < 1100) pwm_RR = 1100;
//		  	              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_FL);
//		  	              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_FR);
//	     //	  	          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_RL);
//	  //	  	          __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4,pwm_RR );
//		  	              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_RL);
//		  	              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_RR);



		  	        // ===== HORIZONTAL MOTORS (forward/backward) =====
		  	        	  	  		      // Only enabled after depth has stabilized
		  	        	  	  		      /***********************************************************************
		  	        	  	  		       * !!! IMPORTANT – VERIFY HORIZONTAL MOTOR DIRECTIONS !!!
		  	        	  	  		       * The +/- signs below assume a standard 45° X-configuration:
		  	        	  	  		       *   O_FL & O_RR spin one way  → "+u_fwd" (normal)
		  	        	  	  		       *   O_FR & O_RL spin opposite → "-u_fwd" (flipped)
		  	        	  	  		       *
		  	        	  	  		       * On FIRST TEST: set forward_input = +200 and observe the ROV.
		  	        	  	  		       *   - If it moves BACKWARD instead of forward, swap ALL signs
		  	        	  	  		       *     (i.e. change +u_fwd to -u_fwd and vice-versa).
		  	        	  	  		       *   - If it spins/yaws instead of translating, swap signs on
		  	        	  	  		       *     only the diagonal pair that is wrong.
		  	        	  	  		       ***********************************************************************/
//		  	        	  	  		      if ((err_depth < 0.05f) && (err_depth > -0.05f)) {
//		  	        	  	  		          int u_fwd = forward_input; // range: -500 to +500
//
//		  	        	  	  		          // O_FL (htim3 CH1) - normal direction
//		  	        	  	  		          int pwm_O_FL = base + u_fwd;
//		  	        	  	  		          // O_FR (htim2 CH2) - flipped direction
//		  	        	  	  		          int pwm_O_FR = base + u_fwd;
//		  	        	  	  		          // O_RL (htim3 CH3) - flipped direction
//		  	        	  	  		          int pwm_O_RL = base - u_fwd;
//		  	        	  	  		          // O_RR (htim3 CH2) - normal direction
//		  	        	  	  		          int pwm_O_RR = base + u_fwd;
//
//		  	        	  	  		          // --- HORIZONTAL SAFETY CLAMP ---
//		  	        	  	  		          if(pwm_O_FL > 1900) pwm_O_FL = 1700;
//		  	        	  	  		          if(pwm_O_FL < 1100) pwm_O_FL = 1200;
//		  	        	  	  		          if(pwm_O_FR > 1900) pwm_O_FR = 1700;
//	  	         	  	  		              if(pwm_O_FR < 1100) pwm_O_FR = 1200;
//		  	        	  	  		          if(pwm_O_RL > 1900) pwm_O_RL = 1700;
//		  	        	  	  		          if(pwm_O_RL < 1100) pwm_O_RL = 1200;
//		  	        	  	  		          if(pwm_O_RR > 1900) pwm_O_RR = 1700;
//		  	        	  	  		          if(pwm_O_RR < 1100) pwm_O_RR = 1200;
//
//		  	        	  	  		          // --- WRITE HORIZONTAL MOTORS ---
//		  	        	  	  		          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_O_FL);  // O_FL
//		  	        	  	  		          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_O_FR);  // O_FR
//		  	        	  	  		          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_O_RL);  // O_RL
//		  	        	  	  		          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_O_RR);  // O_RR
//	 	        	  	  		              }
//		  	        	  	  		      else {
//		//  	        	  	  		          else {
//	//	  	        	  	  		          // Depth NOT stable yet → hold horizontal motors at neutral
//		  	        	  	  		          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);  // O_FL
//		  	        	  	  		          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);  // O_FR
//		  	        	  	  		          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);  // O_RL
//		  	        	  	  		          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);  // O_RR
//		  	        	  	  		      }
	//

		  	  	    }
		  	  	    else{

		  	  	    	pid_depth.s_delay = 0;
		  	  	    	out_depth = 0;
		  	  	    for (int i = 0; i <= 2; i++){
		  	  	  	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1500);
		  	  	  	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1500);
		  	  	  	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
		  	  	  	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);
		  	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);
		  	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);
		  	    	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
		  	    	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500);

		  	  	  	HAL_Delay(2000);


		  	  	  	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1700);
		  	  	  	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1700);
		  	  	  	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1700);
		  	  	  	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1700);
		  	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1700);
		  	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1700);
	     	    	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1700);
	    	    	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1700);
	    	    	HAL_Delay(2000);
	                }
		  		  	  	system_armed = 1;
//		  		  	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);  // O_FL
//		  		    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500);  // O_FR
//		  		  	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);  // O_RL
//		  		  	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);  // O_RR
		  	  	    }
    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// DMA Rx complete callback – routes to MPU6050 driver
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    MPU6050_DMA_CompleteCallback(&mpu6050, hi2c);
}

/* Receive the ASCII command "UP" followed by Enter from STM32ProgrammerApp. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        CommTest_UART_RxCpltCallback();
        return;
    }

    if (huart->Instance != USART1) {
        return;
    }

    /* PC13 LED is active-low: toggle on every received byte for diagnosis. */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    if ((update_rx_byte == '\r') || (update_rx_byte == '\n')) {
        update_command[update_command_index] = '\0';
        if (strcmp(update_command, "UP") == 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            JumpToSystemBootloader();
        }
        update_command_index = 0U;
    } else if (update_command_index < (sizeof(update_command) - 1U)) {
        update_command[update_command_index++] = (char)update_rx_byte;
        update_command[update_command_index] = '\0';

        /* Accept UP immediately; an Enter key is optional. */
        if (strcmp(update_command, "UP") == 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            JumpToSystemBootloader();
        }
    } else {
        update_command_index = 0U;
    }

    (void)HAL_UART_Receive_IT(&huart1, &update_rx_byte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        CommTest_UART_TxCpltCallback();
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (MS5837_Init(&depth_sensor, &hi2c3, MS5837_OSR_4096))
  {

  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

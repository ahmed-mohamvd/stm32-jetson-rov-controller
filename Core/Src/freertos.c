/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_mode.h"
#include "comm_interface.h"
#include "comm_encoder.h"
#include "comm_encoder_selftest.h"
#include "comm_parser.h"
#include "comm_parser_selftest.h"
#include "i2c.h"
#include "mpu6050.h"
#include "ms5837_dma.h"
#include "movement.h"
#include "usart.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t motor_index;
  int8_t direction;
} MotorTestRequest_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_TASK_PERIOD_MS          10U
#define IMU_SAMPLE_PERIOD_MS           20U
#define IMU_STALE_TIMEOUT_MS          500U
#define MAG_SAMPLE_PERIOD_MS           20U
#define MAG_STALE_TIMEOUT_MS          500U
#define DEPTH_STALE_TIMEOUT_MS       1000U
#define SENSOR_FAULT_TIMEOUT_MS      2000U
#define SENSOR_RETRY_PERIOD_MS       1000U
#define DEPTH_SURFACE_SAMPLE_COUNT     20U
#define WATER_DENSITY_KG_M3          997.0f
#define STANDARD_GRAVITY_M_S2          9.80665f
#define CONTROL_SAFETY_PERIOD_MS       20U
#define COMMAND_LINK_TIMEOUT_MS       500U
#define SINGLE_MOTOR_TEST_DURATION_MS  250U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static QueueHandle_t commUartRxQueueHandle;
static QueueHandle_t motorTestQueueHandle;
static uint8_t commUartRxByte;
static SemaphoreHandle_t commUartTxMutexHandle;
static SemaphoreHandle_t commUartTxDoneSemaphoreHandle;
static osThreadId_t telemetryTaskHandle;
static osThreadId_t sensorTaskHandle;
static osThreadId_t controlSafetyTaskHandle;
extern MPU6050_t mpu6050;
extern MS5837_t depth_sensor;
static HW290_Magnetometer_t hw290Magnetometer;
static const osThreadAttr_t telemetryTaskAttributes = {
  .name = "telemetryTask",
  .stack_size = 256U * 4U,
  .priority = (osPriority_t)osPriorityLow,
};
static const osThreadAttr_t sensorTaskAttributes = {
  .name = "sensorTask",
  .stack_size = 512U * 4U,
  .priority = (osPriority_t)osPriorityLow,
};
static const osThreadAttr_t controlSafetyTaskAttributes = {
  .name = "controlSafetyTask",
  .stack_size = 256U * 4U,
  .priority = (osPriority_t)osPriorityAboveNormal,
};

/* USER CODE END Variables */
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void StartTelemetryTask(void *argument);
static void StartSensorTask(void *argument);
static void StartControlSafetyTask(void *argument);
static void CommTest_UART_SendText(const uint8_t *data, uint16_t length);
static int16_t Sensor_DegreesToCentidegrees(float degrees);
static int32_t Sensor_MetresToMillimetres(float metres);
/* USER CODE END FunctionPrototypes */

void StartLedTask(void *argument);
void StartUartTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Protect the command and telemetry snapshots shared by RTOS tasks. */
  if (!CommInterface_Init())
  {
    Error_Handler();
  }

  /* Serialize diagnostic text and asynchronous telemetry on USART6 TX. */
  commUartTxMutexHandle = xSemaphoreCreateMutex();
  if (commUartTxMutexHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USART6 TX-complete interrupt wakes the telemetry task through this. */
  commUartTxDoneSemaphoreHandle = xSemaphoreCreateBinary();
  if (commUartTxDoneSemaphoreHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* The USART6 interrupt puts received bytes here for uartTask to process. */
  commUartRxQueueHandle = xQueueCreate(128U, sizeof(uint8_t));
  motorTestQueueHandle = xQueueCreate(1U, sizeof(MotorTestRequest_t));
  if ((commUartRxQueueHandle == NULL) || (motorTestQueueHandle == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  /* creation of uartTask */
  uartTaskHandle = osThreadNew(StartUartTask, NULL, &uartTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL,
                                    &telemetryTaskAttributes);
  if (telemetryTaskHandle == NULL)
  {
    Error_Handler();
  }

  controlSafetyTaskHandle = osThreadNew(StartControlSafetyTask, NULL,
                                        &controlSafetyTaskAttributes);
  if (controlSafetyTaskHandle == NULL)
  {
    Error_Handler();
  }

#if APP_SENSOR_TASK_ENABLED
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTaskAttributes);
  if (sensorTaskHandle == NULL)
  {
    Error_Handler();
  }
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLedTask */
/**
  * @brief  Function implementing the ledTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  (void)argument;

  /* PC13 is the active-low LED on the Blackpill. */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(500U);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
  static const uint8_t readyMessage[] = "STM32 READY\r\n";
  static const uint8_t pongMessage[] = "PONG\r\n";
  static const uint8_t selfTestPassMessage[] = "SELFTEST PASS\r\n";
  static const uint8_t encoderTestPassMessage[] = "ENCODERTEST PASS\r\n";
  static const uint8_t mixerTestPassMessage[] = "MIXERTEST PASS\r\n";
  static const uint8_t pwmTestPassMessage[] = "PWMTEST PASS\r\n";
#if APP_SINGLE_MOTOR_TEST_ENABLED
  static const uint8_t motorTestQueuedMessage[] = "MOTORTEST QUEUED\r\n";
  static const uint8_t motorTestBusyMessage[] = "MOTORTEST BUSY\r\n";
#endif
  static const uint8_t errorMessage[] = "UNKNOWN\r\n";
  static const uint8_t commandCrcError[] = "CMD ERROR CRC\r\n";
  static const uint8_t commandVersionError[] = "CMD ERROR VERSION\r\n";
  static const uint8_t commandTypeError[] = "CMD ERROR TYPE\r\n";
  static const uint8_t commandLengthError[] = "CMD ERROR LENGTH\r\n";
  static const uint8_t commandRangeError[] = "CMD ERROR RANGE\r\n";
  static const uint8_t commandStoreError[] = "CMD ERROR STORE\r\n";
  uint8_t selfTestFailureMessage[] = "SELFTEST FAIL 0\r\n";
  uint8_t encoderTestFailureMessage[] = "ENCODERTEST FAIL 0\r\n";
  uint8_t mixerTestFailureMessage[] = "MIXERTEST FAIL 0\r\n";
  uint8_t pwmTestFailureMessage[] = "PWMTEST FAIL 0\r\n";
  uint8_t commandOkMessage[] = "CMD OK SEQ=00\r\n";
  char command[16];
  size_t commandIndex = 0U;
  uint8_t receivedByte;
  CommParser_t binaryParser;
  ControlCommand_t receivedCommand;
  CommParseResult_t parseResult;
#if APP_SINGLE_MOTOR_TEST_ENABLED
  MotorTestRequest_t motorTestRequest;
#endif

  (void)argument;

  if (commUartRxQueueHandle == NULL)
  {
    Error_Handler();
  }

  CommParser_Init(&binaryParser);

  if (HAL_UART_Receive_IT(&huart6, &commUartRxByte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  CommTest_UART_SendText(readyMessage, sizeof(readyMessage) - 1U);

  for(;;)
  {
    if (xQueueReceive(commUartRxQueueHandle, &receivedByte, portMAX_DELAY) != pdPASS)
    {
      continue;
    }

    /* Once 0xAA is seen, route all bytes to the binary packet parser until
       that frame is complete or rejected. ASCII PING/SELFTEST remain useful
       while the parser is waiting for a new binary frame. */
    if ((binaryParser.state != COMM_PARSER_WAIT_SOF_1) ||
        (receivedByte == COMM_SOF_1))
    {
      parseResult = CommParser_PushByte(&binaryParser, receivedByte,
                                        HAL_GetTick(), &receivedCommand);
      commandIndex = 0U;

      if (parseResult == COMM_PARSE_COMMAND_READY)
      {
        uint16_t knownFlags = COMM_COMMAND_ARM_REQUEST |
                              COMM_COMMAND_DEPTH_HOLD_ENABLE |
                              COMM_COMMAND_EMERGENCY_STOP;
        bool commandInRange =
            ((receivedCommand.control_flags & (uint16_t)(~knownFlags)) == 0U) &&
            (receivedCommand.surge_permille >= -1000) &&
            (receivedCommand.surge_permille <= 1000) &&
            (receivedCommand.sway_permille >= -1000) &&
            (receivedCommand.sway_permille <= 1000) &&
            (receivedCommand.heave_permille >= -1000) &&
            (receivedCommand.heave_permille <= 1000) &&
            (receivedCommand.yaw_permille >= -1000) &&
            (receivedCommand.yaw_permille <= 1000);

        if (!commandInRange)
        {
          CommTest_UART_SendText(commandRangeError,
                                 sizeof(commandRangeError) - 1U);
        }
        else if (!CommInterface_SetCommand(&receivedCommand,
                                           pdMS_TO_TICKS(10U)))
        {
          CommTest_UART_SendText(commandStoreError,
                                 sizeof(commandStoreError) - 1U);
        }
        else
        {
          static const char hexDigits[] = "0123456789ABCDEF";
          commandOkMessage[11] =
              (uint8_t)hexDigits[(receivedCommand.sequence >> 4) & 0x0FU];
          commandOkMessage[12] =
              (uint8_t)hexDigits[receivedCommand.sequence & 0x0FU];
          CommTest_UART_SendText(commandOkMessage,
                                 sizeof(commandOkMessage) - 1U);
        }
      }
      else if (parseResult == COMM_PARSE_ERROR_CRC)
      {
        CommTest_UART_SendText(commandCrcError,
                               sizeof(commandCrcError) - 1U);
      }
      else if (parseResult == COMM_PARSE_ERROR_VERSION)
      {
        CommTest_UART_SendText(commandVersionError,
                               sizeof(commandVersionError) - 1U);
      }
      else if (parseResult == COMM_PARSE_ERROR_TYPE)
      {
        CommTest_UART_SendText(commandTypeError,
                               sizeof(commandTypeError) - 1U);
      }
      else if (parseResult == COMM_PARSE_ERROR_LENGTH)
      {
        CommTest_UART_SendText(commandLengthError,
                               sizeof(commandLengthError) - 1U);
      }

      continue;
    }

    if (receivedByte == '\r')
    {
      continue;
    }

    if (receivedByte == '\n')
    {
      command[commandIndex] = '\0';

      if (strcmp(command, "PING") == 0)
      {
        CommTest_UART_SendText(pongMessage, sizeof(pongMessage) - 1U);
      }
      else if (strcmp(command, "SELFTEST") == 0)
      {
        uint8_t failedTest = CommParser_RunSelfTest();

        if (failedTest == 0U)
        {
          CommTest_UART_SendText(selfTestPassMessage,
                                 sizeof(selfTestPassMessage) - 1U);
        }
        else
        {
          selfTestFailureMessage[14] = (uint8_t)('0' + failedTest);
          CommTest_UART_SendText(selfTestFailureMessage,
                                 sizeof(selfTestFailureMessage) - 1U);
        }
      }
      else if (strcmp(command, "ENCODERTEST") == 0)
      {
        uint8_t failedTest = CommEncoder_RunSelfTest();

        if (failedTest == 0U)
        {
          CommTest_UART_SendText(encoderTestPassMessage,
                                 sizeof(encoderTestPassMessage) - 1U);
        }
        else
        {
          encoderTestFailureMessage[17] = (uint8_t)('0' + failedTest);
          CommTest_UART_SendText(encoderTestFailureMessage,
                                 sizeof(encoderTestFailureMessage) - 1U);
        }
      }
      else if (strcmp(command, "MIXERTEST") == 0)
      {
        uint8_t failedTest = Movement_RunSelfTest();

        if (failedTest == 0U)
        {
          CommTest_UART_SendText(mixerTestPassMessage,
                                 sizeof(mixerTestPassMessage) - 1U);
        }
        else
        {
          mixerTestFailureMessage[15] = (uint8_t)('0' + failedTest);
          CommTest_UART_SendText(mixerTestFailureMessage,
                                 sizeof(mixerTestFailureMessage) - 1U);
        }
      }
      else if (strcmp(command, "PWMTEST") == 0)
      {
        uint8_t failedTest = Movement_RunPwmNeutralTest();

        if (failedTest == 0U)
        {
          CommTest_UART_SendText(pwmTestPassMessage,
                                 sizeof(pwmTestPassMessage) - 1U);
        }
        else
        {
          pwmTestFailureMessage[13] = (uint8_t)('0' + failedTest);
          CommTest_UART_SendText(pwmTestFailureMessage,
                                 sizeof(pwmTestFailureMessage) - 1U);
        }
      }
#if APP_SINGLE_MOTOR_TEST_ENABLED
      else if ((commandIndex == 3U) && (command[0] == 'M') &&
               (command[1] >= '1') && (command[1] <= '8') &&
               ((command[2] == '+') || (command[2] == '-')))
      {
        motorTestRequest.motor_index = (uint8_t)(command[1] - '1');
        motorTestRequest.direction = (command[2] == '+') ? 1 : -1;

        if (xQueueSend(motorTestQueueHandle, &motorTestRequest, 0U) == pdPASS)
        {
          CommTest_UART_SendText(motorTestQueuedMessage,
                                 sizeof(motorTestQueuedMessage) - 1U);
        }
        else
        {
          CommTest_UART_SendText(motorTestBusyMessage,
                                 sizeof(motorTestBusyMessage) - 1U);
        }
      }
#endif
      else if (commandIndex > 0U)
      {
        CommTest_UART_SendText(errorMessage, sizeof(errorMessage) - 1U);
      }

      commandIndex = 0U;
    }
    else if (commandIndex < (sizeof(command) - 1U))
    {
      command[commandIndex++] = (char)receivedByte;
    }
    else
    {
      /* Discard a line that is longer than the command buffer. */
      commandIndex = 0U;
    }
  }
  /* USER CODE END StartUartTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void CommTest_UART_SendText(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U) || (commUartTxMutexHandle == NULL))
  {
    return;
  }

  if (xSemaphoreTake(commUartTxMutexHandle, pdMS_TO_TICKS(20U)) == pdTRUE)
  {
    (void)HAL_UART_Transmit(&huart6, (uint8_t *)data, length, 100U);
    (void)xSemaphoreGive(commUartTxMutexHandle);
  }
}

static void StartTelemetryTask(void *argument)
{
  TelemetrySnapshot_t telemetry = {
    .uptime_ms = 0U,
    .depth_mm = 0,
    .roll_cdeg = 0,
    .pitch_cdeg = 0,
    .yaw_cdeg = 0,
    .status_flags = 0U,
    .last_command_sequence = 0U
  };
  ControlCommand_t latestCommand;
  uint8_t frame[COMM_TELEMETRY_FRAME_SIZE];
  uint8_t sequence = 0U;
  uint16_t frameLength;
  bool linkOk;
  bool failsafeActive;
  TickType_t lastWakeTime;

  (void)argument;

  /* Sensor fields remain zero and invalid until a sensor reader updates them. */
  if (!CommInterface_SetTelemetry(&telemetry, pdMS_TO_TICKS(10U)))
  {
    Error_Handler();
  }

  lastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100U));

    if (!CommInterface_GetTelemetry(&telemetry, pdMS_TO_TICKS(10U)))
    {
      continue;
    }

    telemetry.uptime_ms = HAL_GetTick();
    if (CommInterface_GetCommand(&latestCommand, 0U))
    {
      telemetry.last_command_sequence = latestCommand.sequence;
    }

    telemetry.status_flags &=
        (uint16_t)~(COMM_STATUS_ARMED |
                    COMM_STATUS_COMMAND_LINK_OK |
                    COMM_STATUS_FAILSAFE_ACTIVE);

    if (CommInterface_GetSafetyState(&linkOk, &failsafeActive, 0U))
    {
      if (linkOk)
      {
        telemetry.status_flags |= COMM_STATUS_COMMAND_LINK_OK;
      }

      if (failsafeActive)
      {
        telemetry.status_flags |= COMM_STATUS_FAILSAFE_ACTIVE;
      }

#if APP_MOTOR_OUTPUTS_ENABLED
      if (linkOk && (!failsafeActive) &&
          ((latestCommand.control_flags & COMM_COMMAND_ARM_REQUEST) != 0U))
      {
        telemetry.status_flags |= COMM_STATUS_ARMED;
      }
#endif
    }

    frameLength = CommEncoder_BuildTelemetry(&telemetry, sequence, frame,
                                             sizeof(frame));
    if (frameLength == 0U)
    {
      continue;
    }

    if (xSemaphoreTake(commUartTxMutexHandle, pdMS_TO_TICKS(20U)) != pdTRUE)
    {
      continue;
    }

    /* Remove a stale completion token before starting a new transfer. */
    (void)xSemaphoreTake(commUartTxDoneSemaphoreHandle, 0U);

    if (HAL_UART_Transmit_IT(&huart6, frame, frameLength) == HAL_OK)
    {
      ++sequence;
      if (xSemaphoreTake(commUartTxDoneSemaphoreHandle,
                         pdMS_TO_TICKS(20U)) != pdTRUE)
      {
        (void)HAL_UART_AbortTransmit(&huart6);
      }
    }

    (void)xSemaphoreGive(commUartTxMutexHandle);
  }
}

static void StartControlSafetyTask(void *argument)
{
  ControlCommand_t activeCommand;
  MotorOutputs_t motorOutputs;
#if APP_SINGLE_MOTOR_TEST_ENABLED
  MotorTestRequest_t motorTestRequest;
  uint32_t motorTestEndMs = 0U;
#endif
  TickType_t lastWakeTime;
  bool linkOk;
  bool failsafeActive;
  bool commandSafe;
#if APP_SINGLE_MOTOR_TEST_ENABLED
  bool motorTestActive = false;
#endif

  (void)argument;
  Movement_SetNeutral(&motorOutputs);

#if APP_MOTOR_OUTPUTS_ENABLED || APP_MOTOR_NEUTRAL_TEST
  /* ESCs see neutral only during their startup/arming interval. */
  if (!Movement_StartPwmNeutral())
  {
    Error_Handler();
  }
  osDelay(3000U);
#endif

  lastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    commandSafe = false;

    if (CommInterface_UpdateCommandSafety(HAL_GetTick(),
                                          COMMAND_LINK_TIMEOUT_MS,
                                          pdMS_TO_TICKS(10U)) &&
        CommInterface_GetSafetyState(&linkOk, &failsafeActive,
                                     pdMS_TO_TICKS(10U)) &&
        CommInterface_GetCommand(&activeCommand, pdMS_TO_TICKS(10U)) &&
        Movement_Compute(&activeCommand, &motorOutputs))
    {
      commandSafe = linkOk && (!failsafeActive);
    }

    if (!commandSafe)
    {
      Movement_SetNeutral(&motorOutputs);
    }

#if APP_MOTOR_OUTPUTS_ENABLED
    /* A validation-build hardware boundary: never exceed the configured
       low-power PWM delta even if a valid packet requests full thrust. */
    Movement_LimitPwmDelta(&motorOutputs, APP_MOTOR_OUTPUT_LIMIT_US);
    Movement_Apply(&motorOutputs);
#elif APP_MOTOR_NEUTRAL_TEST
    /* Binary motion commands remain blocked in hardware validation mode. */
    Movement_SetNeutral(&motorOutputs);

#if APP_SINGLE_MOTOR_TEST_ENABLED
    if (motorTestActive &&
        ((int32_t)(HAL_GetTick() - motorTestEndMs) >= 0))
    {
      motorTestActive = false;
    }

    if ((!motorTestActive) &&
        (xQueueReceive(motorTestQueueHandle, &motorTestRequest, 0U) == pdPASS))
    {
      motorTestActive = true;
      motorTestEndMs = HAL_GetTick() + SINGLE_MOTOR_TEST_DURATION_MS;
    }

    if (motorTestActive)
    {
      (void)Movement_SetSingleMotorTest(&motorOutputs,
                                        motorTestRequest.motor_index,
                                        motorTestRequest.direction);
    }
#endif

    Movement_Apply(&motorOutputs);
#else
    (void)motorOutputs;
#endif

    vTaskDelayUntil(&lastWakeTime,
                    pdMS_TO_TICKS(CONTROL_SAFETY_PERIOD_MS));
  }
}

static void StartSensorTask(void *argument)
{
  TelemetrySnapshot_t telemetry;
  uint32_t nowMs;
  uint32_t lastImuStartMs = 0U;
  uint32_t lastImuDataMs = 0U;
  uint32_t lastMagReadMs = 0U;
  uint32_t lastMagDataMs = 0U;
  uint32_t lastDepthDataMs = 0U;
  uint32_t lastImuRetryMs;
  uint32_t lastMagRetryMs;
  uint32_t lastDepthRetryMs;
  uint16_t surfaceSampleCount = 0U;
  uint8_t imuAddress = MPU6050_ADDR;
  bool imuDetected = false;
  bool magDetected = false;
  bool depthDetected = false;
  bool imuInitialized = false;
  bool magInitialized = false;
  bool depthInitialized = false;
  bool imuValid = false;
  bool magValid = false;
  bool depthValid = false;
  HAL_StatusTypeDef magReadStatus;
  float surfacePressureSum = 0.0f;
  float surfacePressureMbar = 0.0f;
  float pressureMbar;
  float temperatureC;
  TickType_t lastWakeTime;

  (void)argument;

  /* Let power and the shared telemetry snapshot settle before sensor reset. */
  osDelay(100U);
  nowMs = HAL_GetTick();
  lastImuRetryMs = nowMs - SENSOR_RETRY_PERIOD_MS;
  lastMagRetryMs = nowMs - SENSOR_RETRY_PERIOD_MS;
  lastDepthRetryMs = nowMs - SENSOR_RETRY_PERIOD_MS;
  lastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    nowMs = HAL_GetTick();

    /* Detect and initialize the IMU. AD0 selects either 0x68 or 0x69. */
    if ((!imuInitialized) &&
        ((nowMs - lastImuRetryMs) >= SENSOR_RETRY_PERIOD_MS))
    {
      lastImuRetryMs = nowMs;
      imuDetected = false;

      if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 2U, 20U) == HAL_OK)
      {
        imuAddress = MPU6050_ADDR;
        imuDetected = true;
      }
      else if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR_ALT,
                                     2U, 20U) == HAL_OK)
      {
        imuAddress = MPU6050_ADDR_ALT;
        imuDetected = true;
      }

      if (imuDetected &&
          (MPU6050_Init(&mpu6050, &hi2c1, imuAddress) == HAL_OK))
      {
        imuInitialized = true;
        imuValid = false;
        lastImuStartMs = nowMs - IMU_SAMPLE_PERIOD_MS;
        lastImuDataMs = nowMs;
      }
    }

    /* Process a completed MPU6050 DMA sample before sharing I2C1. */
    if (imuInitialized)
    {
      if (mpu6050.data_ready != 0U)
      {
        MPU6050_ProcessData(&mpu6050);

        if ((mpu6050.roll >= -180.0f) && (mpu6050.roll <= 180.0f) &&
            (mpu6050.pitch >= -90.0f) && (mpu6050.pitch <= 90.0f))
        {
          imuValid = true;
          lastImuDataMs = nowMs;
        }
      }

      if ((nowMs - lastImuDataMs) > IMU_STALE_TIMEOUT_MS)
      {
        imuValid = false;
      }
    }

    /* The HW-290 magnetometer is exposed on I2C1 through MPU6050 bypass. */
    if (imuInitialized && (!magInitialized) &&
        ((nowMs - lastMagRetryMs) >= SENSOR_RETRY_PERIOD_MS) &&
        (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY))
    {
      lastMagRetryMs = nowMs;
      magDetected =
          (HAL_I2C_IsDeviceReady(&hi2c1, HW290_QMC5883L_ADDR,
                                 2U, 20U) == HAL_OK) ||
          (HAL_I2C_IsDeviceReady(&hi2c1, HW290_HMC5883L_ADDR,
                                 2U, 20U) == HAL_OK);

      if (magDetected &&
          (HW290_Magnetometer_Init(&hw290Magnetometer, &hi2c1) == HAL_OK))
      {
        magInitialized = true;
        magValid = false;
        lastMagReadMs = nowMs - MAG_SAMPLE_PERIOD_MS;
        lastMagDataMs = nowMs;
      }
    }

    if (magInitialized &&
        ((nowMs - lastMagReadMs) >= MAG_SAMPLE_PERIOD_MS) &&
        (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY))
    {
      lastMagReadMs = nowMs;
      magReadStatus = HW290_Magnetometer_Read(&hw290Magnetometer);

      if (magReadStatus == HAL_OK)
      {
        (void)HW290_Magnetometer_ComputeYaw(&hw290Magnetometer,
                                            mpu6050.roll,
                                            mpu6050.pitch);
        magValid = true;
        lastMagDataMs = nowMs;
      }
    }

    if (magInitialized &&
        ((nowMs - lastMagDataMs) > MAG_STALE_TIMEOUT_MS))
    {
      magValid = false;
    }

    /* Start MPU DMA last so no blocking magnetometer access overlaps it. */
    if (imuInitialized &&
        ((nowMs - lastImuStartMs) >= IMU_SAMPLE_PERIOD_MS) &&
        (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) &&
        (MPU6050_ReadAll_DMA(&mpu6050) == HAL_OK))
    {
      lastImuStartMs = nowMs;
    }

    /* Detect, reset and load factory calibration from the depth sensor. */
    if ((!depthInitialized) &&
        ((nowMs - lastDepthRetryMs) >= SENSOR_RETRY_PERIOD_MS))
    {
      lastDepthRetryMs = nowMs;
      depthDetected =
          (HAL_I2C_IsDeviceReady(&hi2c3, MS5837_ADDR, 2U, 20U) == HAL_OK);

      if (depthDetected &&
          (MS5837_Init(&depth_sensor, &hi2c3, MS5837_OSR_4096) == MS5837_OK))
      {
        depthInitialized = true;
        depthValid = false;
        surfaceSampleCount = 0U;
        surfacePressureSum = 0.0f;
        surfacePressureMbar = 0.0f;
        lastDepthDataMs = nowMs;
        (void)MS5837_StartReading(&depth_sensor);
      }
    }

    if (depthInitialized)
    {
      MS5837_Process(&depth_sensor);

      if (MS5837_IsDataReady(&depth_sensor))
      {
        pressureMbar = MS5837_GetPressure(&depth_sensor);
        temperatureC = MS5837_GetTemperature(&depth_sensor);
        depth_sensor.data_ready = false;

        /* Reject impossible compensation results before publishing them. */
        if ((pressureMbar > 100.0f) && (pressureMbar < 35000.0f) &&
            (temperatureC > -40.0f) && (temperatureC < 100.0f))
        {
          lastDepthDataMs = nowMs;

          /* The first twenty samples define zero depth at startup. */
          if (surfaceSampleCount < DEPTH_SURFACE_SAMPLE_COUNT)
          {
            surfacePressureSum += pressureMbar;
            ++surfaceSampleCount;
            depth_sensor.depth = 0.0f;

            if (surfaceSampleCount == DEPTH_SURFACE_SAMPLE_COUNT)
            {
              surfacePressureMbar =
                  surfacePressureSum / (float)DEPTH_SURFACE_SAMPLE_COUNT;
              depthValid = true;
            }
          }
          else
          {
            /* mbar -> Pa, then hydrostatic pressure -> metres of fresh water. */
            depth_sensor.depth =
                ((pressureMbar - surfacePressureMbar) * 100.0f) /
                (WATER_DENSITY_KG_M3 * STANDARD_GRAVITY_M_S2);
            depthValid = true;
          }
        }
        else
        {
          depthValid = false;
        }
      }

      if (MS5837_GetState(&depth_sensor) == MS5837_STATE_IDLE)
      {
        (void)MS5837_StartReading(&depth_sensor);
      }

      if ((nowMs - lastDepthDataMs) > DEPTH_STALE_TIMEOUT_MS)
      {
        depthValid = false;
      }
    }

    if (CommInterface_GetTelemetry(&telemetry, pdMS_TO_TICKS(10U)))
    {
      telemetry.status_flags &=
          (uint16_t)~(COMM_STATUS_IMU_DETECTED |
                      COMM_STATUS_DEPTH_SENSOR_DETECTED |
                      COMM_STATUS_MAGNETOMETER_DETECTED |
                      COMM_STATUS_SENSOR_FAULT |
                      COMM_STATUS_DEPTH_VALID |
                      COMM_STATUS_ATTITUDE_VALID |
                      COMM_STATUS_YAW_VALID);

      if (imuDetected)
      {
        telemetry.status_flags |= COMM_STATUS_IMU_DETECTED;
      }

      if (depthDetected)
      {
        telemetry.status_flags |= COMM_STATUS_DEPTH_SENSOR_DETECTED;
      }

      if (magDetected)
      {
        telemetry.status_flags |= COMM_STATUS_MAGNETOMETER_DETECTED;
      }

      if (imuValid)
      {
        telemetry.roll_cdeg = Sensor_DegreesToCentidegrees(mpu6050.roll);
        telemetry.pitch_cdeg = Sensor_DegreesToCentidegrees(mpu6050.pitch);
        telemetry.status_flags |= COMM_STATUS_ATTITUDE_VALID;
      }
      else
      {
        telemetry.roll_cdeg = 0;
        telemetry.pitch_cdeg = 0;
      }

      if (imuValid && magValid)
      {
        telemetry.yaw_cdeg =
            Sensor_DegreesToCentidegrees(hw290Magnetometer.yaw_deg);
        telemetry.status_flags |= COMM_STATUS_YAW_VALID;
      }
      else
      {
        telemetry.yaw_cdeg = 0;
      }

      if (depthValid)
      {
        telemetry.depth_mm =
            Sensor_MetresToMillimetres(depth_sensor.depth);
        telemetry.status_flags |= COMM_STATUS_DEPTH_VALID;
      }
      else
      {
        telemetry.depth_mm = 0;
      }

      if ((!imuInitialized) || (!magInitialized) || (!depthInitialized) ||
          (imuInitialized &&
           ((nowMs - lastImuDataMs) > SENSOR_FAULT_TIMEOUT_MS)) ||
          (magInitialized &&
           ((nowMs - lastMagDataMs) > SENSOR_FAULT_TIMEOUT_MS)) ||
          (depthInitialized &&
           ((nowMs - lastDepthDataMs) > SENSOR_FAULT_TIMEOUT_MS)))
      {
        telemetry.status_flags |= COMM_STATUS_SENSOR_FAULT;
      }

      (void)CommInterface_SetTelemetry(&telemetry, pdMS_TO_TICKS(10U));
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
  }
}

static int16_t Sensor_DegreesToCentidegrees(float degrees)
{
  float scaled = degrees * 100.0f;

  if (scaled > 32767.0f)
  {
    return 32767;
  }
  if (scaled < -32768.0f)
  {
    return -32768;
  }

  scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
  return (int16_t)scaled;
}

static int32_t Sensor_MetresToMillimetres(float metres)
{
  float scaled = metres * 1000.0f;
  scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
  return (int32_t)scaled;
}

void CommTest_UART_RxCpltCallback(void)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  uint8_t receivedByte = commUartRxByte;

  if (commUartRxQueueHandle != NULL)
  {
    (void)xQueueSendFromISR(commUartRxQueueHandle, &receivedByte,
                            &higherPriorityTaskWoken);
  }

  (void)HAL_UART_Receive_IT(&huart6, &commUartRxByte, 1U);
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void CommTest_UART_TxCpltCallback(void)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (commUartTxDoneSemaphoreHandle != NULL)
  {
    (void)xSemaphoreGiveFromISR(commUartTxDoneSemaphoreHandle,
                                &higherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/* USER CODE END Application */


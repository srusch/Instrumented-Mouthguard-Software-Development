/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/p2p_server_app.c
  * @author  MCD Application Team
  * @brief   Peer to peer Server Application
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IMUFunctions.h"
#include "MagFunctions.h"
#include "Mahony.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// BLE timer
#define CFG_TASK_SENSOR_DATA_SEND_ID    (5)   // Use bit position 5
#define CFG_TIM_PROC_ID_ISR             (0)   // Use timer 0

// Data storage
#define MaxSamples 3000  // 5 minutes at 10Hz

// Sensor data structure
typedef struct{
	float time;
    float accelerationX;
    float accelerationY;
    float accelerationZ;
    float gyroscopeX;
    float gyroscopeY;
    float gyroscopeZ;
    float roll;
    float pitch;
    float yaw;
    float buffer[10];
    char CSVbuffer[256];
}P2P_SensorData;

typedef struct
{
  uint8_t               NotificationStatus;
  P2P_SensorData        SensorData;
  uint16_t              ConnectionHandle;
  uint8_t 				timerID;
} P2P_Server_App_Context;

// Storage variables
typedef struct
{
    float data[9];
} SensorSample;

SensorSample sampleBuffer[MaxSamples];
volatile uint32_t sampleIndex = 0;
volatile uint8_t BufferFull = 0;

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SensorSendPeriod    100  // send sensor data every 100ms
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static P2P_Server_App_Context P2PServer_AppContext;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void P2PS_Send_SensorNotification(void);
static void P2PS_APP_Sensor_Context_Init(void);
void P2PS_ConvertToCSV(P2P_SensorData* sensor_data);
void SendDataOverUART(void);
void ClearDataOverUART(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */

/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
    	P2PServer_AppContext.NotificationStatus = 1;
    	APP_DBG_MSG("-- P2P APPLICATION SERVER : SENSOR NOTIFICATION ENABLED\n");
    	APP_DBG_MSG(" \n\r");
    	ClearDataOverUART();

    	// start periodic sensor data transmission
    	HW_TS_Start(P2PServer_AppContext.timerID, SensorSendPeriod);
/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
    	P2PServer_AppContext.NotificationStatus = 0;
    	APP_DBG_MSG("-- P2P APPLICATION SERVER : SENSOR NOTIFICATION DISABLED\n");
    	APP_DBG_MSG(" \n\r");

    	// stop periodic sensor data transmission
    	HW_TS_Stop(P2PServer_AppContext.timerID);
    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;

    case P2PS_STM_WRITE_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */

/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */

/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */

/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
	  P2PServer_AppContext.ConnectionHandle = pNotification->ConnectionHandle;
	  APP_DBG_MSG("-- P2P APPLICATION SERVER : CONNECTED\n");
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
    	P2PS_APP_Sensor_Context_Init();
    	HW_TS_Stop(P2PServer_AppContext.timerID); // stop timer on disconnect
        APP_DBG_MSG("-- P2P APPLICATION SERVER : DISCONNECTED\n");
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
        SendDataOverUART();
/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;

    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */
	UTIL_SEQ_RegTask( 1<< CFG_TASK_SENSOR_DATA_SEND_ID, UTIL_SEQ_RFU, P2PS_Send_SensorNotification );
	P2PServer_AppContext.NotificationStatus = 0;
	P2PS_APP_Sensor_Context_Init();

	// initialize timer for periodic sensor data sending
	uint8_t timer_ID;
    HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timer_ID, hw_ts_Repeated, P2PS_Sensor_Timer_Callback);
    P2PServer_AppContext.timerID = timer_ID;
/* USER CODE END P2PS_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void P2PS_APP_Sensor_Context_Init(void)
{
  // initialize sensor data structure
	P2PServer_AppContext.SensorData.time = 0.0f;
	P2PServer_AppContext.SensorData.accelerationX = 0.0f;
  	P2PServer_AppContext.SensorData.accelerationY = 0.0f;
	P2PServer_AppContext.SensorData.accelerationZ = 0.0f;
	P2PServer_AppContext.SensorData.gyroscopeX = 0.0f;
    P2PServer_AppContext.SensorData.gyroscopeY = 0.0f;
    P2PServer_AppContext.SensorData.gyroscopeZ = 0.0f;
    P2PServer_AppContext.SensorData.roll = 0.0f;
    P2PServer_AppContext.SensorData.pitch = 0.0f;
    P2PServer_AppContext.SensorData.yaw = 0.0f;
}

// Function to be called by main.c to update sensor data
void P2PS_APP_Update_SensorData(void)
{
	P2PServer_AppContext.SensorData.time = ((float) (IMUDataAverage.timestamp)) / 1000.0f;
	P2PServer_AppContext.SensorData.accelerationX = IMUDataAverage.axAvg;
	P2PServer_AppContext.SensorData.accelerationY = IMUDataAverage.ayAvg;
	P2PServer_AppContext.SensorData.accelerationZ = IMUDataAverage.azAvg;
	P2PServer_AppContext.SensorData.gyroscopeX = IMUDataAverage.gxAvg;
	P2PServer_AppContext.SensorData.gyroscopeY = IMUDataAverage.gyAvg;
	P2PServer_AppContext.SensorData.gyroscopeZ = IMUDataAverage.gzAvg;
    P2PServer_AppContext.SensorData.roll = Orientation.roll;
    P2PServer_AppContext.SensorData.pitch = Orientation.pitch;
    P2PServer_AppContext.SensorData.yaw = Orientation.yaw;

    P2PServer_AppContext.SensorData.buffer[0] = P2PServer_AppContext.SensorData.time;
    P2PServer_AppContext.SensorData.buffer[1] = P2PServer_AppContext.SensorData.accelerationX;
    P2PServer_AppContext.SensorData.buffer[2] = P2PServer_AppContext.SensorData.accelerationY;
    P2PServer_AppContext.SensorData.buffer[3] = P2PServer_AppContext.SensorData.accelerationZ;
    P2PServer_AppContext.SensorData.buffer[4] = P2PServer_AppContext.SensorData.gyroscopeX;
    P2PServer_AppContext.SensorData.buffer[5] = P2PServer_AppContext.SensorData.gyroscopeY;
    P2PServer_AppContext.SensorData.buffer[6] = P2PServer_AppContext.SensorData.gyroscopeZ;
    P2PServer_AppContext.SensorData.buffer[7] = P2PServer_AppContext.SensorData.roll;
    P2PServer_AppContext.SensorData.buffer[8] = P2PServer_AppContext.SensorData.pitch;
    P2PServer_AppContext.SensorData.buffer[9] = P2PServer_AppContext.SensorData.yaw;

    P2PS_ConvertToCSV(&P2PServer_AppContext.SensorData);
}

// timer callback function
void P2PS_Sensor_Timer_Callback(void)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_SENSOR_DATA_SEND_ID, CFG_SCH_PRIO_0);
}

// function to convert data to a CSV line
void P2PS_ConvertToCSV(P2P_SensorData* sensor_data)
{
	int len = snprintf(sensor_data->CSVbuffer, 256,
             "%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f\n",
			 sensor_data->time,
			 sensor_data->accelerationX,
             sensor_data->accelerationY,
             sensor_data->accelerationZ,
             sensor_data->gyroscopeX,
             sensor_data->gyroscopeY,
             sensor_data->gyroscopeZ,
             sensor_data->roll,
             sensor_data->pitch,
             sensor_data->yaw);

    if (len >= 256)
    {
        APP_DBG_MSG("CSV buffer overflow\n");
    }

}

void SendDataOverUART(void)
{
    // column headers
    printf("Time,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Roll,Pitch,Yaw\r\n");

    // send all stored samples
    for (uint32_t i = 0; i < sampleIndex; i++) {
        printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
            sampleBuffer[i].data[0],
            sampleBuffer[i].data[1],
            sampleBuffer[i].data[2],
            sampleBuffer[i].data[3],
            sampleBuffer[i].data[4],
            sampleBuffer[i].data[5],
            sampleBuffer[i].data[6],
            sampleBuffer[i].data[7],
            sampleBuffer[i].data[8],
			sampleBuffer[i].data[9]);

        HAL_Delay(50);
    }

    printf("END - %lu samples dumped\n", sampleIndex);
}

void ClearDataOverUART(void)
{
	sampleIndex = 0;
	BufferFull = 0;

	// set all values in buffer = 0
	memset(sampleBuffer, 0, sizeof(sampleBuffer));

    printf("Samples cleared\n");
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void P2PS_Send_SensorNotification(void)
{
	if(P2PServer_AppContext.NotificationStatus)
	{
		APP_DBG_MSG("-- P2P APPLICATION SERVER : SENDING SENSOR DATA\n");
		APP_DBG_MSG("CSV: %s", P2PServer_AppContext.SensorData.CSVbuffer);

		// send the complete sensor data structure
		P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)P2PServer_AppContext.SensorData.CSVbuffer);

		// save sample in buffer
	    if (sampleIndex < MaxSamples)
	    {
	    	memcpy(sampleBuffer[sampleIndex].data, P2PServer_AppContext.SensorData.buffer, sizeof(float) * 10);
	        sampleIndex++;
	    }

	    else
	    {
	        BufferFull = 1;
	    }
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	}
	else
	{
		APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T SEND SENSOR DATA - NOTIFICATION DISABLED\n");
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
	}

  return;
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/

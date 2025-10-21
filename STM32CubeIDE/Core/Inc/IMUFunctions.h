/*
 * IMUFunctions.h
 *
 *  Created on: Sep 2, 2025
 *      Author: sarah
 */

#ifndef INC_IMUFUNCTIONS_H_
#define INC_IMUFUNCTIONS_H_

#include "stm32wbxx_hal.h"
#include "main.h"

#define IMUAddress		(0x6B << 1)		// fixed address of IMU

// define a sate state machine for the IMU
typedef enum {
	IMU_INITIALIZING,
	IMU_IDLE,
	IMU_ERROR,
	IMU_READING,
	IMU_COMPLETE
} IMU_state;

// define a structure to store the IMU data
typedef struct {
	float ax;
	float ay;
	float az;

	float gx;
	float gy;
	float gz;

	uint32_t timestamp;
} IMU_data;

typedef struct {
	float ax[6];
	float ay[6];
	float az[6];

	float gx[6];
	float gy[6];
	float gz[6];

	float axAvg;
	float ayAvg;
	float azAvg;

	float gxAvg;
	float gyAvg;
	float gzAvg;

	uint32_t timestamp;
} IMU_data_average;

// function prototypes
void IMUWriteB (uint8_t data, uint8_t address);
uint8_t IMUReadB (uint8_t address);
void IMUInit (void);
void IMURead (void);
void IMUGetData (void);
void IMUAverage (void);

// global variables
extern volatile uint8_t IMUDMABusy;				// flag to indicate when IMU DMA is busy
extern volatile uint8_t IMUDataReadyFlag;		// flag to indicate IMU INT1 has triggered
extern volatile uint8_t IMUGetDataFlag;			// flag to indicate IMU DMA for data is complete
extern volatile uint8_t IMUMadgwick;			// flag to indicate when IMU data can be sent to Madgwick filter
extern volatile IMU_state IMUState; 			// structure which stores the current state of the IMU
extern IMU_data IMUData;						// structure which stores the IMU's accelerometer and gyroscope measurements
extern IMU_data_average IMUDataAverage;			// structure which stored the average of 5 IMU readings
extern volatile uint8_t IMUIndex;

#endif /* INC_IMUFUNCTIONS_H_ */

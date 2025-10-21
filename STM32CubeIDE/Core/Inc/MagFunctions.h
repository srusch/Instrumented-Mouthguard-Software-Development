/*
 * MagFunctions.h
 *
 *  Created on: Sep 2, 2025
 *      Author: sarah
 */

#ifndef INC_MAGFUNCTIONS_H_
#define INC_MAGFUNCTIONS_H_

#include "stm32wbxx_hal.h"
#include "main.h"

#define MagAddress		(0x0C << 1)		// fixed address of magnetic sensor

// define a sate state machine for the magnetic sensor
typedef enum {
	MAG_INITIALIZING,
	MAG_IDLE,
	MAG_ERROR,
	MAG_SINGLEMEASUREMENT,
	MAG_READING,
	MAG_COMPLETE,
	MAG_TIMEOUT
} Mag_state;

// define a structure to store the magnetic sensor data
typedef struct {
	float mx;
	float my;
	float mz;

	uint32_t timestamp;
} Mag_data;

typedef struct {
	float mx[5];
	float my[5];
	float mz[5];

	float mxAvg;
	float myAvg;
	float mzAvg;

	uint32_t timestamp;
} Mag_data_average;

// function prototypes
void MagWriteB (uint8_t data, uint8_t address);
uint8_t MagReadB (uint8_t address);
void MagInit (void);
void MagSingleMode (void);
void MagPowerDown (void);
void MagRead(void);
void MagGetData (void);
void MagAverage (void);

// global variables
extern volatile uint8_t MagDMABusy;				// flag to indicate when magnetic sensor DMA is busy
extern volatile uint8_t MagGetDataFlag;			// flag to indicate magnetic sensor DMA for data is complete
extern volatile uint8_t MagSMFlag;				// flag to indicate the magnetic sensor must be put into single measurement mode
extern volatile uint8_t MagMadgwick;			// flag to indicate when magnetic sensor data can be sent to Madgwick filter
extern volatile Mag_state MagState; 			// structure which stores the current state of the magnetic sensor
extern Mag_data MagData;						// structure which stores the magnetic sensor's measurements
extern volatile uint32_t magMeasurementStartTime;
extern Mag_data_average MagDataAverage;			// structure which stored the average of 5 IMU readings
extern volatile uint8_t MagIndex;

#endif /* INC_MAGFUNCTIONS_H_ */

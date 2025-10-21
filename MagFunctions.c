/*
 * MagFunctions.c
 *
 *  Created on: Sep 2, 2025
 *      Author: sarah
 */

#include "MagFunctions.h"
#include "main.h"

// external I2C3 handle
extern I2C_HandleTypeDef hi2c3;

// variable declarations
static uint8_t DMABuffer[6];					// buffer to store data read over DMA

Mag_data MagData;								// structure which stores the IMU's accelerometer and gyroscope measurements
Mag_data_average MagDataAverage;
volatile Mag_state MagState = MAG_IDLE;			// structure which stores the current state of the IMU
volatile uint8_t MagDMABusy = 0;				// flag to indicate when magnetic sensor DMA is busy
volatile uint8_t MagGetDataFlag = 0;			// flag to indicate magnetic sensor DMA for data is complete
volatile uint8_t MagSMFlag = 0;					// flag to indicate the magnetic sensor must be put into single measurement mode
volatile uint8_t MagMadgwick = 0;				// flag to indicate when magnetic sensor data can be sent to Madgwick filter
volatile uint32_t magMeasurementStartTime = 0;
volatile uint8_t MagIndex = 0;

// blocking function to write to the magnetic sensor
void MagWriteB (uint8_t data, uint8_t address)
{
	HAL_I2C_Mem_Write(&hi2c3, MagAddress, address, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

// blocking function to read from the magnetic sensor
uint8_t MagReadB (uint8_t address)
{
	uint8_t data;				// variable to store what is read from magnetic sensor

	HAL_I2C_Mem_Read(&hi2c3, MagAddress, address, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	return data;
}

// function to initialise magnetic sensor (configuration)
void MagInit(void)
{
	MagState = MAG_INITIALIZING;

	// check if magnetic sensor is connected
	if(HAL_I2C_IsDeviceReady(&hi2c3, MagAddress, 3, HAL_MAX_DELAY) != HAL_OK)
	{
		MagState = MAG_ERROR;			// indicate that magnetic sensor is not connected
		return;
	}

	// reset the magnetic sensor
	MagWriteB(0x06, 0x11);

	uint32_t StartTime = HAL_GetTick();		// get reset start time
	while ( (HAL_GetTick() - StartTime) < 3 )
	{
		// wait 3ms for reset to finish - takes 1.6ms
	}

	if( (MagReadB(0x00) & 0x08) != 0x08)		// determine if reset is successful
	{
		MagState = MAG_ERROR; 			// indicate that reset not successful
		return;
	}

	// configure OSRs and filters
	MagWriteB(0xAC, 0x14);	// set osrXY = 64, osrT = 32, XY and T filters
	MagWriteB(0xB5, 0x15);	// enable temperature compensation (reduce drift) and Z filter
	// note! configuration makes sample time approximately 8ms per sample

	// put magnetic sensor into power down mode
	MagWriteB(0x10, 0x00);

	MagState = MAG_IDLE;
}

// function to put magnetic sensor in single measurement mode
void MagSingleMode (void)
{
    if ((MagState == MAG_IDLE) && (MagDMABusy != 1))
    {
        MagWriteB(0x01, 0x10); // put sensor into single measurement mode

        MagState = MAG_SINGLEMEASUREMENT;	// change magnetic sensor state to single measurement mode
        MagSMFlag = 0;

        magMeasurementStartTime = HAL_GetTick();
    }
}

void MagPowerDown (void)
{
	// put magnetic sensor into power down mode
	MagWriteB(0x10, 0x00);

	MagState = MAG_IDLE;
}

void MagRead(void)
{
    if ((MagState == MAG_SINGLEMEASUREMENT) && (MagDMABusy != 1))
    {
        if ((HAL_GetTick() - magMeasurementStartTime) > 200)
        {
        	MagState = MAG_TIMEOUT;
        }

        uint8_t status = MagReadB(0x00); // check status register to see if data is ready

        if (status & 0x01)  // if data ready bit is set
        {
        	 // start DMA to read magnetic sensor measurement
        	 if (HAL_I2C_Mem_Read_DMA(&hi2c3, MagAddress, 0x01, I2C_MEMADD_SIZE_8BIT, DMABuffer, 6) == HAL_OK)
        	 {
        	     MagDMABusy = 1;
        	     MagState = MAG_READING;
        	 }
        	 else
        	 {
        		 MagState = MAG_ERROR;
        	 }
        }
    }
}

// function to get magnetic sensor measurements
void MagGetData (void)
{
	MagWriteB(0x10, 0x00);

	int16_t mx, my, mz;

	// get gyroscope data
	float XYsensitivityM = 1.5f;
	float ZsensitivityM = 1.5f;

	mx = (int16_t) ((DMABuffer[1] << 8) | DMABuffer[0]);
	my = (int16_t) ((DMABuffer[3] << 8) | DMABuffer[2]);
	mz = (int16_t) ((DMABuffer[5] << 8) | DMABuffer[4]);

	MagData.mx = ( (float)mx * XYsensitivityM );
	MagData.my = ( (float)my * XYsensitivityM );
	MagData.mz = ( (float)mz * ZsensitivityM );

	// get time stamp
	MagData.timestamp = HAL_GetTick();

	// put magnetic sensor into power down mode
	MagWriteB(0x10, 0x00);

	// store measurements into buffer
	MagDataAverage.mx[MagIndex] = MagData.mx;
	MagDataAverage.my[MagIndex] = MagData.my;
	MagDataAverage.mz[MagIndex] = MagData.mz;

	if (MagIndex == 4)
	{
		MagDataAverage.timestamp = MagData.timestamp;		// save the timestamp
		MagIndex = 0;										// reset index for saving the IMU data
	}
	else
	{
		MagIndex = MagIndex + 1;							// increment index for saving the IMU data
	}

	MagState = MAG_IDLE;

	MagMadgwick = 1;		 // set flag to indicate that mag data is ready to be filtered
	MagGetDataFlag = 0;      // reset the data ready flag
}

void MagAverage (void)
{
	// calculate the average of the previous 5 readings
	MagDataAverage.mxAvg = (MagDataAverage.mx[0] + MagDataAverage.mx[1] + MagDataAverage.mx[2] + MagDataAverage.mx[3] + MagDataAverage.mx[4]) / 5.0f;
	MagDataAverage.myAvg = (MagDataAverage.my[0] + MagDataAverage.my[1] + MagDataAverage.my[2] + MagDataAverage.my[3] + MagDataAverage.my[4]) / 5.0f;
	MagDataAverage.mzAvg = (MagDataAverage.mz[0] + MagDataAverage.mz[1] + MagDataAverage.mz[2] + MagDataAverage.mz[3] + MagDataAverage.mz[4]) / 5.0f;
}

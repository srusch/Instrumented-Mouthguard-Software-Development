/*
 * IMUFunctions.c
 *
 *  Created on: Sep 2, 2025
 *      Author: sarah
 */

#include "IMUFunctions.h"
#include "main.h"

// external I2C1 handle
extern I2C_HandleTypeDef hi2c1;

// variable declarations
static uint8_t DMABuffer[12];				// buffer to store data read over DMA

IMU_data IMUData;							// structure which stores the IMU's accelerometer and gyroscope measurements
IMU_data_average IMUDataAverage;
volatile IMU_state IMUState = IMU_IDLE;		// structure which stores the current state of the IMU
volatile uint8_t IMUDMABusy = 0;			// flag to indicate when IMU DMA is busy
volatile uint8_t IMUDataReadyFlag = 0;		// flag to indicate IMU INT1 has triggered
volatile uint8_t IMUGetDataFlag = 0;		// flag to indicate IMU DMA for data is complete
volatile uint8_t IMUMadgwick = 0;			// flag to indicate when IMU data can be sent to Madgwick filter
volatile uint8_t IMUIndex = 0;


float gxMax = -200;
float gyMax = -200;
float gzMax = -200;
float gxMin = 200;
float gyMin = 200;
float gzMin = 200;

float axMax = -200;
float ayMax = -200;
float azMax = - 200;
float axMin = 200;
float ayMin = 200;
float azMin = 200;
// blocking function to write to the IMU
void IMUWriteB (uint8_t data, uint8_t address)
{
	HAL_I2C_Mem_Write(&hi2c1, IMUAddress, address, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

// blocking function to read from IMU
uint8_t IMUReadB (uint8_t address)
{
	uint8_t data;				// variable to store what is read from IMU

	HAL_I2C_Mem_Read(&hi2c1, IMUAddress, address, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	return data;
}

// function to initialize IMU
void IMUInit (void)
{
	IMUState = IMU_INITIALIZING;

	// check if IMU is connected
	uint8_t WhoAmI = IMUReadB(0x0F);
	if(WhoAmI != 0x70)
	{
		IMUState = IMU_ERROR;	// indicate that IMU is not connected
	}

	// CTRL3
	IMUWriteB(0x01, 0x12); 		// Reset all registers to default
	while((IMUReadB(0x12) & 0x01) == 1)
	{
		// wait for software reset to complete
	}
	IMUWriteB(0x44, 0x12);		// set BDU, set auto increment

	// disable FIFO buffer
	IMUWriteB(0x00, 0x0A);

	// interrupt configuration - active high
	IMUWriteB(0x02, 0x0D);		// connect int1 to gyroscope data ready
	IMUWriteB(0x01, 0x0E);		// connect int2 to accelerometer data ready

	// gyroscope configuration
	IMUWriteB(0x05, 0x11);		// put gyroscope into high performance mode with a sample rate of 60Hz
	IMUWriteB(0x60, 0x15);		// set gyroscope FSR to 125dps with a bandwidth of 15Hz
	IMUWriteB(0x01, 0x16);		// enable gyroscope LPF

	// accelerometer configuration
	IMUWriteB(0x05, 0x10);		// put accelerometer into high performance mode with a sample rate of 60Hz
	IMUWriteB(0x00, 0x17);		// set accelerometer FSR to 2g with a bandwidth of 15Hz

	IMUState = IMU_IDLE;
}

// function to read IMU data
void IMURead (void)
{
	if ((IMUState == IMU_IDLE) && (IMUDMABusy != 1))		// if initialization is complete and DMA transfer is not occurring
	{
		// read IMU accelerometer and gyroscope measurements
		HAL_I2C_Mem_Read_DMA(&hi2c1, IMUAddress, 0x22, I2C_MEMADD_SIZE_8BIT, DMABuffer, 12);
		IMUDMABusy = 1;		// set IMU DMA flag
		IMUState = IMU_READING;		// change IMU state
	}
}

// function to get accelerometer and gyroscope measurements
void IMUGetData (void)
{
	int16_t gx, gy, gz, ax, ay, az;

	// get gyroscope data
	float sensitivityG = 0.004375f;

	gx = (int16_t) ((DMABuffer[1] << 8) | DMABuffer[0]);
	gy = (int16_t) ((DMABuffer[3] << 8) | DMABuffer[2]);
	gz = (int16_t) ((DMABuffer[5] << 8) | DMABuffer[4]);

	IMUData.gx = ( ((float)gx * sensitivityG ) +0.317187503 );
	IMUData.gy = ( ((float)gy * sensitivityG ) -0.2493750005 );
	IMUData.gz = ( ((float)gz * sensitivityG ) +0.238437496 );

	if (IMUData.gx > gxMax) gxMax = IMUData.gx;
	if (IMUData.gx < gxMin) gxMin = IMUData.gx;
	if (IMUData.gy > gyMax) gyMax = IMUData.gy;
	if (IMUData.gy < gyMin) gyMin = IMUData.gy;
	if (IMUData.gz > gzMax) gzMax = IMUData.gz;
	if (IMUData.gz < gzMin) gzMin = IMUData.gz;

	// get accelerometer data
	float sensitivityA = 0.000061;

	ax = (int16_t) ((DMABuffer[7] << 8) | DMABuffer[6]);
	ay = (int16_t) ((DMABuffer[9] << 8) | DMABuffer[8]);
	az = (int16_t) ((DMABuffer[11] << 8) | DMABuffer[10]);

	IMUData.ax = ( ((float)ax * sensitivityA * 9.81f) -0.0209443513f );
	IMUData.ay = ( ((float)ay * sensitivityA * 9.81f) -0.168751627f );
	IMUData.az = ( ((float)az * sensitivityA * 9.81f) -0.089770174f );

	if (IMUData.ax > axMax) axMax = IMUData.ax;
	if (IMUData.ax < axMin) axMin = IMUData.ax;
	if (IMUData.ay > ayMax) ayMax = IMUData.ay;
	if (IMUData.ay < ayMin) ayMin = IMUData.ay;
	if (IMUData.az > azMax) azMax = IMUData.az;
	if (IMUData.az < azMin) azMin = IMUData.az;
	// get time stamp
	IMUData.timestamp = HAL_GetTick();

	// store measurements into buffer
	IMUDataAverage.ax[IMUIndex] = IMUData.ax;
	IMUDataAverage.ay[IMUIndex] = IMUData.ay;
	IMUDataAverage.az[IMUIndex] = IMUData.az;

	IMUDataAverage.gx[IMUIndex] = IMUData.gx;
	IMUDataAverage.gy[IMUIndex] = IMUData.gy;
	IMUDataAverage.gz[IMUIndex] = IMUData.gz;

	if (IMUIndex == 5)
	{
		IMUDataAverage.timestamp = IMUData.timestamp;		// save the timestamp
		IMUIndex = 0;										// reset index for saving the IMU data
	}
	else
	{
		IMUIndex = IMUIndex + 1;							// increment index for saving the IMU data
	}

	IMUMadgwick = 1;		   // set flag to indicate that IMU data is ready to be filtered
    IMUGetDataFlag = 0;        // reset the IMU get data flag
    IMUDataReadyFlag = 0;      // reset the INT1 flag
    IMUState = IMU_IDLE;       // return IMU to idle state
}

void IMUAverage (void)
{
	// calculate the average of the previous 5 readings
	IMUDataAverage.axAvg = (IMUDataAverage.ax[0] + IMUDataAverage.ax[1] + IMUDataAverage.ax[2] + IMUDataAverage.ax[3] + IMUDataAverage.ax[4]) / 5.0f;
	IMUDataAverage.ayAvg = (IMUDataAverage.ay[0] + IMUDataAverage.ay[1] + IMUDataAverage.ay[2] + IMUDataAverage.ay[3] + IMUDataAverage.ay[4]) / 5.0f;
	IMUDataAverage.azAvg = (IMUDataAverage.az[0] + IMUDataAverage.az[1] + IMUDataAverage.az[2] + IMUDataAverage.az[3] + IMUDataAverage.az[4]) / 5.0f;

	IMUDataAverage.gxAvg = (IMUDataAverage.gx[0] + IMUDataAverage.gx[1] + IMUDataAverage.gx[2] + IMUDataAverage.gx[3] + IMUDataAverage.gx[4]) / 5.0f;
	IMUDataAverage.gyAvg = (IMUDataAverage.gy[0] + IMUDataAverage.gy[1] + IMUDataAverage.gy[2] + IMUDataAverage.gy[3] + IMUDataAverage.gy[4]) / 5.0f;
	IMUDataAverage.gzAvg = (IMUDataAverage.gz[0] + IMUDataAverage.gz[1] + IMUDataAverage.gz[2] + IMUDataAverage.gz[3] + IMUDataAverage.gz[4]) / 5.0f;
}

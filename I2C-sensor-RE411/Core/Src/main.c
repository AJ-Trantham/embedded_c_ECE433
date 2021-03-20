////////////////////////////////////
// LAB 8 - I2C Program to talk to MPU-9250.
// MPU-9250 is a System in Package (SiP)= MPU-6500 (which contains a 3-axis gyroscope, a 3-axis accelerometer) +  AK8963 (a 3-axis magnetometer).
// MPU-6500  is an I2C device with an address of 0x68 and AK8963 is another device with an address 0f 0x0C.
//
// The connections
//    I2C1_SCL - PB08
//    I2C1_SDA - PB09
//
// Assignment:
//    1) Implement the I2C1 initialization function  (I2C1_init)
//    2) Implement the I2C1 write function (I2C1_byteWrite)
//    3) Implement the I2C1 read function I2C1_byteRead)
//    Optional:
//    4) Use 4 LEDs and place them on 4 different directions (N, S, E, and W) on the breadboard
//    5) Using the accelerometer X and Y values blink the leds to reflect the tilting status.
//       This is left for you to decide how do implement. One way is to adjust the LED freq by
//       frequency divider function so that when the sensor is set on flat surface the LEDs are
//       blinking very fast when you think they are solid ON. when the sensor is tilted in the X
//       then X LED would slow the blinking, the slower the higher the tilt is.
//
////////////////////////////
#include "stm32f4xx.h"
#include "stdio.h"  // needed for the printing

#define _BV( idx ) ( 1<<(idx) )  // A nice macro for setting up the bit number <idx>


// All these defines are extracted from the device datasheet. Usually we put them in a separate .h file and we jut include it here.
#define MPU9250_ID    (0x68<<1) //MPU-6500 7-bit I2C address = 0x68  (or 0x69 with some circuit change)
#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75  // Should return 0x71  -- this can be used for testing that you can communicate with the device
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Those defines are for the other sensor
#define AK8963_ID        (0x0C<<1) // //AK8963   7-bit I2C address = 0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48 -- this can be used for testing that you can communicate with the device
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04  // data
#define AK8963_YOUT_L    0x05  // data
#define AK8963_YOUT_H    0x06  // data
#define AK8963_ZOUT_L    0x07  // data
#define AK8963_ZOUT_H    0x08  // data
#define AK8963_ST2       0x09  // status2
#define AK8963_CNTL      0x0A
#define AK8963_CNTL2     0x0B
#define AK8963_ASAX      0x10
#define AK8963_ASAY      0x11
#define AK8963_ASAZ      0x12
// All done with the datasheet registers


// Along with the 7-bit I2C device address, you need to send Read or Write bit as defined here
#define I2C_WRITE 0
#define I2C_READ  1

//***************************//
// Function Declaration
void I2C1_init(void);


// Device drivers
void  AK8963init();
void  MPU9250init();
void    I2C1_byteWrite(uint8_t, uint8_t, uint8_t) ;
uint8_t I2C1_byteRead(uint8_t, uint8_t);
int16_t readTemp();
int16_t readGyroX();
int16_t readGyroY();
int16_t readGyroZ();
int16_t readAccelX();
int16_t readAccelY();
int16_t readAccelZ();
int16_t readMagX();
int16_t readMagY();
int16_t readMagZ();

// General utility functions
void delayMs(int n);
void USART2_init(void);
void USART2_write(int c);
int  USART2_read(void);
void myprint(char msg[]);




int main(void)
{
    char txt[256];
    USART2_init();

    I2C1_init();
    MPU9250init();

    // I2C MPU9250 TEST
    uint8_t test_value= I2C1_byteRead(MPU9250_ID, WHO_AM_I_MPU9250);
    // test_value must be 0x71 if you can talk to the MPU-6500 device
    // Try to run in debug and check its value

    // I2C AK8963 TEST
    AK8963init();
    test_value= I2C1_byteRead(AK8963_ID, WHO_AM_I_AK8963);
    // test_value must be 0x48 if you can talk to the AK8963 device
    // Try to run in debug and check its value

    // Now the device is configured
    while(1)
    {
       int16_t T=readTemp();    // Read the sensor temperature
       int16_t Gx=readGyroX();  // Read the sensor Gyrosope for X axis
       int16_t Gy=readGyroY();  // Read the sensor Gyrosope for Y axis
       int16_t Gz=readGyroZ();  // Read the sensor Gyrosope for Z axis
       int16_t Ax=readAccelX(); // Read the sensor Accelerometer for X axis -- This reg will need to control LED X_left, X_right
       int16_t Ay=readAccelY(); // Read the sensor Accelerometer for Y axis -- This reg will need to control LED Y_up, y_down
       int16_t Az=readAccelZ(); // Read the sensor Accelerometer for Z axis
       int16_t Mx=readMagX();   // Read the sensor Magnetometer for X axis
       int16_t My=readMagY();   // Read the sensor Magnetometer for Y axis
       int16_t Mz=readMagZ();   // Read the sensor Magnetometer for Z axis

       ///////////////////////
       ///  *** Start Debugging Section -- Should be removed from final code *** ///
       sprintf(txt, "$%d %d %d %d %d %d %d %d %d %d;", T, Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz); // printing all sensors in one shot!
       // sample data output: 24 29 407 -148 5 -449 2066 0 0 0
       // One issue with dumping all data in one graph is that some of the have a high range while others don't. Consider scaling them down
       myprint(txt);
       ///  *** End Debugging Section -- Should be removed from final code *** ///
       //////////////////////



    //Re-strobe the mag data
    I2C1_byteWrite(AK8963_ID, AK8963_CNTL, 0x11); // Continuous mode, 16 bit output
    delayMs(100);
    }
}


// I2C1 Initialization function. This function will
// 1- Enable clocks (I2C and GPIO)
// 2- Enable GPIO in AF and Open-drain. Note that is no need for
//    pull up from the MCU since MPU-9250 has a built in pull-ups
// 3- Reset I2C1 module
// 4- Program its clock  (two fields in two different regs, checkout the book for example).
//    The target speed is 100KHz.
// 4- Enable peripheral
void I2C1_init(void) {
	RCC->AHB1ENR |= 2; 			// enable GPIOB clock
	RCC->APB1ENR |= (1<<21);	// Enable I2C1 clock

	// enable AF pins: PB8=I2C1_SCL, PB9=I2C_SDA
	GPIOB->AFR[1] &= ~0x000000FF;
	GPIOB->AFR[1] |= 0x44;

	GPIOB->MODER &= ~0x000F0000;
	GPIOB->MODER |= 0x000A0000;			// set PB8 and 9 to AF 10

	GPIOB->OTYPER |= 0x00000300;		// output with open drain

	// pull up resistors default on stm32F411

	I2C1->CR1 = 0x8000;			// software reset the I2C
	I2C1->CR1 &= ~0x8000;
	I2C1->CR2 = 0x0010;			// peripheral clock is 16 MHz
	I2C1->CCR = 80;				// standard mode, 100Khz
	I2C1->TRISE = 17;			// maximum rise time
	I2C1->CR1 |= 0x0001;		// enable I2C module
}



// I2C Write Operation
// Input parameters:
//   slave_addr -> slave device id (address is already shifted one place)
//   reg_addr   -> register address inside the slave device that needs to be written
//   reg_data   -> data to be written to the register
//
// This function will
//  1- make sure that the peripheral is not busy
//  2- issue the START condition
//  3- check that you see start flag (meaning that the master can control the SDA)
//  4- Send the slave ID (7bits) along with the intended operation (1bit for read or write)
//  5- check that you see address flag (meaning there is a device with a matching address)
//  6- clear SR1 and SR2  (by reading them)
//  7- check that the data reg is empty
//  8- send reg_addr
//  9- check that the data reg is empty
//  10- send reg_data
//  11- issue the STOP condition
void I2C1_byteWrite(uint8_t slave_addr, uint8_t reg_addr, uint8_t reg_data) {

	int used_to_clear;

	while (I2C1->SR2 & 2);			/*wait for I2C to be available*/

	I2C1->CR1 |= 0x100;				// generate start
	while (!(I2C1->SR1 & 1));		// wait until start flag gets set

	I2C1->DR = slave_addr | I2C_WRITE;			// transmit the slave address or since we address is already shifted to the left 1
	while (!(I2C1->SR1 & 2));					// wait until address flag is set
	used_to_clear = I2C1->SR2;					// Clear SR1
	used_to_clear = I2C1->SR1;					// Clear SR2

	while (!(I2C1->SR1 &0x80));					// check that data reg is empty TxE is off
	I2C1->DR = reg_addr;						// send address

	while (!(I2C1->SR1 & 0x80));				// check that data reg is empty TxE is off
	I2C1->DR = reg_data;						// send data

	while (!(I2C1->SR1 & 0x80));				// wait until we are done writing to kill convo - this is important and omitting it caused issues
	I2C1->CR1 |= 0x200;							// generate stop

}

// I2C Read Operation
// Input parameters:
//   slave_addr -> slave device id (address is already shifted one place)
//   reg_addr   -> register address inside the slave device that needs to be read
//  Returned
//   reg_data   -> data from register
//
// This function will
//  1- make sure that the peripheral is not busy
//  2- issue the START condition
//  3- check that you see start flag (meaning that the master can control the SDA)
//  4- Send the slave ID (7bits) along with the intended operation (1bit for read or write)
//  5- check that you see address flag (meaning there is a device with a matching address)
//  6- clear SR1 and SR2  (by reading them)
//  7- check that the data reg is empty
//  8- send reg_addr
//  9- check that the data reg is empty
//  10- issue the RESTART condition
//  11- check that you see start flag (meaning that the master can control the SDA)
//  12- Send the slave ID (7bits) along with the intended operation (1bit for read or write)
//  13- check that you see address flag (meaning there is a device with a matching address)
//  14- Disable Acknowledge
//  15- clear SR1 and SR2  (by reading them)
//  16- check that the receive data reg is not empty
//  17- read the received data and store it into reg_data
//  18- issue the STOP condition
//  19- return reg_data
uint8_t I2C1_byteRead(uint8_t slave_addr, uint8_t reg_addr) {
    uint8_t reg_data;
    int used_to_clear;

    while (I2C1->SR2 & 2);			/*wait for I2C to be available*/

    I2C1->CR1 |= 0x100;				// generate start
    while (!(I2C1->SR1 & 1));		// wait until start flag gets set

    I2C1->DR = slave_addr | I2C_WRITE;			// transmit the slave address and action bit
    while (!(I2C1->SR1 & 2));					// wait until address flag is set
    used_to_clear = I2C1->SR2;					// Clear SR1
    used_to_clear = I2C1->SR1;					// Clear SR2

    while (!(I2C1->SR1 &0x80));					// check that data reg is empty TxE is off
    I2C1->DR = reg_addr;						// send address

    while (!(I2C1->SR1 &0x80));					// check that data reg is empty TxE is off

    I2C1->CR1 |= 0x100;							// generate restart 10
    while (!(I2C1->SR1 & 1));					// wait until start flag gets set

    I2C1->DR = slave_addr | I2C_READ;			// transmit the slave address and read bit
    while (!(I2C1->SR1 & 2));					// wait until address flag is set
    I2C1->CR1 &= ~(1<<10);						// clear the ack bit
    used_to_clear = I2C1->SR2;					// Clear SR1
    used_to_clear = I2C1->SR1;					// Clear SR2

    while (!(I2C1->SR1 & 0x40));				// wait until rxne is set, we are ready to read
    reg_data = I2C1->DR;						// read data from data register

    I2C1->CR1 |= 0x200;							// generate stop


    return reg_data;
}



void MPU9250init(){
   #define H_RESET 7
   #define CLKSEL  1
   // MPU-9250 Chip

   // Register[PWR_MGMT_1] : H_RESET   SLEEP   CYCLE   GYRO_STANDBY   PD_PTAT   CLKSEL[2:0]
   //                           1        0       0         0             0          1
   I2C1_byteWrite(MPU9250_ID, PWR_MGMT_1, _BV(H_RESET)|CLKSEL); // Clear SLEEP mode bit, enable all sensors

   delayMs(10);                             // Sometime to allow reseting the chip

   I2C1_byteWrite(MPU9250_ID, PWR_MGMT_1, CLKSEL);             //  Auto select clock or use internal
   delayMs(100);                           // Sometime to allow stabilizing the chip

   // Register[CONFIG] : FIFO_MODE   EXT_SYNC_SET[2:0]   DLPF_CFG[2:0]
   //                        0              0                3
   I2C1_byteWrite(MPU9250_ID, CONFIG,     0x03); // Low-pass filter enable

   // Register[SMPLRT_DIV] : SMPLRT_DIV[7:0]
   //                             4
   // SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) = 1K/(1+4)= 200Hz
   I2C1_byteWrite(MPU9250_ID, SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate


   // Register[GYRO_CONFIG]: XGYRO_Cten   YGYRO_Cten    ZGYRO_Cten   GYRO_FS_SEL[1:0]         -     FCHOICE_B[1:0]
   //                           0             0             0          00  (+250dps)         N/A          00
   //                                                                  01  (+500dps)
   //                                                                  10  (+1000dps)
   //                                                                  11  (+2000dps)
   //
   //I2C1_byteWrite(MPU9250_ID, GYRO_CONFIG, (3<<3) );   // Set full scale range for the gyro +2000dps
   I2C1_byteWrite(MPU9250_ID, GYRO_CONFIG, 0 );   // Set full scale range for the gyro +250dps




   // Register[ACCEL_CONFIG]: ax_st_en   ay_st_en   az_st_en   ACCEL_FS_SEL[1:0]   -[2:0]
   //                             0          0         0             00  (+-2g)
   //                                                                01  (+-4g)
   //                                                                10  (+-8g)
   //                                                                11  (+-16g)
   I2C1_byteWrite(MPU9250_ID, ACCEL_CONFIG, (3<<3));  // Set full scale range for the accelerometer +-16g


   // Register[ACCEL_CONFIG2]: -     ACCEL_FCHOICE_B[3:2]       A_DLPF_CFG[1:0]
   //                                           0                       3
   I2C1_byteWrite(MPU9250_ID, ACCEL_CONFIG2, (3<<3));  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz


   // Register[INT_PIN_CFG]: ACTL      OPEN      LATCH_INT_EN      INT_ANYRD_2CLEAR    ACTL_FSYNC    FSYNC_INT_MODE_EN     BYPASS_EN   -
   //                         0         0             0                  1                 0                0                1        N/A
   I2C1_byteWrite(MPU9250_ID, INT_PIN_CFG, 0x12);             // Allows the MCU to access the magnetometer on the I2C b us
}

void AK8963init(){
I2C1_byteWrite(AK8963_ID, AK8963_CNTL, 0x00); // Power down magnetometer
I2C1_byteWrite(AK8963_ID, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
I2C1_byteWrite(AK8963_ID, AK8963_CNTL, 0x12); // Continuous mode, 16 bit output
}

int16_t readTemp(){
int16_t tempInt;
tempInt =I2C1_byteRead(MPU9250_ID, TEMP_OUT_H)<<8;
tempInt|=I2C1_byteRead(MPU9250_ID, TEMP_OUT_L);
float  tempFloat=((float) tempInt) / 333.87 + 21.0;
return (int16_t) tempFloat;
}

int16_t readGyroX(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, GYRO_XOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, GYRO_XOUT_L);
return res;
}

int16_t readGyroY(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, GYRO_YOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, GYRO_YOUT_L);
return res;
}

int16_t readGyroZ(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, GYRO_ZOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, GYRO_ZOUT_L);
return res;
}

int16_t readAccelX(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, ACCEL_XOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, ACCEL_XOUT_L);
return res;
}

int16_t readAccelY(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, ACCEL_YOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, ACCEL_YOUT_L);
return res;
}

int16_t readAccelZ(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, ACCEL_ZOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, ACCEL_ZOUT_L);
return res;
}
int16_t readMagX(){
int16_t res;
res  = I2C1_byteRead(AK8963_ID, AK8963_XOUT_H)<<8;
res |= I2C1_byteRead(AK8963_ID, AK8963_XOUT_L);
return res;
}
int16_t readMagY(){
int16_t res;
res  = I2C1_byteRead(AK8963_ID, AK8963_YOUT_H)<<8;
res |= I2C1_byteRead(AK8963_ID, AK8963_YOUT_L);
return res;
}
int16_t readMagZ(){
int16_t res;
res  = I2C1_byteRead(AK8963_ID, AK8963_ZOUT_H)<<8;
res |= I2C1_byteRead(AK8963_ID, AK8963_ZOUT_L);
return (res);
}


void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0' ){ USART2_write(msg[idx++]);}
}

/* initialize USART2 to transmit at 9600 Baud */
void USART2_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* Enable USART2 clock */

    /* Configure PA2, PA3 for USART2 TX, RX */
    GPIOA->AFR[0] &= ~0xFF00;
    GPIOA->AFR[0] |=  0x7700;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x00F0;
    GPIOA->MODER  |=  0x00A0;   /* enable alt. function for PA2, PA3 */

    USART2->BRR = 0x008B;       /* 115200 baud @ 16 MHz */
    USART2->CR1 = 0x000C;       /* enable Tx, Rx, 8-bit data */
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */
}

/* Write a character to USART2 */
void USART2_write (int ch) {
    while (!(USART2->SR & 0x0080)) {}   // wait until Tx buffer empty
    USART2->DR = (ch & 0xFF);
}

/* Read a character from USART2 */
int USART2_read(void) {
    while (!(USART2->SR & 0x0020)) {}   // wait until char arrives
    return USART2->DR;
}


void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/io.h>
#include <stdio.h>
#include <errno.h>

#include "i2c-utils.h"
#include "legato.h"
#include "interfaces.h"

#define PIXEL_NUM				64

#define POWER_CONTROL_REG_ADDR			0X00
#define RESET_REG_ADDR				0X01
#define FRAME_RATE_ADDR				0X02
#define INTERRUPT_CONTROL_REG_ADDR		0X03
#define STATUS_REG_ADDR				0x04
#define STATUS_CLEAR_REG_ADDR			0X05
#define AVERAGE_REG_ADDR			0X07
#define INT_LEVEL_REG_ADDR_HL			0X08
#define INT_LEVEL_REG_ADDR_HH			0X09
#define INT_LEVEL_REG_ADDR_LL			0X0A
#define INT_LEVEL_REG_ADDR_LH			0X0B
#define INT_LEVEL_REG_ADDR_YSL			0X0C
#define INT_LEVEL_REG_ADDR_YSH			0X0D
#define THERMISTOR_REG_ADDR_L			0X0E
#define THERMISTOR_REG_ADDR_H			0X0F
#define INTERRUPT_TABLE_1_8_REG_ADDR		0X10
#define INTERRUPT_TABLE_9_16_REG_ADDR		0X11
#define INTERRUPT_TABLE_17_24_REG_ADDR		0X12
#define INTERRUPT_TABLE_25_32_REG_ADDR		0X13
#define INTERRUPT_TABLE_33_40_REG_ADDR		0X14
#define INTERRUPT_TABLE_41_48_REG_ADDR		0X15
#define INTERRUPT_TABLE_49_56_REG_ADDR		0X16
#define INTERRUPT_TABLE_57_64_REG_ADDR		0X17

#define TEMPERATURE_REG_ADDR_L			0X80
#define TEMPERATURE_REG_ADDR_H			0X81

#define INT_ACTIVE				0X01
#define INT_ABS_VALUE_INT_MODE			0X02

#define NORMAL_MODE				0X00
#define SLEEP_MODE				0X10
#define STAND_BY_MODE_60S_INTERMITTENCE		0X20
#define STAND_BY_MODE_10S_INTERMITTENCE		0X21

#define CLEAR_ALL_STATUS			0X0E
#define CLEAR_INTERRUPT_STATUS			0X02

#define INIT_RESET_VALUE			0X3F
#define FLAG_RESET_VALUE			0X30

#define DEFAULT_IIC_ADDR			0x69

#define I2C_HUB_PORT_RASPI			0x08
#define I2C_HUB_PORT_IOT			0x01
#define I2C_HUB_PORT_GPIO			0x04
#define I2C_HUB_PORT_USB_HUB			0x02
#define I2C_HUB_PORT_ALL 			0x0F

char infrared_temperature_i2c_bus[256] = "/dev/i2c-1";

int I2C_write_byte(uint8_t reg, uint8_t data)
{
	int res = 0;
	int i2c_fd = open("/dev/i2c-1", O_RDWR);
	if (i2c_fd < 0) {
		LE_ERROR("i2cSendByte: failed to open /dev/i2c-1");
		close(i2c_fd);
		return -1;
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, DEFAULT_IIC_ADDR) < 0) {
		LE_ERROR("Could not set address to 0x%02x: %s\n",
		       DEFAULT_IIC_ADDR, strerror(errno));
		close(i2c_fd);
		return -1;
	}
	res = i2c_smbus_write_byte_data(i2c_fd, reg, data);
	close(i2c_fd);
	return res;
}

int I2C_write_16bit(uint8_t reg, uint16_t data)
{
	int res = 0;
	uint8_t val[2]={0,};
	int i2c_fd = open("/dev/i2c-1", O_RDWR);
	if (i2c_fd < 0) {
		printf("i2cSendByte: failed to open /dev/i2c-1");
		close(i2c_fd);
		return -1;
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, DEFAULT_IIC_ADDR) < 0) {
		printf("Could not set address to 0x%02x: %s\n",
		       DEFAULT_IIC_ADDR, strerror(errno));
		close(i2c_fd);
		return -1;
	}
	val[0] = 0xff&(data>>8) ;
	val[1] = data&0xff;
	res = i2c_smbus_write_i2c_block_data(i2c_fd, reg, 2, val);
	close(i2c_fd);
        return res;

}

void I2C_read_byte(uint8_t reg, uint8_t *data)
{
	int i2c_fd = open("/dev/i2c-1", O_RDWR);
	if (i2c_fd < 0) {
		printf("i2cSendByte: failed to open /dev/i2c-1");
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, DEFAULT_IIC_ADDR) < 0) {
		printf("Could not set address to 0x%02x: %s\n",
		       DEFAULT_IIC_ADDR, strerror(errno));
		// return;
	}
	*data = i2c_smbus_read_byte_data(i2c_fd, reg);
	close(i2c_fd);
}

void I2C_read_16bit(uint8_t start_reg, uint16_t *value)
{
	*value = 0;
	uint8_t tmp[2] = {0,};
	int i2c_fd = open("/dev/i2c-1", O_RDWR);
	if (i2c_fd < 0) {
		printf("i2cSendByte: failed to open /dev/i2c-1");
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, DEFAULT_IIC_ADDR) < 0) {
		printf("Could not set address to 0x%02x: %s\n",
		       DEFAULT_IIC_ADDR,
		       strerror(errno));
		// return;
	}
	i2c_smbus_read_i2c_block_data(i2c_fd, start_reg, 2, tmp);
	*value|=(uint16_t)tmp[0]<<8;
	*value|=tmp[1];
	close(i2c_fd);
}

/**@brief set frame rate,default is 10FPS
 * @param rate
 * @return 0 if success
 * */
int ma_infraredTemperature_set_frame_rate(uint8_t rate)
{
	return I2C_write_byte(FRAME_RATE_ADDR, rate);
}

/**@brief set sensor mode.
 * @param sensor mode
 * @return 0 if success
 * */

int set_sensor_mode(uint8_t mode)
{
	return I2C_write_byte(POWER_CONTROL_REG_ADDR, mode);
}

/**@brief set interrupt mode.
 * @param interrupt mode
 * @return 0 if success
 * */

int set_interrupt_mode(uint8_t mode)
{
	return I2C_write_byte(INTERRUPT_CONTROL_REG_ADDR, mode);
}

/**@brief get interrupt status.
 * @param interrupt status.
 * @return 0 if no interrupt.1 indicate interrupt was generated.
 * */
int ma_infraredTemperature_get_interrupt_status()
{
	uint8_t value;
	I2C_read_byte(STATUS_REG_ADDR, &value);
	if(value&0x02)
	{
		return 1;
	}
	return 0;
}

/**@brief clear status.overflow status or interrupt status.
 * @param value
 * @return 0 if success
 * */
int clear_status(uint8_t value)
{
	return I2C_write_byte(STATUS_CLEAR_REG_ADDR, value);
}

/**@brief read interrupt status for 64 channel pixels.
 * @param u8 [8],dst value.
 * @return 0 if success
 * */
int ma_infraredTemperature_read_pixels_interrupt_status(uint8_t *status)
{
	uint32_t i = 0;
	for(i = 0; i < 8; i++)
	{
		I2C_read_byte(INTERRUPT_TABLE_1_8_REG_ADDR+i, &status[i]);
	}
 	return 0;
}

/**@brief set upper limit.
 * @param u8[2] value.
 * @return 0 if success
 * */
int set_upper_limit(uint8_t value[])
{
	int ret;
	ret = I2C_write_byte(INT_LEVEL_REG_ADDR_HL, value[0]);
	ret = I2C_write_byte(INT_LEVEL_REG_ADDR_HH, value[1]);
	return ret;
}

/**@brief set lower limit.
 * @param u8[2] value.
 * @return 0 if success
 * */
int set_lower_limit(uint8_t value[])
{
	int ret;
	ret = I2C_write_byte(INT_LEVEL_REG_ADDR_LL, value[0]);
	ret = I2C_write_byte(INT_LEVEL_REG_ADDR_LH, value[1]);
	return ret;
}


/**@brief set hysteresis for interrupt.
 * @param u8[2] value.
 * @return 0 if success
 * */
int set_hysteresis(uint8_t value[])
{
	int ret;
	ret = I2C_write_byte(INT_LEVEL_REG_ADDR_YSL, value[0]);
	ret = I2C_write_byte(INT_LEVEL_REG_ADDR_YSH, value[1]);
	return ret;
}


/**@brief reset sensor flag,0x30-reset interrupt flag,0x3f-reinit the sensor.
 * @param reset option value.
 * @return 0 if success
 * */
int reset_flags(uint8_t value)
{
	return I2C_write_byte(RESET_REG_ADDR,value);
}



/**@brief read pixels for 64 channel register data,convert to float data
 * @param float[64],dst value
 * @return 0 if success
 * */
void ma_infraredTemperature_read_pixel_temperature(double *pixel_data, size_t *data_len)
{
	uint8_t val_l=0,val_h=0;
	uint16_t value=0;
	for(int i = 0; i < *data_len; i++)
 	{
		I2C_read_byte(TEMPERATURE_REG_ADDR_L+2*i, &val_l);
		I2C_read_byte(TEMPERATURE_REG_ADDR_H+2*i, &val_h);
		value=(uint16_t)val_h<<8|val_l;
		pixel_data[i]=(double)(value>>2)+(double)(value&0x03)*0.25;
	}
}


/**
 * Function: Configure I2C hub to enable I2C bus that connected to temperature sensor
 * Params: 	- hub_address: I2C address of hub
 * 		- port: Bus ports
 **/
int i2c_hub_select_port(uint8_t hub_address, uint8_t port)
{
	int result = 0;
	int i2c_fd = open(infrared_temperature_i2c_bus, O_RDWR);

	if (i2c_fd < 0) {
	
		LE_ERROR("i2cSendByte: failed to open %s", infrared_temperature_i2c_bus);
	}
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, hub_address) < 0) {
		LE_ERROR("Could not set address to 0x%02x: %s\n",
			 hub_address,
			 strerror(errno));
		return -1;
	}
	const int writeResult = i2c_smbus_write_byte(i2c_fd, port);
	if (writeResult < 0) {
		LE_ERROR("smbus write failed with error %d\n", writeResult);
		result = -1;
	} else {
		result = 0;
	}
	close(i2c_fd);

	return result;
}

/**@brief config sensor.
 * @return 0 if success
 * 
 * 
 * */
void ma_infraredTemperature_Init()
{

	/*32°C，calculate method：u16 value,last bit0~bit1*0.25,bit2~bit10 /4,bit11 is sign bit,1 is negative.*/
	uint8_t cfg_data_high_limit[]={0x80, 0x00};
	/*0.25°C*/
	uint8_t cfg_data_low_limit[]={0x01, 0x00};
	/*2°C,When temperature lower than (high_limit-hysteresis)(30-2=28℃),INT pin back to origin status. */
	uint8_t cfg_data_hysteresis[]={0x08, 0x00};

	/*Normal mode!!,if return none-zero,IIC communication error,return error code.*/
	set_sensor_mode(NORMAL_MODE);
	
	sleep(1);
	set_upper_limit(cfg_data_high_limit);
	set_lower_limit(cfg_data_low_limit);
	set_hysteresis(cfg_data_hysteresis);
	/*Enable interrupt function.*/
	set_interrupt_mode(INT_ACTIVE|INT_ABS_VALUE_INT_MODE);
	//set_interrupt_mode(INT_ACTIVE);
	sleep(1);
	/*Clear all status register*/
	clear_status(CLEAR_ALL_STATUS);
	/*Reset all flag*/
	reset_flags(INIT_RESET_VALUE);

	i2c_hub_select_port(0x71, I2C_HUB_PORT_ALL);
}


void ma_infraredTemperature_Deinit(void)
{
}

COMPONENT_INIT
{
	LE_INFO("Infrared Temperature service started");
	// i2c_hub_select_port(0x71, I2C_HUB_PORT_RASPI);
}
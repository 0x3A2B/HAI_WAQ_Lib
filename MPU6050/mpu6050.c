#include "mpu6050.h"
#include "stm32f1xx_hal.h"

/**
  * @brief   写数据到MPU6050寄存器
  * @param   
  * @retval  
  */
void MPU6050_WriteReg(I2C_HandleTypeDef *  hi2c, uint8_t reg_add,uint8_t reg_dat)
{
	HAL_I2C_Mem_Write(hi2c, MPU6050_SLAVE_ADDRESS, reg_add, I2C_MEMADD_SIZE_8BIT, &reg_dat, 1, 0xffff );
}


/**
  * @brief   从MPU6050寄存器读取数据
  * @param   
  * @retval  
  */
void MPU6050_ReadData(I2C_HandleTypeDef *  hi2c, uint8_t reg_add, unsigned char * Read, uint8_t num)
{
	HAL_I2C_Mem_Read(hi2c, MPU6050_SLAVE_ADDRESS, reg_add, num * I2C_MEMADD_SIZE_8BIT, Read, num, 0xffff);
}


/**
  * @brief   初始化MPU6050芯片
  * @param   
  * @retval  
  */
void MPU6050_Init(I2C_HandleTypeDef *  hi2c)
{
	HAL_Delay(20);                                                                              //延时, 等待数据正常
	MPU6050_WriteReg(hi2c, MPU6050_RA_PWR_MGMT_1, 0x00);	    //解除休眠状态
	MPU6050_WriteReg(hi2c, MPU6050_RA_SMPLRT_DIV , 0x07);	    //陀螺仪采样率，1KHz
	MPU6050_WriteReg(hi2c, MPU6050_RA_CONFIG , 0x06);	    //低通滤波器的设置，截止频率是1K，带宽是5K
	MPU6050_WriteReg(hi2c, MPU6050_RA_ACCEL_CONFIG , 0x00);   //配置加速度传感器工作在2G模式，不自检
	MPU6050_WriteReg(hi2c, MPU6050_RA_GYRO_CONFIG, 0x18);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
}

/**
  * @brief   读取MPU6050的ID
  * @param   
  * @retval  
  */
uint8_t MPU6050ReadID(I2C_HandleTypeDef *  hi2c)
{
	unsigned char Re = 0;
  MPU6050_ReadData(hi2c, MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
	if(Re != 0x68){
		return Re;
	}
	else{
		return 1;
	}
		
}
/**
  * @brief   读取MPU6050的加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadAcc(I2C_HandleTypeDef *  hi2c, short *accData)
{
    uint8_t buf[6];
    MPU6050_ReadData(hi2c, MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的角加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadGyro(I2C_HandleTypeDef *  hi2c, short *gyroData)
{
    uint8_t buf[6];
    MPU6050_ReadData(hi2c, MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}


/**
  * @brief   读取MPU6050的原始温度数据
  * @param   
  * @retval  
  */
void MPU6050ReadTemp(I2C_HandleTypeDef *  hi2c, short *tempData)
{
	uint8_t buf[2];
    MPU6050_ReadData(hi2c, MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}


/**
  * @brief   读取MPU6050的温度数据，转化成摄氏度
  * @param   
  * @retval  
  */
void MPU6050_ReturnTemp(I2C_HandleTypeDef *  hi2c, short*Temperature)
{
	short temp3;
	uint8_t buf[2];
	
	MPU6050_ReadData(hi2c, MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
  temp3= (buf[0] << 8) | buf[1];
	*Temperature=(((double) (temp3 + 13200)) / 280)-13;
}

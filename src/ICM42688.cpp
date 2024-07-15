#include <iostream>
#include "ICM42688.hpp"
uint64_t micro_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}
ICM42688::ICM42688(){

    time_interval=0.005f;
    ex_i = 0;
    ey_i = 0;
    ez_i = 0;
    Kp=3.0;
    Ki=0.5;

}


void ICM42688::SPIinit(int spinum,int csnum, uint8_t aMode, uint8_t gMode, uint8_t Ascale,uint8_t Gscale,uint8_t AODR,uint8_t GODR)
{
    fd = SPIInit(spinum,csnum);
    fd_address=&fd;
    gstGyroOffset ={0,0,0}; 
    state_check bRet = ICM42688::Check();
  
    if( true == bRet)
    {
        std::cout<< "Motion sersor is ICM-42688\n"<<std::endl;
        ICM42688::Init(aMode,gMode, Ascale,Gscale,AODR,GODR);
    }
    else
    {
     
        std::cout<<    "Motion sersor NULL\n"<<std::endl;
    }
    q0 = 1.0f;  
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

}
state_check ICM42688::Check(void)
{
    state_check bRet = false;
    if(VAL_WHO_AM_I == SPI_ReadOneByte(fd,ICM42688_WHO_AM_I))
    {
        SPI_WriteOneByte(fd,ICM42688_REG_BANK_SEL, 0x00);
        bRet=true;
        
    }
    return bRet;
}

void ICM42688::Init(uint8_t aMode, uint8_t gMode, uint8_t Ascale,uint8_t Gscale,uint8_t AODR,uint8_t GODR)
{
    SPI_WriteOneByte(fd,ICM42688_REG_BANK_SEL, 0x00); // select register bank 0
    SPI_WriteOneByte(fd, ICM42688_PWR_MGMT0,  gMode << 2 | aMode); // set accel and gyro modes
    SPI_WriteOneByte(fd,ICM42688_ACCEL_CONFIG0, Ascale << 5 | AODR); // set accel ODR and FS
    SPI_WriteOneByte(fd,ICM42688_GYRO_CONFIG0,  Gscale << 5 | GODR); // set gyro ODR and FS
    SPI_WriteOneByte(fd,ICM42688_GYRO_ACCEL_CONFIG0,  0x44); // set gyro and accel bandwidth to ODR/10
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
     case GFS_15_625DPS:
        Gscale_factor=GFS_15_625DPS_SCALE_FACTOR;
          break;
    case GFS_31_25DPS:
        Gscale_factor=GFS_31_25DPS_SCALE_FACTOR;
          break;
    case GFS_62_50DPS:
        Gscale_factor=GFS_62_50DPS_SCALE_FACTOR;
          break;
    case GFS_125DPS:
        Gscale_factor=GFS_125DPS_SCALE_FACTOR;
          break;
    case GFS_250DPS:
        Gscale_factor=GFS_250DPS_SCALE_FACTOR;
          break;
    case GFS_500DPS:
        Gscale_factor=GFS_500DPS_SCALE_FACTOR;
          break;
    case GFS_1000DPS:
        Gscale_factor=GFS_1000DPS_SCALE_FACTOR;
         break;
    case GFS_2000DPS:
        Gscale_factor=GFS_2000DPS_SCALE_FACTOR;
         break;
  }
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
    case AFS_2G:
        Ascale_factor= AFS_2G_SCALE_FACTOR;
         break;
    case AFS_4G:
        Ascale_factor= AFS_4G_SCALE_FACTOR;
         break;
    case AFS_8G:
        Ascale_factor= AFS_8G_SCALE_FACTOR;
         break;
    case AFS_16G:
        Ascale_factor= AFS_16G_SCALE_FACTOR;
         break;
  }
    usleep(5000);
    /* offset */
    ICM42688::GyroOffset();

    return;
}

void ICM42688::Close(void)
{
    SPIClose(fd_address);
    std::cout<<    "imu sensor close"<<std::endl;
}

void ICM42688::GyroRead(int16_t& s16X, int16_t& s16Y, int16_t& s16Z)
{
    uint8_t u8Buf[2];
    int16_t s16Buf[3] = {0}; 

    u8Buf[0]=SPI_ReadOneByte(fd,ICM42688_GYRO_DATA_X0); 
    u8Buf[1]=SPI_ReadOneByte(fd,ICM42688_GYRO_DATA_X1); 

    s16Buf[0]=  (u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=SPI_ReadOneByte(fd,ICM42688_GYRO_DATA_Y0); 
    u8Buf[1]=SPI_ReadOneByte(fd,ICM42688_GYRO_DATA_Y1); 
    s16Buf[1]=  (u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=SPI_ReadOneByte(fd,ICM42688_GYRO_DATA_Z0); 
    u8Buf[1]=SPI_ReadOneByte(fd,ICM42688_GYRO_DATA_Z1); 
    s16Buf[2]=  (u8Buf[1]<<8)|u8Buf[0];
    

    s16X = s16Buf[0] - gstGyroOffset.s16X;
    s16Y = s16Buf[1] - gstGyroOffset.s16Y;
    s16Z = s16Buf[2] - gstGyroOffset.s16Z;

    return;
}
void ICM42688::AccelRead(int16_t& s16X, int16_t& s16Y, int16_t& s16Z)
{
    uint8_t u8Buf[2];
    int16_t s16Buf[3] = {0}; 

    u8Buf[0]=SPI_ReadOneByte(fd,ICM42688_ACCEL_DATA_X0); 
    u8Buf[1]=SPI_ReadOneByte(fd,ICM42688_ACCEL_DATA_X1); 

    s16Buf[0]=  (u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=SPI_ReadOneByte(fd,ICM42688_ACCEL_DATA_Y0); 
    u8Buf[1]=SPI_ReadOneByte(fd,ICM42688_ACCEL_DATA_Y1); 
    s16Buf[1]=  (u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=SPI_ReadOneByte(fd,ICM42688_ACCEL_DATA_Z0); 
    u8Buf[1]=SPI_ReadOneByte(fd,ICM42688_ACCEL_DATA_Z1); 
    s16Buf[2]=  (u8Buf[1]<<8)|u8Buf[0];


    s16X = s16Buf[0];
    s16Y = s16Buf[1];
    s16Z = s16Buf[2];

    return;
}

void ICM42688::GyroOffset(void)
{
  uint8_t i;
  int16_t s16Gx = 0, s16Gy = 0, s16Gz = 0;
  int32_t s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;

  for(i = 0; i < 64; i ++)
  {
    ICM42688::GyroRead(s16Gx, s16Gy, s16Gz);
    s32TempGx += s16Gx;
    s32TempGy += s16Gy;
    s32TempGz += s16Gz;
    usleep(5000);
  }
  gstGyroOffset.s16X = s32TempGx >> 6;
  gstGyroOffset.s16Y = s32TempGy >> 6;
  gstGyroOffset.s16Z = s32TempGz >> 6;
  
  return;
}

void ICM42688::imuDataGet()
                {
        
        float MotionVal[6];
        int16_t s16Gyro[3], s16Accel[3];
        ICM42688::AccelRead(s16Accel[0], s16Accel[1], s16Accel[2]);
        
        ICM42688::GyroRead(s16Gyro[0], s16Gyro[1], s16Gyro[2]);


        MotionVal[0]=(float)s16Gyro[0]/Gscale_factor;
        MotionVal[1]=(float)s16Gyro[1]/Gscale_factor;
        MotionVal[2]=(float)s16Gyro[2]/Gscale_factor;
        MotionVal[3]=s16Accel[0];
        MotionVal[4]=s16Accel[1];
        MotionVal[5]=s16Accel[2];

        mahonyAHRSupdate((float)MotionVal[0] * 0.01745, (float)MotionVal[1] * 0.01745, (float)MotionVal[2] * 0.01745,
                        (float)MotionVal[3], (float)MotionVal[4], (float)MotionVal[5]);
        stQuaternion.q0=q0;
        stQuaternion.q1=q1;
        stQuaternion.q2=q2;
        stQuaternion.q3=q3;

        /*
        pstAngles.fPitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.295; // pitch
        pstAngles.fRoll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.295; // roll
        pstAngles.fYaw = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.295; 
        */
        stGyroRawData.s16X = s16Gyro[0]/Gscale_factor;
        stGyroRawData.s16Y = s16Gyro[1]/Gscale_factor;
        stGyroRawData.s16Z = s16Gyro[2]/Gscale_factor;

        stAccelRawData.s16X = s16Accel[0]/Ascale_factor;
        stAccelRawData.s16Y = s16Accel[1]/Ascale_factor;
        stAccelRawData.s16Z  = s16Accel[2]/Ascale_factor;
}






void ICM42688::mahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az){
    float Norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float qa, qb, qc;
        
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		Norm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= Norm;
		ay *= Norm;
		az *= Norm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		vx = 2*(q1 * q3 - q0 * q2);
		vy = 2*(q0 * q1 + q2 * q3);
		vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		ex = (ay * vz - az * vy);
		ey = (az * vx - ax * vz);
		ez = (ax * vy - ay * vx);
    ex_i += ex*time_interval;
    ey_i += ey*time_interval;
    ez_i += ez*time_interval;

    gx += Kp * ex + Ki * ex_i;
    gy += Kp * ey + Ki * ey_i;
    gz += Kp * ez + Ki * ez_i;

	}
	

	qa = q0;
	qb = q1;
	qc = q2;
 
	q0 += 0.5f*(-qb * gx - qc * gy - q3 * gz)*time_interval;
	q1 += 0.5f*(qa * gx + qc * gz - q3 * gy)*time_interval;
	q2 += 0.5f*(qa * gy - qb * gz + q3 * gx)*time_interval;
	q3 += 0.5f*(qa * gz + qb * gy - qc * gx)*time_interval; 
	
	// Normalise quaternion
	Norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= Norm;
	q1 *= Norm;
	q2 *= Norm;
	q3 *= Norm;

}
/*
std::thread ICM42688::imuDataGet_thread(void){
    return std::thread([this]{imuDataGet();});
}
*/

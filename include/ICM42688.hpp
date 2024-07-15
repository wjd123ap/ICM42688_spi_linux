#include "IMU_spi.h"

#include <thread>
uint64_t micro_time();
class ICM42688
{
    public:
        ICM42688();
        void imuDataGet();
        void SPIinit(int spinum,int csnum, uint8_t aMode, uint8_t gMode, uint8_t Ascale,uint8_t Gscale,uint8_t AODR,uint8_t GODR);

        void Close(void);
        std::thread imuDataGet_thread(void);
        float time_interval;
        float ex_i;
        float ey_i;
        float ez_i;
        float Kp;
        float Ki;
        float q0,q1,q2,q3;
        IMU_ST_QUATERNION_DATA stQuaternion;
        IMU_ST_SENSOR_DATA stGyroRawData;
        IMU_ST_SENSOR_DATA stAccelRawData;
    private:
        
        state_check Check(void);
        void Init(uint8_t aMode, uint8_t gMode, uint8_t Ascale,uint8_t Gscale,uint8_t AODR,uint8_t GODR);
        void GyroRead(int16_t& s16X, int16_t& s16Y, int16_t& s16Z);
        void AccelRead(int16_t& s16X, int16_t& s16Y, int16_t& s16Z);
        void GyroOffset(void);

        void mahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
        float Ascale_factor;
        float Gscale_factor;


        IMU_ST_SENSOR_DATA gstGyroOffset;
        int fd;
        int *fd_address;

};

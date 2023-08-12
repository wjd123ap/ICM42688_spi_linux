#include "ICM42688.hpp"
#include <iostream>



int main(int argc, char* argv[]){
    ICM42688 imu1;
    
    imu1.SPIinit(0,0,aMode_LN,gMode_LN,AFS_2G,GFS_31_25DPS,AODR_1kHz,GODR_1kHz);
    long start = micro_time();
    
    long end = micro_time();
    for (int i=0;i<100;++i){
    imu1.imuDataGet();
    std::cout<<"Accel_X:"<<imu1.stAccelRawData.s16X<< "\tAccel_Y:"<<imu1.stAccelRawData.s16Y<<"\tAccel_Z:" << imu1.stAccelRawData.s16Z<<std::endl;
    std::cout<<"gyro_X:"<<imu1.stGyroRawData.s16X<< "\tgyro_Y:"<<imu1.stGyroRawData.s16Y<<"\tgyro_Z:" << imu1.stGyroRawData.s16Z<<std::endl;
    usleep(2000);
    }
    imu1.Close();

}
#include "INS_IMU.h"
#include "bsp_imu.h"
#include "KalmanFilter.h"
#include "Calibration.h"

extern IMU_FLOAT_DATA_T imu_real_data;

INS_t INS;
uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;


void IMU_AHRS_sCalcu_task(void)
{
     static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};//TODO
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    INS.Accel[X] =imu_real_data.Accel.X;
    INS.Accel[Y] = imu_real_data.Accel.Y;
    INS.Accel[Z] = imu_real_data.Accel.Z;
    INS.Gyro[X] =imu_real_data.Gyro.X;
    INS.Gyro[Y] = imu_real_data.Gyro.Y;
    INS.Gyro[Z] = imu_real_data.Gyro.Z;

          // demo function,����������װ���,���Բ���,��demo��ʱû��
     //   IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

            // �����������ٶ�ʸ����bϵ��XY����ļн�,������������չ,��demo��ʱû��
        INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

    // ���ĺ���,EKF������Ԫ��
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);
        
           
}
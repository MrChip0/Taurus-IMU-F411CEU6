/**
 * @file     bsp_imu.c
 * @version  v1.0
 * @date     2020.1.10
 *
 * @brief    IMU�������
 *
 *	@author   YY
 *
 */

#include "bsp_imu.h"
#include "KalmanFilter.h"
#include "Calibration.h"
#include "QuaternionEKF.h"
#include "bsp_dwt.h"

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define gNORM 9.69293118f
float BMI088_AccelScale = 9.783f / gNORM;

imu_mode_e imu_mode;
// #define Kp 1.35f    // proportional gain governs rate of convergence to accelerometer/magnetometer
// #define Ki 0.002f   // integral gain governs rate of convergence of gyroscope biases
extern float AccRatioOffset;
volatile float exInt, eyInt, ezInt; // ������
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;
// float halfT=0.001;	 //�������ڵ�һ��
float halfT = 0.0005;
float Kp = 0.1f;
// float Ki=0.002f;
float Ki = 0.01f;
float ahrs_count;
float ahrs_norm;
float ez_test;
float norm_test;
float imu_odom_vx, odom_x, last_time, d_time, time, last_time;
// float test_vz;
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;

volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ ms

IMU_FLOAT_DATA_T imu_real_data;
float pitch_angle_test, kf2_pitch_angle;
float *kf2_pitch;

/**
 * @brief IMU_Values_Convert
 * @param
 * @attention
 * @note  IMU���ݵ�λ����
 */
void IMU_Values_Convert(void)
{

    time = HAL_GetTick();
    d_time = time - last_time;
    imu_real_data.Gyro.X = imu_output_data.Gyro.X / 16.384f / 57.29577951308f; // ���ٶȵ�λLSB->rad/sBMI088_AccelScale
    imu_real_data.Gyro.Y = imu_output_data.Gyro.Y / 16.384f / 57.29577951308f; // ���ٶȵ�λLSB->rad/sBMI088_AccelScale
    imu_real_data.Gyro.Z = imu_output_data.Gyro.Z / 16.384f / 57.29577951308f; // ���ٶȵ�λLSB->rad/sBMI088_AccelScale
    //	imu_real_data.Accel.X = imu_output_data.Accel.X/1365.0f-Bias.Accel.X;
    //	imu_real_data.Accel.Y = imu_output_data.Accel.Y/1365.0f-Bias.Accel.Y;
    //	imu_real_data.Accel.Z = imu_output_data.Accel.Z/1365.0f-Bias.Accel.Z; 			//���ٶ�AD->g
    imu_real_data.Accel.X = imu_output_data.Accel.X * BMI088_ACCEL_3G_SEN * BMI088_AccelScale;
    imu_real_data.Accel.Y = imu_output_data.Accel.Y * BMI088_ACCEL_3G_SEN * BMI088_AccelScale;
    imu_real_data.Accel.Z = imu_output_data.Accel.Z * BMI088_ACCEL_3G_SEN * BMI088_AccelScale;
    last_time = time;

    imu_real_data.Mag.X = -imu_output_data.Mag.X;
    imu_real_data.Mag.Y = imu_output_data.Mag.Y;
    imu_real_data.Mag.Z = imu_output_data.Mag.Z; // ����������ϵ��һ

    // Uart2Printf("%7f,%7f,%7f,%7f,%7f,%7f\n", imu_real_data.Gyro.X,imu_real_data.Gyro.Y,imu_real_data.Gyro.Z,imu_real_data.Gyro.X,imu_real_data.Gyro.Y,imu_real_data.Gyro.Z);
}

/**
 * @brief IMU_AHRS_Calcu
 * @param
 * @attention
 * @note
 */
void IMU_AHRS_Calcu(void)
{
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float tempq0, tempq1, tempq2, tempq3;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    ahrs_count++;
    gx = imu_real_data.Gyro.X;
    gy = imu_real_data.Gyro.Y;
    gz = imu_real_data.Gyro.Z;

    ax = imu_real_data.Accel.X;
    ay = imu_real_data.Accel.Y;
    az = imu_real_data.Accel.Z;

    norm = invSqrt(ax * ax + ay * ay + az * az);
    norm_test = 1 / norm;
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    if (fabs(norm_test - 1) > 0.03)
    // Kp =0;
    {
        Kp = 0.0;
        Ki = 0.0;
    }
    else
    // Kp=0.03;
    {
        Kp = 0.6;
        Ki = 0.0;
    }
    //  Kp=2;
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;
    // compute reference direction of flux
    hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
    hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
    hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    ez_test = ez;
    if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;
        ezInt = ezInt + ez * Ki * halfT;

        gx = gx + Kp * ex + exInt;
        gy = gy + Kp * ey + eyInt;
        gz = gz + Kp * ez + ezInt;
        // gz=gz;
    }

    tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    tempq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    tempq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    tempq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    if (ahrs_count > 2000)
    {
        norm = invSqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
        q0 = tempq0 * norm;
        q1 = tempq1 * norm;
        q2 = tempq2 * norm;
        q3 = tempq3 * norm;
    }
    if (ahrs_count < 2000)
    {
        ahrs_norm = sqrtf(imu_real_data.Accel.X * imu_real_data.Accel.X + imu_real_data.Accel.Y * imu_real_data.Accel.Y + imu_real_data.Accel.Z * imu_real_data.Accel.Z);
        if (fabs(ahrs_norm - 1) < 0.1f && imu_real_data.Gyro.X < 0.1f && imu_real_data.Gyro.Y < 0.1f && imu_real_data.Gyro.Z < 0.1f)
        {
            if (imu_real_data.Accel.Z / ahrs_norm >= 0)
            {

                q0 = sqrtf((1.0f + imu_real_data.Accel.Z / ahrs_norm) / 2.0f);
                q1 = imu_real_data.Accel.Y / ahrs_norm / sqrtf(2.0f * (imu_real_data.Accel.Z / ahrs_norm + 1.0f));
                q2 = -imu_real_data.Accel.X / ahrs_norm / sqrtf(2.0f * (imu_real_data.Accel.Z / ahrs_norm + 1.0f));
                q3 = 0;
            }
            else
            {
                q0 = imu_real_data.Accel.Y / ahrs_norm / sqrtf(2.0f * (1.0f - imu_real_data.Accel.Z / ahrs_norm));
                q1 = sqrtf((1.0f - imu_real_data.Accel.Z / ahrs_norm));
                q2 = 0;
                q3 = imu_real_data.Accel.X / ahrs_norm / sqrtf(2.0f * (1.0f - imu_real_data.Accel.Z / ahrs_norm));
            }
        }
    }
    imu_real_data.q[0] = q0;
    imu_real_data.q[1] = q1;
    imu_real_data.q[2] = q2;
    imu_real_data.q[3] = q3;
    // ��Ԫ��ת��ŷ����
    imu_real_data.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3 + 180.0f; // yaw
    imu_real_data.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                        // pitch
    imu_real_data.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;         // roll
}

/*wanghongxi 方案*/
INS_t INS;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;

void IMU_AHRS_Calcu_task(void)
{

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;
    INS.AccelLPF = 0.0085;

    INS.Accel[X_axis] = imu_real_data.Accel.X;
    INS.Accel[Y_axis] = imu_real_data.Accel.Y;
    INS.Accel[Z_axis] = imu_real_data.Accel.Z;
    INS.Gyro[X_axis] = imu_real_data.Gyro.X;
    INS.Gyro[Y_axis] = imu_real_data.Gyro.Y;
    INS.Gyro[Z_axis] = imu_real_data.Gyro.Z;

    // demo function,用于修正安装误差,可以不管,本demo暂时没用
    // IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

    // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
    INS.atanxz = -atan2f(INS.Accel[X_axis], INS.Accel[Z_axis]) * 180 / PI;
    INS.atanyz = atan2f(INS.Accel[Y_axis], INS.Accel[Z_axis]) * 180 / PI;
    // Uart2Printf("%7f,%7f\n", INS.atanxz,INS.atanyz);

    // 扩展卡尔曼核心函数
    IMU_QuaternionEKF_Update_Chip(INS.Gyro[X_axis], INS.Gyro[Y_axis], INS.Gyro[Z_axis], INS.Accel[X_axis], INS.Accel[Y_axis], INS.Accel[Z_axis], dt);

    //     memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    //     // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    //     BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    //     BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    //     BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    //             // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    //    float gravity_b[3];
    //    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    //    for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
    //    {
    //        INS.MotionAccel_b[i] = INS.Accel[i];
    //    }
    //    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

    //     EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    //     for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
    //     {
    //         INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    //     }
    // 			BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n
    // 获取最终数据
    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Pitch;
    INS.Roll = QEKF_INS.Roll;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
    // //赋值给输出接口
    imu_real_data.yaw = INS.Yaw;
    imu_real_data.pitch = INS.Pitch; // pitch
    imu_real_data.roll = INS.Roll;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

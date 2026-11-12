#include "classify/imu_activity.h"

#include "imu_tree.h"   /* generated: imu_classify(), imu_class_name[], IMU_N_FEATURES, IMU_N_CLASSES */

#if IMU_N_FEATURES != IMU_N_FEAT
#error "imu_tree.h was generated for a different feature list than classify/imu_window.c computes"
#endif

#define ACCEL_LSB_PER_G    16384
#define GYRO_LSB_PER_DPS   131

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void imu_activity_init(imu_activity_t *a)
{
    imu_window_init(&a->win);
    a->last_seq = 0;
    a->have_last = 0;
    a->windows = 0;
}

int imu_activity_push_raw(imu_activity_t *a, uint16_t seq, const uint8_t raw[14], IMU_FP_T feat[IMU_N_FEAT])
{
    const int gap = a->have_last && (uint16_t)(seq - a->last_seq) != 1u;
    a->last_seq = seq;
    a->have_last = 1;

    IMU_FP_T s[6];
    s[0] = (IMU_FP_T)be16(raw + 0) / (IMU_FP_T)ACCEL_LSB_PER_G;
    s[1] = (IMU_FP_T)be16(raw + 2) / (IMU_FP_T)ACCEL_LSB_PER_G;
    s[2] = (IMU_FP_T)be16(raw + 4) / (IMU_FP_T)ACCEL_LSB_PER_G;
    s[3] = (IMU_FP_T)be16(raw + 8) / (IMU_FP_T)GYRO_LSB_PER_DPS;
    s[4] = (IMU_FP_T)be16(raw + 10) / (IMU_FP_T)GYRO_LSB_PER_DPS;
    s[5] = (IMU_FP_T)be16(raw + 12) / (IMU_FP_T)GYRO_LSB_PER_DPS;

    if (!imu_window_push(&a->win, s, gap != 0))
        return -1;

    IMU_FP_T f[IMU_N_FEAT];
    imu_window_features(&a->win, f);
    float ff[IMU_N_FEATURES];
    for (unsigned i = 0; i < IMU_N_FEAT; i++) {
        ff[i] = (float)f[i];
        if (feat) feat[i] = f[i];
    }
    a->windows++;
    return imu_classify(ff);
}

const char *imu_activity_name(int cls)
{
    return (cls >= 0 && cls < IMU_N_CLASSES) ? imu_class_name[cls] : "?";
}

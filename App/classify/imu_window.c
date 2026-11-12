#include "classify/imu_window.h"

#include <math.h>
#include <string.h>

/* sqrt for the chosen type (float uses sqrtf, which is a single FPU instruction on the Cortex-M7) */
#define FP_SQRT(x) (sizeof(IMU_FP_T) == sizeof(float) ? (IMU_FP_T)sqrtf((float)(x)) : (IMU_FP_T)sqrt((double)(x)))

void imu_window_init(imu_window_t *w)
{
    memset(w, 0, sizeof *w);
}

bool imu_window_push(imu_window_t *w, const IMU_FP_T s[6], bool gap)
{
    memcpy(w->buf[w->head], s, 6 * sizeof(IMU_FP_T));
    w->head = (uint16_t)((w->head + 1u) % IMU_WIN_SIZE);

    if (gap)
        w->clean = 1;                       /* this sample starts a fresh run */
    else if (w->clean < IMU_WIN_SIZE)
        w->clean++;
    else
        w->phase = (uint16_t)((w->phase + 1u) % IMU_WIN_STEP);

    if (gap || w->clean < IMU_WIN_SIZE) {
        w->phase = 0;
        return false;
    }
    /* clean == SIZE: either the run just reached a full window (phase 0) or the phase says so */
    return w->phase == 0;
}

void imu_window_features(const imu_window_t *w, IMU_FP_T f[IMU_N_FEAT])
{
    const unsigned n = IMU_WIN_SIZE;
    IMU_FP_T mean[6] = {0}, var[6] = {0};
    IMU_FP_T amag[IMU_WIN_SIZE];
    IMU_FP_T amean = 0, gmean = 0, amin = 0, amax = 0, jerk = 0;
    IMU_FP_T gmag[IMU_WIN_SIZE];

    /* pass 1: sums, magnitudes (oldest sample first: the ring's head is the oldest once full) */
    for (unsigned i = 0; i < n; i++) {
        const IMU_FP_T *s = w->buf[(w->head + i) % IMU_WIN_SIZE];
        for (unsigned k = 0; k < 6; k++)
            mean[k] += s[k];
        amag[i] = FP_SQRT(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
        gmag[i] = FP_SQRT(s[3] * s[3] + s[4] * s[4] + s[5] * s[5]);
        amean += amag[i];
        gmean += gmag[i];
        if (i == 0 || amag[i] < amin) amin = amag[i];
        if (i == 0 || amag[i] > amax) amax = amag[i];
        if (i > 0) {
            IMU_FP_T d = amag[i] - amag[i - 1];
            jerk += d < 0 ? -d : d;
        }
    }
    for (unsigned k = 0; k < 6; k++)
        mean[k] /= (IMU_FP_T)n;
    amean /= (IMU_FP_T)n;
    gmean /= (IMU_FP_T)n;
    jerk /= (IMU_FP_T)(n - 1);

    /* pass 2: variances (two-pass: no cancellation) and the zero crossings of the mean-removed gy */
    IMU_FP_T avar = 0, gvar = 0;
    unsigned zcr = 0;
    int prev_neg = 0;
    for (unsigned i = 0; i < n; i++) {
        const IMU_FP_T *s = w->buf[(w->head + i) % IMU_WIN_SIZE];
        for (unsigned k = 0; k < 6; k++) {
            IMU_FP_T d = s[k] - mean[k];
            var[k] += d * d;
        }
        IMU_FP_T da = amag[i] - amean, dg = gmag[i] - gmean;
        avar += da * da;
        gvar += dg * dg;
        int neg = (s[4] - mean[4]) < 0;
        if (i > 0 && neg != prev_neg)
            zcr++;
        prev_neg = neg;
    }

    f[0] = mean[0]; f[1] = mean[1]; f[2] = mean[2];
    f[3] = FP_SQRT(var[0] / (IMU_FP_T)n);
    f[4] = FP_SQRT(var[1] / (IMU_FP_T)n);
    f[5] = FP_SQRT(var[2] / (IMU_FP_T)n);
    f[6] = FP_SQRT(var[3] / (IMU_FP_T)n);
    f[7] = FP_SQRT(var[4] / (IMU_FP_T)n);
    f[8] = FP_SQRT(var[5] / (IMU_FP_T)n);
    f[9] = amean;
    f[10] = FP_SQRT(avar / (IMU_FP_T)n);
    f[11] = amin;
    f[12] = amax;
    f[13] = gmean;
    f[14] = FP_SQRT(gvar / (IMU_FP_T)n);
    f[15] = jerk;
    f[16] = (IMU_FP_T)zcr;
}

#ifndef APP_CLASSIFY_IMU_WINDOW_H
#define APP_CLASSIFY_IMU_WINDOW_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Sliding window over IMU samples plus the 17 window features.  The features and their order are
 * the same as FEATURES in tools/imu_features.py (which is the reference; tests compare the two).
 *
 * A window is IMU_WIN_SIZE samples (2 s at ~100 Hz); a new one is ready every IMU_WIN_STEP samples.
 * A window never contains a gap: when events were lost, the window restarts with the sample that
 * follows the hole (so the first window after a gap is ready IMU_WIN_SIZE samples later).
 * Without gaps the windows are exactly the ones tools/imu_features.py cuts (starts 0, 100, 200, ...).
 *
 * Arithmetic type: IMU_FP_T (default float; the host tests also build it as double).
 */
#define IMU_WIN_SIZE   200u
#define IMU_WIN_STEP   100u
#define IMU_N_FEAT     17u

#ifndef IMU_FP_T
#define IMU_FP_T float
#endif

typedef struct {
    IMU_FP_T buf[IMU_WIN_SIZE][6];   /* ax ay az (g), gx gy gz (deg/s); ring buffer */
    uint16_t head;                   /* next write position = oldest sample once full */
    uint16_t clean;                  /* samples since the last gap, saturates at IMU_WIN_SIZE */
    uint16_t phase;                  /* samples since the last window, counted once clean is saturated */
} imu_window_t;

void imu_window_init(imu_window_t *w);

/**
 * Add one sample (physical units).  `gap` = events were lost right before this sample.
 * @return true if a window is now complete; call imu_window_features() before the next push.
 */
bool imu_window_push(imu_window_t *w, const IMU_FP_T s[6], bool gap);

/** Features of the newest IMU_WIN_SIZE samples, order = FEATURES in tools/imu_features.py. */
void imu_window_features(const imu_window_t *w, IMU_FP_T f[IMU_N_FEAT]);

#ifdef __cplusplus
}
#endif

#endif /* APP_CLASSIFY_IMU_WINDOW_H */

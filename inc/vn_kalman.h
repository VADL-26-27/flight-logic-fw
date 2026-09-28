/*
 * vn_kalman.h
 *
 * Vertical-channel Kalman filter for VectorNav VN-200 data.
 * Estimates altitude, vertical velocity and vertical accel bias.
 *
 * Usage:
 *   #include "vn_kalman.h"
 *
 *   vn_kf_t kf;
 *   vn_kf_init(&kf, 0.5f, 0.01f, 0.5f);
 *
 *   // for every packet:
 *   vn_kf_step(&kf, yaw, pitch, roll, ax, ay, az, pressure, dt);
 *   float h = vn_kf_altitude(&kf);
 *   float v = vn_kf_velocity(&kf);
 *
 * Units: angles deg, accel g, pressure kPa, dt s.
 */
#ifndef VN_KALMAN_H
#define VN_KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x[3];        /* [h (m, up), v (m/s, up), accel bias (m/s^2)] */
    float P[3][3];     /* state covariance */

    float sigma_accel; /* vertical accel noise, m/s^2 */
    float sigma_bias;  /* bias random walk, m/s^2 per sqrt(s) */
    float sigma_baro;  /* baro altitude noise, m */

    float p0_kpa;      /* reference pressure, set from first sample */
    int   initialized;

    /* diagnostics from the last step */
    float accel_up;    /* earth-frame vertical accel, m/s^2 */
    float baro_alt;    /* raw baro altitude, m */
    float innovation;  /* baro_alt - predicted altitude, m */
} vn_kf_t;

/* Call once before the first packet. */
void vn_kf_init(vn_kf_t *kf, float sigma_accel, float sigma_bias, float sigma_baro);

/* Call once per packet. The first call only records the reference pressure. */
void vn_kf_step(vn_kf_t *kf,
                float yaw, float pitch, float roll,
                float ax, float ay, float az,
                float pressure_kpa, float dt);

float vn_kf_altitude(const vn_kf_t *kf);  /* m above start, up positive */
float vn_kf_velocity(const vn_kf_t *kf);  /* m/s, up positive */
float vn_kf_bias(const vn_kf_t *kf);      /* m/s^2 */

#ifdef __cplusplus
}
#endif

#endif /* VN_KALMAN_H */

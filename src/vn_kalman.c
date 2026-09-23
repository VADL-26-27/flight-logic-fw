/*
 * vn_kalman.c
 *
 * Vertical-channel Kalman filter for VectorNav VN-200 binary output.
 *
 * Inputs (as parsed from the 35-byte packet):
 *   yaw, pitch, roll   [deg]   (VN 3-2-1 Euler angles, body -> NED)
 *   ax, ay, az         [g]     (body-frame specific force, already / VN_G_CONST)
 *   pressure           [kPa]   (VN-200 barometric pressure)
 *
 * State:
 *   x[0] = h   altitude above start point      [m, up]
 *   x[1] = v   vertical velocity               [m/s, up]
 *   x[2] = b   vertical accelerometer bias     [m/s^2, up]
 *
 * Process model (driven by earth-frame vertical acceleration a):
 *   h' = h + v*dt + 0.5*(a - b)*dt^2
 *   v' = v + (a - b)*dt
 *   b' = b                                  (random walk)
 *
 * Measurement: barometric altitude relative to the first pressure sample.
 *
 * Build test: gcc -O2 -DVN_KF_TEST vn_kalman.c -lm -o vn_kf_test
 */

#include "vn_kalman.h"

#include <math.h>
#include <string.h>

#ifndef VN_G_CONST
#define VN_G_CONST 9.80665f
#endif

#define VN_DEG2RAD 0.017453292519943295f

/* ---------------------------------------------------------------------- */

void vn_kf_init(vn_kf_t *kf, float sigma_accel, float sigma_bias, float sigma_baro)
{
    memset(kf, 0, sizeof(*kf));
    kf->sigma_accel = sigma_accel;
    kf->sigma_bias  = sigma_bias;
    kf->sigma_baro  = sigma_baro;

    kf->P[0][0] = 1.0f;    /* m^2 */
    kf->P[1][1] = 1.0f;    /* (m/s)^2 */
    kf->P[2][2] = 0.1f;    /* (m/s^2)^2 */
}

/* Pressure [kPa] -> altitude [m] relative to p0, standard atmosphere. */
static float altitude_calculation(float p_kpa, float p0_kpa)
{
    return 44330.0f * (1.0f - powf(p_kpa / p0_kpa, 0.190295f));
}

/*
 * Rotate body specific force into NED, add gravity, return upward accel.
 * Only the third row of the body->NED DCM is needed, so yaw drops out.
 * VN convention: at rest and level, az reads about -1 g.
 */
static float vertical_accel_up(float pitch_deg, float roll_deg,
                               float ax_g, float ay_g, float az_g)
{
    float th  = pitch_deg * VN_DEG2RAD;
    float phi = roll_deg  * VN_DEG2RAD;

    float fx = ax_g * VN_G_CONST;
    float fy = ay_g * VN_G_CONST;
    float fz = az_g * VN_G_CONST;

    float f_down = -sinf(th) * fx
                 +  sinf(phi) * cosf(th) * fy
                 +  cosf(phi) * cosf(th) * fz;

    float a_down = f_down + VN_G_CONST;   /* kinematic accel, NED down */
    return -a_down;                       /* up positive */
}

static void predict(vn_kf_t *kf, float a_up, float dt)
{
    float dt2 = dt * dt;
    float F[3][3] = {
        { 1.0f, dt,   -0.5f * dt2 },
        { 0.0f, 1.0f, -dt         },
        { 0.0f, 0.0f,  1.0f       },
    };

    /* state */
    float a = a_up - kf->x[2];
    kf->x[0] += kf->x[1] * dt + 0.5f * a * dt2;
    kf->x[1] += a * dt;

    /* P = F P F^T */
    float FP[3][3], Pn[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            FP[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) FP[i][j] += F[i][k] * kf->P[k][j];
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            Pn[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) Pn[i][j] += FP[i][k] * F[j][k];
        }

    /* + Q: accel noise through G = [dt^2/2, dt, 0], plus bias random walk */
    float qa = kf->sigma_accel * kf->sigma_accel;
    float g0 = 0.5f * dt2, g1 = dt;
    Pn[0][0] += g0 * g0 * qa;
    Pn[0][1] += g0 * g1 * qa;
    Pn[1][0] += g1 * g0 * qa;
    Pn[1][1] += g1 * g1 * qa;
    Pn[2][2] += kf->sigma_bias * kf->sigma_bias * dt;

    memcpy(kf->P, Pn, sizeof(Pn));
}

static void update_baro(vn_kf_t *kf, float z)
{
    float y = z - kf->x[0];                       /* H = [1 0 0] */
    float S = kf->P[0][0] + kf->sigma_baro * kf->sigma_baro;
    float K[3] = { kf->P[0][0] / S, kf->P[1][0] / S, kf->P[2][0] / S };

    for (int i = 0; i < 3; i++) kf->x[i] += K[i] * y;

    float P0[3] = { kf->P[0][0], kf->P[0][1], kf->P[0][2] };
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            kf->P[i][j] -= K[i] * P0[j];

    /* keep symmetric */
    for (int i = 0; i < 3; i++)
        for (int j = i + 1; j < 3; j++) {
            float m = 0.5f * (kf->P[i][j] + kf->P[j][i]);
            kf->P[i][j] = kf->P[j][i] = m;
        }

    kf->innovation = y;
}

/*
 * Run one filter step per VN packet. dt = time since previous packet [s].
 * yaw is accepted to match the packet layout; it does not affect the
 * vertical channel.
 */
void vn_kf_step(vn_kf_t *kf,
                float yaw, float pitch, float roll,
                float ax, float ay, float az,
                float pressure_kpa, float dt)
{
    (void)yaw;

    if (!kf->initialized) {
        kf->p0_kpa = pressure_kpa;
        kf->initialized = 1;
        kf->baro_alt = 0.0f;
        return;
    }

    kf->accel_up = vertical_accel_up(pitch, roll, ax, ay, az);
    kf->baro_alt = altitude_calculation(pressure_kpa, kf->p0_kpa);

    predict(kf, kf->accel_up, dt);
    update_baro(kf, kf->baro_alt);
}

float vn_kf_altitude(const vn_kf_t *kf) { return kf->x[0]; }
float vn_kf_velocity(const vn_kf_t *kf) { return kf->x[1]; }
float vn_kf_bias(const vn_kf_t *kf)     { return kf->x[2]; }

/* ---------------------------------------------------------------------- */
#ifdef VN_KF_TEST
#include <stdio.h>
#include <stdlib.h>

static float randn(void)
{
    float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    float u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

int main(void)
{
    vn_kf_t kf;
    vn_kf_init(&kf, 0.5f, 0.01f, 0.5f);

    const float dt = 0.01f;           /* 100 Hz */
    const float true_bias = 0.05f;    /* m/s^2, upward accel error */
    const float p0 = 101.325f;        /* kPa */

    /* 10 s at rest, then climb at 1 m/s^2 for 3 s, then coast 5 s */
    float h = 0.0f, v = 0.0f;
    for (int n = 0; n <= 1800; n++) {
        float t = n * dt;
        float a_true = (t >= 10.0f && t < 13.0f) ? 1.0f : 0.0f;
        v += a_true * dt;
        h += v * dt;

        /* level sensor: az = -(g + a_up)/g, plus bias and noise */
        float az = -(VN_G_CONST + a_true + true_bias + 0.05f * randn()) / VN_G_CONST;
        float p = p0 * powf(1.0f - h / 44330.0f, 1.0f / 0.190295f)
                + 0.006f * randn();   /* ~0.5 m of baro noise */

        vn_kf_step(&kf, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, az, p, dt);

        if (n % 200 == 0)
            printf("t=%5.2f  true h=%7.3f v=%6.3f | est h=%7.3f v=%6.3f b=%7.4f\n",
                   t, h, v, vn_kf_altitude(&kf), vn_kf_velocity(&kf), vn_kf_bias(&kf));
    }
    return 0;
}
#endif

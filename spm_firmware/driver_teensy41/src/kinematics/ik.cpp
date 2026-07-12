#include "ik.h"
#include <math.h>

// internal helpers

// rotation matrices; weird convention -> x (yaw), y (pitch), z (roll)
static void Rx(float a, float R[3][3]) {
    R[0][0]=1; R[0][1]=0;       R[0][2]=0;
    R[1][0]=0; R[1][1]=cosf(a); R[1][2]=-sinf(a);
    R[2][0]=0; R[2][1]=sinf(a); R[2][2]=cosf(a);
}

static void Ry(float a, float R[3][3]) {
    R[0][0]=cosf(a);  R[0][1]=0;  R[0][2]=sinf(a);
    R[1][0]=0;        R[1][1]=1;  R[1][2]=0;
    R[2][0]=-sinf(a); R[2][1]=0;  R[2][2]=cosf(a);
}
 
static void Rz(float a, float R[3][3]) {
    R[0][0]=cosf(a);  R[0][1]=-sinf(a); R[0][2]=0;
    R[1][0]=sinf(a);  R[1][1]=cosf(a);  R[1][2]=0;
    R[2][0]=0;        R[2][1]=0;        R[2][2]=1;
}

// 3x3 matrix multiplication (out = A * B)
static void mat_mul(float A[3][3], float B[3][3], float out[3][3]) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            out[i][j] = 0;
            for(int k = 0; k < 3; k++) {
                out[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 3x3 matrix * 3x1 vector
static void mat_vec(float M[3][3], float v[3], float out[3]) {
    for(int i = 0; i < 3; i++) {
        out[i] = 0;
        for(int j = 0; j < 3; j++) {
            out[i] += M[i][j] * v[j];
        }
    }
}

// composite rotation matrix
static void rotation_matrix(float roll, float pitch, float yaw, float R[3][3]) {
    float Rx_[3][3], Ry_[3][3], Rz_[3][3], temp[3][3];
    Rx(roll, Rx_);
    Ry(pitch, Ry_);
    Rz(yaw, Rz_);
    mat_mul(Rx_, Ry_, temp);
    mat_mul(temp, Rz_, R);
}

// home configuration joint vectors
static void get_vi_home(float beta, float vi_home[3][3]) {
    for(int i = 0; i < 3; i++) {
        float eta_i = 2.0f * i * M_PI / 3.0f;
        vi_home[i][0] = cosf(eta_i) * sinf(beta);
        vi_home[i][1] = sinf(eta_i) * sinf(beta);
        vi_home[i][2] = cosf(beta);
    }
}


// branch tracking

// initialize to home position
static float _prev_thetas[3] = {
    BETA, BETA, BETA
};
static bool _initialized = false;

void ik_reset_home() {
    _prev_thetas[0] = BETA;
    _prev_thetas[1] = BETA;
    _prev_thetas[2] = BETA;
    _initialized = true;
}


// inverse kinematics

IKResult ik(float roll, float pitch, float yaw) {
    IKResult result;
    result.valid = false;

    if(!_initialized) {
        ik_reset_home();
    }

    // get desired rotation matrix
    float R[3][3]
    rotation_matrix(roll, pitch, yaw, R);

    // home platform joint vectors
    float vi_home[3][3];
    get_vi_home(BETA, vi_home);

    float sa1 = sinf(ALPHA1);
    float ca1 = cosf(ALPHA1);
    float cos_a2 = cosf(ALPHA2);

    for(int i = 0; i < 3; i++) {
        float eta_i = 2.0f * i * M_PI / 3.0f;

        // get rotated vi vectors
        float vi[3];
        mat_vec(R, vi_home[i], vi);

        // get coefficients A, B, C
        float ce = cosf(eta_i);
        float se = sinf(eta_i);
 
        float A = sa1 * (ce * vi[0] + se * vi[1]);
        float B = sa1 * (se * vi[0] - ce * vi[1]);
        float C = cos_a2 + ca1 * vi[2];
 
        // solve quadratic
        float qa = C + A;
        float qb = -2.0f * B;
        float qc = C - A;

        float discriminant = (qb * qb) - (4.0f * qa * qc);

        if(discriminant < 0.0f) {
            // no real solution (outside workspace)
            return result;
        }

        float theta_i;

        if (fabsf(qa) < 1e-8f) {
            // degenerate case
            if (fabsf(qb) < 1e-8f) return result;
            float t = -qc / qb;
            float s = 2.0f*t / (1.0f + t*t);
            float c = (1.0f - t*t) / (1.0f + t*t);
            theta_i = atan2f(s, c);
        } else {
            float sq = sqrtf(discriminant);
            float t1 = (-qb + sq) / (2.0f * qa);
            float t2 = (-qb - sq) / (2.0f * qa);
 
            // solve transcendental with atan2 to generate continuous trajectories
            float s1 = 2.0f*t1 / (1.0f + t1*t1);
            float c1 = (1.0f - t1*t1) / (1.0f + t1*t1);
            float theta_t1 = atan2f(s1, c1);
 
            float s2 = 2.0f*t2 / (1.0f + t2*t2);
            float c2 = (1.0f - t2*t2) / (1.0f + t2*t2);
            float theta_t2 = atan2f(s2, c2);
 
            // branch continuity
            // accumulate onto prev_theta for continuous tracking
            float delta1 = theta_t1 - _prev_thetas[i];
            float delta2 = theta_t2 - _prev_thetas[i];
 
            // normalize to (-pi, pi)
            delta1 = fmodf(delta1 + M_PI, 2.0f*M_PI) - M_PI;
            delta2 = fmodf(delta2 + M_PI, 2.0f*M_PI) - M_PI;
 
            // pick smallest delta, accumulate
            if (fabsf(delta1) < fabsf(delta2)) {
                theta_i = _prev_thetas[i] + delta1;
            } else {
                theta_i = _prev_thetas[i] + delta2;
            }
        }
 
        result.theta[i] = theta_i;
        _prev_thetas[i] = theta_i;  // update branch tracking
    }
 
    result.valid = true;
    return result;
}
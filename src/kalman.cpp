/**
 kalman.cpp
 
 A very simple and versatile Kalman filter
 
 @author Tobias Ebsen
 @version 1.0.0 23/4/2018

 */

#include "kalman.h"

KalmanModel::KalmanModel() {
    P.m00 = 1.f;
    P.m01 = -1.f;
    P.m10 = -1.f;
    P.m11 = 1.f;
}

void KalmanModel::predict(float dt, float u, Vec2 Q) {
    
    // State transition model (Sometimes called A)
    Mat2x2 F = Mat2x2(
        1, -dt,
        0, 1
    );
    
    // Control-input model
    Vec2 B = Vec2(
        dt,
        0);

    // Predict current state
    x = F * x + B * u;

    // Transposed transition model
    Mat2x2 Ft = F.transposed();

    // Estimation of (pre) error covariance matrix
    P = F * P * Ft + Q * dt;
}

void KalmanModel::correct(float z, float R) {
    
    // Observation model
    Mat2x2 H = Mat2x2(
        1, 1,
        0, 0
    );
    
    // Transposed observation model
    Mat2x2 Ht = H.transposed();
    
    // Innovation covariance
    Mat2x2 Sm = H * P * Ht + R;
    float S = Sm.m00;
    
    // Kalman gain
    Mat2x2 Km = P * Ht * (1/S);
    Vec2 K(Km.m00, Km.m10);
    
    // Update the state
    float y = z - x[0];
    x = x + K * y;
    
    // Identity matrix
    Mat2x2 I = Mat2x2(
        1, 0,
        0, 1);
    
    // Post error covariance matrix
    P = (I - Km * H) * P;
}

KalmanFilter::KalmanFilter() {
    pmodels = 0;
    nmodels = 0;
    Q.x = 0.01;
    Q.y = 0.01;
    R = 0.001;
}

KalmanFilter::KalmanFilter(unsigned int nmodels) : KalmanFilter() {
    init(nmodels);
}

void KalmanFilter::predict(float dt, float * pu, unsigned int nu) {
    for (unsigned int i=0; i<nmodels; i++) {
        KalmanModel & m = pmodels[i];
        m.predict(dt, i < nu ? pu[i] : 0, Q);
    }
}

void KalmanFilter::predict(float dt) {
    predict(dt, 0, 0);
}

void KalmanFilter::correct(float *pz, unsigned int nz) {
    for (unsigned int i=0; i<nmodels; i++) {
        KalmanModel & m = pmodels[i];
        if (i < nz)
            m.correct(pz[i], R);
    }
}

void KalmanFilter::correct(float z) {
    correct(&z, 1);
}

void KalmanFilter::get(float *px, unsigned int nx) {
    for (unsigned int i=0; i<nmodels; i++) {
        KalmanModel & m = pmodels[i];
        if (i < nx)
            px[i] = m.x[0];
    }
}

float KalmanFilter::get(unsigned int istate) {
    return pmodels[istate].x[0];
}

void KalmanFilter::set(float *px, unsigned int nx) {
    for (unsigned int i=0; i<nmodels; i++) {
        KalmanModel & m = pmodels[i];
        if (i < nx)
            m.x[0] = px[i];
    }
}

void KalmanFilter::set(float x, unsigned int istate) {
    pmodels[istate].x[0] = x;
}

Vec2::Vec2() {
    this->x = 0;
    this->y = 0;
}

Vec2::Vec2(float x, float y) {
    this->x = x;
    this->y = y;
}

Vec2 Vec2::operator*(const float &scalar) const {
    return Vec2(x * scalar, y * scalar);
}

Vec2 Vec2::operator*(const Vec2 &other) const {
    return Vec2(x * other.x, y * other.y);
}

Vec2 Vec2::operator+(const Vec2 &other) const {
    return Vec2(x + other.x, y + other.y);
}

float & Vec2::operator[](int index) {
    return index == 0 ? x : y;
}

Mat2x2::Mat2x2() {
    this->m00 = 0;
    this->m01 = 0;
    this->m10 = 0;
    this->m11 = 0;
}

Mat2x2::Mat2x2(float m00, float m01, float m10, float m11) {
    this->m00 = m00;
    this->m01 = m01;
    this->m10 = m10;
    this->m11 = m11;
}

Mat2x2 Mat2x2::transposed() {
    return Mat2x2(m00, m10, m01, m11);
}

Mat2x2 Mat2x2::inversed() {
    float det = 1 / (m00 * m11 - m01 * m10);
    return Mat2x2(m11, -m01, -m10, m00) * det;
}

Mat2x2 Mat2x2::operator*(const Mat2x2 &other) const {
    Mat2x2 m;
    m.m00 = this->m00 * other.m00 + this->m01 * other.m10;
    m.m01 = this->m00 * other.m01 + this->m01 * other.m11;
    m.m10 = this->m10 * other.m00 + this->m11 * other.m10;
    m.m11 = this->m10 * other.m01 + this->m11 * other.m11;
    return m;
}

Vec2 Mat2x2::operator*(const Vec2 &other) const {
    Vec2 v;
    v.x = this->m00 * other.x + this->m01 * other.y;
    v.y = this->m10 * other.x + this->m11 * other.y;
    return v;
}

Mat2x2 Mat2x2::operator * (const float & scalar) const {
    Mat2x2 m;
    m.m00 = this->m00 * scalar;
    m.m01 = this->m01 * scalar;
    m.m10 = this->m10 * scalar;
    m.m11 = this->m11 * scalar;
    return m;
}

Mat2x2 Mat2x2::operator+(const Vec2 &other) const {
    Mat2x2 m;
    m.m00 = this->m00 + other.x;
    m.m01 = this->m01 + other.x;
    m.m10 = this->m10 + other.y;
    m.m11 = this->m11 + other.y;
    return m;
}

Mat2x2 Mat2x2::operator+(const float &scalar) const {
    Mat2x2 m;
    m.m00 = this->m00 + scalar;
    m.m01 = this->m01 + scalar;
    m.m10 = this->m10 + scalar;
    m.m11 = this->m11 + scalar;
    return m;
}

Mat2x2 Mat2x2::operator-(const Mat2x2 &other) const {
    Mat2x2 m;
    m.m00 = this->m00 - other.m00;
    m.m01 = this->m01 - other.m01;
    m.m10 = this->m10 - other.m10;
    m.m11 = this->m11 - other.m11;
    return m;
}

// FkToIkBake - Matching and Baking Engine
// Math.h : minimal, Maya-compatible linear algebra.
//
// Conventions match Maya's MMatrix exactly so values can be passed through
// without transposition:
//   * row vectors, points transform as  p' = p * M
//   * translation lives in row 3 (m[3][0..2])
//   * world = local * parentWorld
//   * rotateOrder XYZ means X is applied first:  R = Rx * Ry * Rz
#pragma once

#include <cmath>
#include <cstdint>

namespace fk2ik {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEps = 1e-12;

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    constexpr Vec3(double ax, double ay, double az) : x(ax), y(ay), z(az) {}
    double& operator[](int i) { return (&x)[i]; }
    double operator[](int i) const { return (&x)[i]; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
};

inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalized(const Vec3& a) {
    const double l = length(a);
    return l > kEps ? a / l : Vec3{0, 0, 0};
}
// Component of v perpendicular to the unit axis a.
inline Vec3 perpendicular(const Vec3& v, const Vec3& unitAxis) {
    return v - unitAxis * dot(v, unitAxis);
}
// Rodrigues rotation of v about a unit axis by angle (right-handed).
inline Vec3 rotateAbout(const Vec3& v, const Vec3& unitAxis, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return v * c + cross(unitAxis, v) * s + unitAxis * (dot(unitAxis, v) * (1.0 - c));
}
// Signed angle from a to b around unitAxis (a, b assumed perpendicular to axis).
inline double signedAngle(const Vec3& a, const Vec3& b, const Vec3& unitAxis) {
    return std::atan2(dot(cross(a, b), unitAxis), dot(a, b));
}

struct Mat4 {
    double m[4][4];

    Mat4() { setIdentity(); }
    void setIdentity() {
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) m[r][c] = (r == c) ? 1.0 : 0.0;
    }
    static Mat4 identity() { return Mat4(); }
    static Mat4 fromArray(const double a[4][4]) {
        Mat4 o;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) o.m[r][c] = a[r][c];
        return o;
    }
    static Mat4 translation(const Vec3& t) {
        Mat4 o;
        o.m[3][0] = t.x; o.m[3][1] = t.y; o.m[3][2] = t.z;
        return o;
    }
    static Mat4 scale(const Vec3& s) {
        Mat4 o;
        o.m[0][0] = s.x; o.m[1][1] = s.y; o.m[2][2] = s.z;
        return o;
    }
    // Maya shear (xy, xz, yz) as used by MTransformationMatrix.
    static Mat4 shear(const Vec3& sh) {
        Mat4 o;
        o.m[1][0] = sh.x; o.m[2][0] = sh.y; o.m[2][1] = sh.z;
        return o;
    }

    Vec3 row(int r) const { return {m[r][0], m[r][1], m[r][2]}; }
    void setRow(int r, const Vec3& v) { m[r][0] = v.x; m[r][1] = v.y; m[r][2] = v.z; }
    Vec3 getTranslation() const { return row(3); }
    void setTranslation(const Vec3& t) { setRow(3, t); }

    Mat4 operator*(const Mat4& b) const {
        Mat4 o;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                o.m[r][c] = m[r][0] * b.m[0][c] + m[r][1] * b.m[1][c] +
                            m[r][2] * b.m[2][c] + m[r][3] * b.m[3][c];
        return o;
    }

    Vec3 transformPoint(const Vec3& p) const {
        return {p.x * m[0][0] + p.y * m[1][0] + p.z * m[2][0] + m[3][0],
                p.x * m[0][1] + p.y * m[1][1] + p.z * m[2][1] + m[3][1],
                p.x * m[0][2] + p.y * m[1][2] + p.z * m[2][2] + m[3][2]};
    }
    Vec3 transformVector(const Vec3& v) const {
        return {v.x * m[0][0] + v.y * m[1][0] + v.z * m[2][0],
                v.x * m[0][1] + v.y * m[1][1] + v.z * m[2][1],
                v.x * m[0][2] + v.y * m[1][2] + v.z * m[2][2]};
    }

    Mat4 transposed3x3() const {  // rotation inverse for orthonormal 3x3, zero translation
        Mat4 o;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) o.m[r][c] = m[c][r];
        return o;
    }

    // General 4x4 inverse (Maya matrices may contain shear / non-uniform scale).
    Mat4 inverse() const;

    // 3x3 part with each row normalized and made orthogonal (Gram-Schmidt on
    // rows 0,1, row2 = cross).  Translation preserved.  Removes scale/shear.
    Mat4 orthonormalized() const;

    // Rotation-only copy (orthonormalized, zero translation).
    Mat4 rotationOnly() const {
        Mat4 o = orthonormalized();
        o.m[3][0] = o.m[3][1] = o.m[3][2] = 0.0;
        return o;
    }
};

inline Mat4 Mat4::inverse() const {
    const double* a = &m[0][0];
    double inv[16];
    inv[0] = a[5] * a[10] * a[15] - a[5] * a[11] * a[14] - a[9] * a[6] * a[15] + a[9] * a[7] * a[14] + a[13] * a[6] * a[11] - a[13] * a[7] * a[10];
    inv[4] = -a[4] * a[10] * a[15] + a[4] * a[11] * a[14] + a[8] * a[6] * a[15] - a[8] * a[7] * a[14] - a[12] * a[6] * a[11] + a[12] * a[7] * a[10];
    inv[8] = a[4] * a[9] * a[15] - a[4] * a[11] * a[13] - a[8] * a[5] * a[15] + a[8] * a[7] * a[13] + a[12] * a[5] * a[11] - a[12] * a[7] * a[9];
    inv[12] = -a[4] * a[9] * a[14] + a[4] * a[10] * a[13] + a[8] * a[5] * a[14] - a[8] * a[6] * a[13] - a[12] * a[5] * a[10] + a[12] * a[6] * a[9];
    inv[1] = -a[1] * a[10] * a[15] + a[1] * a[11] * a[14] + a[9] * a[2] * a[15] - a[9] * a[3] * a[14] - a[13] * a[2] * a[11] + a[13] * a[3] * a[10];
    inv[5] = a[0] * a[10] * a[15] - a[0] * a[11] * a[14] - a[8] * a[2] * a[15] + a[8] * a[3] * a[14] + a[12] * a[2] * a[11] - a[12] * a[3] * a[10];
    inv[9] = -a[0] * a[9] * a[15] + a[0] * a[11] * a[13] + a[8] * a[1] * a[15] - a[8] * a[3] * a[13] - a[12] * a[1] * a[11] + a[12] * a[3] * a[9];
    inv[13] = a[0] * a[9] * a[14] - a[0] * a[10] * a[13] - a[8] * a[1] * a[14] + a[8] * a[2] * a[13] + a[12] * a[1] * a[10] - a[12] * a[2] * a[9];
    inv[2] = a[1] * a[6] * a[15] - a[1] * a[7] * a[14] - a[5] * a[2] * a[15] + a[5] * a[3] * a[14] + a[13] * a[2] * a[7] - a[13] * a[3] * a[6];
    inv[6] = -a[0] * a[6] * a[15] + a[0] * a[7] * a[14] + a[4] * a[2] * a[15] - a[4] * a[3] * a[14] - a[12] * a[2] * a[7] + a[12] * a[3] * a[6];
    inv[10] = a[0] * a[5] * a[15] - a[0] * a[7] * a[13] - a[4] * a[1] * a[15] + a[4] * a[3] * a[13] + a[12] * a[1] * a[7] - a[12] * a[3] * a[5];
    inv[14] = -a[0] * a[5] * a[14] + a[0] * a[6] * a[13] + a[4] * a[1] * a[14] - a[4] * a[2] * a[13] - a[12] * a[1] * a[6] + a[12] * a[2] * a[5];
    inv[3] = -a[1] * a[6] * a[11] + a[1] * a[7] * a[10] + a[5] * a[2] * a[11] - a[5] * a[3] * a[10] - a[9] * a[2] * a[7] + a[9] * a[3] * a[6];
    inv[7] = a[0] * a[6] * a[11] - a[0] * a[7] * a[10] - a[4] * a[2] * a[11] + a[4] * a[3] * a[10] + a[8] * a[2] * a[7] - a[8] * a[3] * a[6];
    inv[11] = -a[0] * a[5] * a[11] + a[0] * a[7] * a[9] + a[4] * a[1] * a[11] - a[4] * a[3] * a[9] - a[8] * a[1] * a[7] + a[8] * a[3] * a[5];
    inv[15] = a[0] * a[5] * a[10] - a[0] * a[6] * a[9] - a[4] * a[1] * a[10] + a[4] * a[2] * a[9] + a[8] * a[1] * a[6] - a[8] * a[2] * a[5];
    double det = a[0] * inv[0] + a[1] * inv[4] + a[2] * inv[8] + a[3] * inv[12];
    Mat4 o;
    if (std::fabs(det) < 1e-300) return o;  // singular: identity (caller validates)
    det = 1.0 / det;
    double* out = &o.m[0][0];
    for (int i = 0; i < 16; ++i) out[i] = inv[i] * det;
    return o;
}

inline Mat4 Mat4::orthonormalized() const {
    Mat4 o = *this;
    Vec3 x = normalized(row(0));
    Vec3 y = row(1);
    y = normalized(y - x * dot(x, y));
    Vec3 z = cross(x, y);
    // Preserve handedness of the source (mirrored matrices with negative det).
    if (dot(z, row(2)) < 0.0) z = -z;
    o.setRow(0, x);
    o.setRow(1, y);
    o.setRow(2, z);
    return o;
}

// ---------------------------------------------------------------------------
// Euler rotations with Maya rotate orders.
// Enum values match Maya's transform.rotateOrder attribute.
enum class RotateOrder : int { XYZ = 0, YZX = 1, ZXY = 2, XZY = 3, YXZ = 4, ZYX = 5 };

struct Euler {
    double x = 0, y = 0, z = 0;  // radians, stored per axis (not per order slot)
    double& operator[](int i) { return (&x)[i]; }
    double operator[](int i) const { return (&x)[i]; }
};

// Axis indices in application order for each Maya rotate order.
inline void rotateOrderAxes(RotateOrder ro, int& i, int& j, int& k) {
    i = 0; j = 1; k = 2;
    switch (ro) {
        case RotateOrder::XYZ: i = 0; j = 1; k = 2; break;
        case RotateOrder::YZX: i = 1; j = 2; k = 0; break;
        case RotateOrder::ZXY: i = 2; j = 0; k = 1; break;
        case RotateOrder::XZY: i = 0; j = 2; k = 1; break;
        case RotateOrder::YXZ: i = 1; j = 0; k = 2; break;
        case RotateOrder::ZYX: i = 2; j = 1; k = 0; break;
    }
}

inline Mat4 axisRotation(int axis, double a) {
    Mat4 o;
    const double c = std::cos(a), s = std::sin(a);
    const int p = (axis + 1) % 3, q = (axis + 2) % 3;
    // Row-vector convention: rows are images of basis vectors.
    o.m[p][p] = c;  o.m[p][q] = s;
    o.m[q][p] = -s; o.m[q][q] = c;
    return o;
}

inline Mat4 eulerToMatrix(const Euler& e, RotateOrder ro) {
    int i, j, k;
    rotateOrderAxes(ro, i, j, k);
    return axisRotation(i, e[i]) * axisRotation(j, e[j]) * axisRotation(k, e[k]);
}

// Extract Euler angles from the rotation part of an orthonormal matrix.
// Verified against eulerToMatrix for all six orders in the test-suite.
inline Euler matrixToEuler(const Mat4& rm, RotateOrder ro) {
    int i, j, k;
    rotateOrderAxes(ro, i, j, k);
    // Work with the column-convention matrix C = transpose(rm):  C = Rk*Rj*Ri.
    auto C = [&](int r, int c) { return rm.m[c][r]; };
    const bool even = ((j - i + 3) % 3) == 1;  // xyz, yzx, zxy
    const double sgn = even ? 1.0 : -1.0;
    Euler e;
    const double cy = std::sqrt(C(i, i) * C(i, i) + C(j, i) * C(j, i));
    if (cy > 1e-9) {
        e[i] = std::atan2(sgn * C(k, j), C(k, k));
        e[j] = std::atan2(-sgn * C(k, i), cy);
        e[k] = std::atan2(sgn * C(j, i), C(i, i));
    } else {  // gimbal: put everything in the first angle
        e[i] = std::atan2(-sgn * C(j, k), C(j, j));
        e[j] = std::atan2(-sgn * C(k, i), cy);
        e[k] = 0.0;
    }
    return e;
}

// Wrap angle into (-pi, pi].
inline double wrapPi(double a) {
    a = std::fmod(a + kPi, kTwoPi);
    if (a < 0) a += kTwoPi;
    return a - kPi;
}

// Return the Euler triple equivalent to `e` (same orientation) that is
// closest to `prev`, considering both the direct and the alternate
// (first+pi, pi-middle, last+pi) solutions plus 2*pi wraps on every axis.
// This is the core of the anti-flip filter.
inline Euler closestEuler(const Euler& e, const Euler& prev, RotateOrder ro) {
    int i, j, k;
    rotateOrderAxes(ro, i, j, k);
    Euler alt = e;
    alt[i] = e[i] + kPi;
    alt[j] = kPi - e[j];
    alt[k] = e[k] + kPi;
    auto unwrap = [&](Euler c) {
        for (int a = 0; a < 3; ++a) c[a] = prev[a] + wrapPi(c[a] - prev[a]);
        return c;
    };
    const Euler a = unwrap(e), b = unwrap(alt);
    auto dist = [&](const Euler& c) {
        double d = 0;
        for (int a2 = 0; a2 < 3; ++a2) d += (c[a2] - prev[a2]) * (c[a2] - prev[a2]);
        return d;
    };
    return dist(a) <= dist(b) ? a : b;
}

}  // namespace fk2ik

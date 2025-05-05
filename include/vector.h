#ifndef VECTOR_H
#define VECTOR_H
#include <cmath>

class Vector2 {
public:
    float x, y;

    // Constructors
    __host__ __device__ Vector2() : x(0), y(0) {}
    __host__ __device__ Vector2(float x, float y) : x(x), y(y) {}

    // Operator Overloads
    __host__ __device__ bool operator==(const Vector2& other) const {
        return x == other.x && y == other.y;
    }

    __host__ __device__ Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }

    __host__ __device__ Vector2& operator+=(const Vector2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    __host__ __device__ Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }

    __host__ __device__ Vector2& operator-=(const Vector2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    __host__ __device__ Vector2 operator*(float scalar) const {
        return Vector2(x * scalar, y * scalar);
    }

    __host__ __device__ Vector2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    __host__ __device__ Vector2 operator/(float scalar) const {
        return Vector2(x / scalar, y / scalar);
    }

    __host__ __device__ Vector2& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    // Dot product
    __host__ __device__ float dot(const Vector2& other) const {
        return x * other.x + y * other.y;
    }

    // Cross product
    __host__ __device__ float cross(const Vector2& other) const {
        return x * other.y - y * other.x;
    }

    // Magnitude
    __host__ __device__ float magnitude() const {
        return sqrt(x * x + y * y);
    }

    // Normalize
    __host__ __device__ Vector2 normalize() const {
        if (magnitude() > 0) {
            return *this / magnitude();
        }
        return *this;
    }
};

#endif // VECTOR_H
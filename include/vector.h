#ifndef VECTOR_H
#define VECTOR_H
#include <cmath>

class Vector2 {
public:
    float x, y;

    // Constructors
    Vector2() : x(0), y(0) {}
    Vector2(float x, float y) : x(x), y(y) {}

    // Operator Overloads
    bool operator==(const Vector2& other) const {
        return x == other.x && y == other.y;
    }

    Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }

    Vector2& operator+=(const Vector2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }

    Vector2& operator-=(const Vector2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vector2 operator*(float scalar) const {
        return Vector2(x * scalar, y * scalar);
    }

    Vector2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vector2 operator/(float scalar) const {
        return Vector2(x / scalar, y / scalar);
    }

    Vector2& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    // Dot product
    float dot(const Vector2& other) const {
        return x * other.x + y * other.y;
    }

    // Cross product
    float cross(const Vector2& other) const {
        return x * other.y - y * other.x;
    }

    // Magnitude
    float magnitude() const {
        return sqrt(x * x + y * y);
    }

    // Normalize
    Vector2 normalize() const {
        if (magnitude() > 0) {
            return *this / magnitude();
        }
        return *this;
    }
};

#endif // VECTOR_H
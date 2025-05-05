#ifndef BOID_H
#define BOID_H

#include "vector.h"
#include <cmath>

class Boid {
public:
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
    SDL_Color color;
    Boid() : position(0, 0), velocity(0, 0), acceleration(0, 0) {}

    Boid(const Vector2& pos, const Vector2& vel)
        : position(pos), velocity(vel), acceleration(0, 0) {}


    bool operator==(const Boid& other) const {
        return (this->velocity == other.velocity && this->acceleration == other.acceleration && this->position == other.position);
    }

    bool operator!=(const Boid& other) const {
        return !(*this == other);
    }

    double distance(const Boid& other) const {
        float x =position.x - other.position.x;
        float y =position.y - other.position.y;
        return std::sqrt((x*x)+(y*y));
    }
};

#endif // BOID_H
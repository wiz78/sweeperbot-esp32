//
// Created by Simone Tellini on 05/04/24.
//

#ifndef FIRMWARE_VECTOR2D_H
#define FIRMWARE_VECTOR2D_H

#include <cmath>

class Vector2d
{
public:
    Vector2d() : _x{0}, _y{0}
    {
    }

    Vector2d( float x, float y ) : _x{x}, _y{y}
    {
    }

    [[nodiscard]] float x() const { return _x; }
    [[nodiscard]] float y() const { return _y; }

    [[nodiscard]] float dot( const Vector2d& other ) const
    {
        return ( _x * other._x ) + ( _y * other._y );
    }

    [[nodiscard]] float project( const Vector2d& other ) const
    {
        return dot( other ) / other.magnitude();
    }

    [[nodiscard]] float magnitude() const { return std::hypot( _x, _y ); }

private:
    float _x;
    float _y;
};

#endif //FIRMWARE_VECTOR2D_H

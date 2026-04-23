#ifndef __UAVWSN_LOCATION_H
#define __UAVWSN_LOCATION_H

#include <cmath>

struct Location {
    double x;
    double y;
    double z;
    
    Location(double _x = 0, double _y = 0, double _z = 0) : x(_x), y(_y), z(_z) {}
    
    double distanceTo(const Location& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    double magnitude() const {
        return sqrt(x*x + y*y + z*z);
    }

    Location operator+(const Location& other) const {
        return Location(x + other.x, y + other.y, z + other.z);
    }

    Location operator-(const Location& other) const {
        return Location(x - other.x, y - other.y, z - other.z);
    }

    Location operator*(double scalar) const {
        return Location(x * scalar, y * scalar, z * scalar);
    }

    Location operator/(double scalar) const {
        if (scalar == 0) {
            return Location(0,0,0);
        }
        return Location(x / scalar, y / scalar, z / scalar);
    }

    Location& operator+=(const Location& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Location& operator-=(const Location& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    friend Location operator*(double scalar, const Location& loc) {
        return Location(loc.x * scalar, loc.y * scalar, loc.z * scalar);
    }
};

#endif

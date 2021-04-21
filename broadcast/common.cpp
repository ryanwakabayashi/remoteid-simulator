#include "common.h"

float Location::longitude_scale() const {
    float scale = cosf(lat * (1.0e-7f * DEG_TO_RAD));
    return MAX(scale, 0.01f);
}

Vector3f Location::get_distance_NED(const Location &loc2) const {
    return Vector3f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    (loc2.lng - lng) * LOCATION_SCALING_FACTOR * longitude_scale(),
                    (alt - loc2.alt) * 0.01f);
}

float Location::get_distance(const Location &loc2) const {
    Vector3f d = get_distance_NED(loc2);
    return sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
}


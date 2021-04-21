#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <math.h>

#define PORT        (7100)
#define BRBUFSIZE   (2048)
#define DEG_TO_RAD  (M_PI / 180.0f)
#define LOCATION_SCALING_FACTOR  (0.011131884502145034f)
#define MAX(x,y)    (x>y ? x : y)
#define RIDLEN      (20)
#define BRRANGE     (500)  // broadcast range in meter

enum MSG_TYPE {
    RSim_Update = 1,       // add or update the vehicle's real location, no payload
    RSim_Broadcast = 2,    // remote id broadcast, the payload is remote id broadcast
};

#define BRBUFSIZE  (2048)
#define BRPLSIZE   (BRBUFSIZE - sizeof(RSimMessage_Header))

typedef struct _message_header {
    uint8_t type;   // message type, MSG_TYPE
    uint8_t id[RIDLEN]; // vehicle id
    uint64_t time;  // message time (assume sending and receiving at the same time, because the transmission time is way smaller than the simulation clock tick, 1/1200=833us.
    int32_t lat;    // real lat
    int32_t lng;    // real lng
    int32_t alt;    // real alt
    uint64_t seq;   // sequence
} __attribute__((packed)) RSimMessage_Header;

typedef struct _message {
    RSimMessage_Header header;
    uint8_t payload[BRPLSIZE];
} __attribute__((packed)) RSimMessage;

class Vector3f {
public:

    float x, y, z;

    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

};

class Location {
public:

    int32_t lat;
    int32_t lng;
    int32_t alt;

    Location() : lat(0), lng(0), alt(0) {};
    Location(int32_t _lat, int32_t _lng, int32_t _alt) : lat(_lat), lng(_lng), alt(_alt) {}

    float longitude_scale() const;
    Vector3f get_distance_NED(const Location &loc2) const;
    float get_distance(const Location &loc2) const;
};

#endif // COMMON_H

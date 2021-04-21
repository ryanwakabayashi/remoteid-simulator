#ifndef CHANNEL_H
#define CHANNEL_H

using namespace std;

#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <vector>

#include "common.h"

class Vehicle {
public:

    Vehicle(const uint8_t* _id, const struct sockaddr_in* _vsck, uint64_t _t, int32_t _lat, int32_t _lng, int32_t _alt) :
        lastUpTime(_t), rl(_lat, _lng, _alt) {
        memcpy(id, _id, 20);
        memcpy(&vsck, _vsck, sizeof(struct sockaddr_in));
    }

    uint8_t id[RIDLEN];  // vehicle id
    struct sockaddr_in vsck;   // vehile's socket to send remote id broadcasts to in the simulator
    uint64_t lastUpTime;   // the last time of updating the vehicle status. should be updated whenever receiving a message from the vehicle

    Location rl;     // real location in simulation, in lat, lng, alt.
    Location gl;     // gps location (the location the vehicle senes from its gps

    bool operator == (const Vehicle& _v);
    bool operator == (const uint8_t* _id);
    void update(const struct sockaddr_in* _vsck, uint64_t _t, int32_t _lat, int32_t _lng, int32_t _alt);

};

class Channel {
public:

    Channel(int _sck) : sck(_sck) {}

    void recvBcst(const uint8_t* msg, ssize_t len, const struct sockaddr_in* s);

protected:

    int sck;
    vector<Vehicle> vs;

    int vehicleExist(const uint8_t* _id);  // check if a vehicle exists in channel, return the vehicle index or -1
    int addOrUpdate(const RSimMessage* m, const struct sockaddr_in* s); // return the index of the vehicle in channel
    void fwdBcst(const RSimMessage* m, ssize_t len);

};

#endif // CHANNEL_H

#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include <assert.h>
#include "channel.h"

void Channel::recvBcst(const uint8_t* msg, ssize_t len, const struct sockaddr_in* s) {
    const RSimMessage* m = (const RSimMessage*)msg;

    switch (m->header.type) {
    case RSim_Update:
        addOrUpdate(m, s);
        break;

    case RSim_Broadcast: {
        fwdBcst(m, len);
        break;
    }

    default:
        ;
    }
}

void Channel::fwdBcst(const RSimMessage* m, ssize_t len) {

    int idx = vehicleExist(m->header.id);
    if (idx < 0) return; // vehicle not found

    int plsize = len - sizeof(RSimMessage_Header);
    if (plsize <= 0) return; // negative or zero payload size

    const Vehicle& v = vs[idx];
    for (size_t i=0; i<vs.size(); i++) {
        if (i == (size_t)idx) continue; // same vehicle, no broadcast forward

        // within broadcast range
        const Vehicle& nv = vs[i];
        if (v.rl.get_distance(nv.rl) < BRRANGE) {
            sendto(sck, m->payload, plsize, MSG_CONFIRM, (const struct sockaddr *)&nv.vsck, sizeof(struct sockaddr_in));
        }
    }

}

int Channel::vehicleExist(const uint8_t* _id) {
    for(size_t i=0; i<vs.size(); i++) {
        if (vs[i] == _id) return i;
    }
    return -1;
}

int Channel::addOrUpdate(const RSimMessage* m, const struct sockaddr_in* s) {
    int vidx = vehicleExist(m->header.id);

    // add a new vehicle in channel
    if (vidx < 0) {
        vs.push_back(Vehicle(m->header.id, s, m->header.time, m->header.lat, m->header.lng, m->header.alt));
        vidx = vs.size()-1;
    } else
        vs[vidx].update(s, m->header.time, m->header.lat, m->header.lng, m->header.alt);

    return vidx;
}

bool Vehicle::operator == (const uint8_t* _id) {
    for (int i=0; i<RIDLEN; i++) {
        if (id[i] != _id[i]) return false;
    }
    return true;
}

bool Vehicle::operator == (const Vehicle& _v) {
    if (this == &_v) return true;
    return (*this) == _v.id;
}

void Vehicle::update(const struct sockaddr_in* _vsck, uint64_t _t, int32_t _lat, int32_t _lng, int32_t _alt) {
    memcpy(&vsck, _vsck, sizeof(sockaddr_in));
    lastUpTime = _t;
    rl.lat = _lat;
    rl.lng = _lng;
    rl.alt = _alt;
}

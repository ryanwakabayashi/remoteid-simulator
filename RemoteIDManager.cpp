#include <stdio.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <string>

#include "RemoteIDManager.h"
#include "Messages/Messages.h"
#include "Messages/Enums.h"
#include "auth/keysys.h"
#include "auth/signer.h"
#include "auth/verifier.h"

void RemoteIDManager::setup() {
    logFile.open ("/var/log/remoteid/-" + std::to_string(std::time(nullptr) - 1546300800) + ".txt");

    gps = AP_GPS::get_singleton();
    baro = AP_Baro::get_singleton();
    
    serial_manager.init();
    gps->init(serial_manager);

    sock.reuseaddress();

    auth = false;  // changed to false to develop processBasicID(m)
    authparams_setup();
    // extract private key
    // method1
    extract_key1(&Us, multicopter->get_remoteid(), RIDLEN);
    // method2
    //extract_key2(&Us, multicopter->get_remoteid(), RIDLEN);
}

/**
 * This is for simulation only. It passes the true vehicle location and time to the broadcast module
 * return the length of msg
 */
int RemoteIDManager::setRSimMsgHeader(RSimMessage& msg, Sim_Msg_Type type) {
    msg.header.type = type;
    memcpy(msg.header.id, multicopter->get_remoteid(), 20);
    msg.header.time = current_time;
    msg.header.lat = current_real_location.lat;
    msg.header.lng = current_real_location.lng;
    msg.header.alt = current_real_location.alt;
    return sizeof(RSimMessage_Header);
}

/**
 * msg header contains info for broadcast simulation only.
 * msg paylaod contains header and body of remote id packet.
 * return the length of msg
 */
int RemoteIDManager::setRSimMsg(RSimMessage& msg, const MessageHeader& header, const MessageBody& body) {
    // set header of msg_out
    int hl = setRSimMsgHeader(msg, RSim_Broadcast);
    msg.header.seq = ++seq;  // only increase seq for broadcast
    // set payload of msg_out
    memcpy(msg.payload, header.data, header.data_len);
    memcpy(msg.payload+header.data_len, body.data, body.data_len);
    return hl + header.data_len + body.data_len;
}

// should only called from update()
bool RemoteIDManager::initBroadcast() {
    // already broading, no need to re-init
    if (broadcasting) return true;

    msg_out_len = setRSimMsgHeader(msg_out, RSim_Update);
    sock.sendto(&msg_out, msg_out_len, BRCHANNEL, multicopter->get_broadcast_port());
    printf("init %lu\n", current_time);

    broadcasting = true;
    nextStaticUpdate = current_time;
    nextDynamicUpdate = current_time;

    return true;
}

void RemoteIDManager::update() {
    // update private status variables
    current_time = multicopter->get_time_now_us();
    current_real_location = multicopter->get_location();

    if (!broadcasting) initBroadcast();

    gps->update();

    if (broadcasting) {
        // update the channel of the position
        msg_out_len = setRSimMsgHeader(msg_out, RSim_Update);
        sock.sendto(&msg_out, msg_out_len, BRCHANNEL, multicopter->get_broadcast_port());
        //printf("updt %lu\n", current_time);

        // first process all received messages
        ssize_t n;
        while ( (n = sock.recv(msg_in, BRBUFSIZE-1, 1)) >= 0) {
            msg_in[n] = '\0';
            recvBroadcast(msg_in, n);
            printf("recv %lu : %s\n", current_time, msg_in);
        }
        
        /**
         * A Standard Remote ID Drone, remote ID capability is built into the drone, must broadcas:
         * A unique identifier for the drone: drone's serial number or a session ID
         * An indication of the drone's latitude, longitude, geometric altitude, and velocity;
         * An indication of the control station's latitude, longitude, and geometric altitude;
         * A time mark
         * An emergency status indication.
         * 
         * A drone with a remote ID broadcast module, remote ID capability through module attached to drone, must broadcast
         * The serial number of the broadcast module;
         * An indication of the drone's latitude, longitude, geometric altitude, and velocity;
         * An indication of the latitude, longitude, and geometric altitude of the drone's take-off location;
         * A time mark.
         * 
         * BCMinUasLocRefreshRate 1 Seconds BUR0010 5.4.4
         * BCMinStaticRefreshRate 3 Seconds BUR0020 5.4.4
         */
        // then, broadcast remote id on a schedule
        if (current_time >= nextStaticUpdate) {
            broadcast(RID_BasicID);
            printf("brct basic %lu\n", current_time);

            nextStaticUpdate += updateStaticInterval;
            if(savedDrones.empty()){printf("drone list is empty**********************************************************\n");}// to show when savedDrone map is empty

        }

        if (current_time >= nextDynamicUpdate) {
            broadcast(RID_Location);

            nextDynamicUpdate += updateDynamicInterval;
        }

        for( auto it = savedDrones.begin(); it != savedDrones.end(); ){
            if (current_time > it->second.timestamp + updateStaticInterval * 2){ // time needs to be adjusted
                it =  savedDrones.erase(it);
            }
            else{
                ++it;
            }
        }
    }

    return;

}

void RemoteIDManager::recvBroadcast(uint8_t* m, size_t n) {
    MessageHeader h(m);
    const uint8_t* d = m + h.data_len;
    uint8_t testid[20] = {0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x77, 0x78, 0x79, 0x7A};
    uint8_t *value = multicopter->get_remoteid();
    switch (h.header.MessageType) {
        case RID_BasicID : {
            // printf("////////////////////remote id value: %x\n", value[19]);  // used to make sure the value was correct for the following if statement
            if (value[19] == testid[19]){
                LOG("recv,basicid," + std::to_string(current_time));
                processBasicID(d);  // TODO // either d or m
                printf("******************* MY Remote ID IS NEXT\n"); // easy flag to see if the if statment worked! not needed now
                for(int i = 0; i < 20; i++){
                    printf("%x", *(value + i)); // shows that get remote id is array of hex
                }
                printf("\n");
                for(auto x : savedDrones){
                    // for(int i = 0; i < 20; i++){std::cout << x.first[i];}
                    std::cout << x.second.basicIDMessage.uasId << "__Saved drone remote ID in chars\n" << std::to_string(x.second.timestamp);
                }
                printf("___saved drone time entered\n");
            }
            break;
        }
        case RID_Location :
        LOG("recv,location," + std::to_string(current_time));
        // processLocationVector(m); // TODO
        break;

        case RID_Auth :
        break;

        case RID_SelfID :
        break;

        case RID_System :
        break;

        case RID_OperatorID :
        break;

        case RID_MsgPack :
        LOG("recv,msgpack," + std::to_string(current_time));
        processMessagePack(d);
        break;

        default:
        ;
    }
}

void RemoteIDManager::processBasicID(const uint8_t* d){
    BasicIDMessage b(d);

    savedDrone t1;
    t1.basicIDMessage = b.basicIDMessage;
    t1.timestamp = current_time = multicopter->get_time_now_us();

    // std::array<uint8_t, 20> id;
    // for(int i = 0; i < 20; i++){ id[i] = b.basicIDMessage.uasId[i]; }
    // savedDrones[id] = 25465;

    std::array<uint8_t, 20> id;
    for(int i = 0; i < 20; i++){ id[i] = b.basicIDMessage.uasId[i]; }
    savedDrones[id] = t1;

    // first variable in map will be uint8_t* remoteID 
    // make object using d, then pass object as second variable in map
    // try to delete after a minute

}

void RemoteIDManager::broadcast(RID_Msg_Type rmt) {

    bool ready = true;

    switch (rmt) {
        case RID_BasicID : {
            if (auth) msg_out_len = makeAuthBasicID(msg_out);
            else msg_out_len = makeBasicID(msg_out);
            break;
        }

        case RID_Location : {
            if (auth) msg_out_len = makeAuthLocationVector(msg_out);
            else msg_out_len = makeLocationVector(msg_out);
            break;
        }

        /*case RID_SelfID : {
            uint8_t descriptionType = 0;
            char description[23] = "DronesRus:Survey";
            MessageHeader selfIDMessageHeader(RID_SelfID);
            SelfIDMessage selfIDMessage(descriptionType, description);
            setRSimMsg(selfIDMessageHeader, selfIDMessage);
            break;
        }*/

        /*case RID_System : {
            uint8_t systemFlags = 0;
            int32_t operatorLatitude = multicopter->get_home().lat;
            int32_t operatorLongitude = multicopter->get_home().lng;
            uint16_t areaCount = 1;
            uint8_t areaRadius = 0;
            uint16_t AreaCeiling = 0;
            uint16_t AreaFloor = 0;
            MessageHeader systemMessageHeader(RID_System);
            SystemMessage systemMessage(            
                systemFlags,
                operatorLatitude,
                operatorLongitude,
                areaCount,
                areaRadius,
                AreaCeiling,
                AreaFloor
            );
            setRSimMsg(systemMessageHeader, systemMessage);
            break;
        }*/

        /*case RID_OperatorID : {
            uint8_t operatorIdType = 0;
            char operatorId[20] = "Damen Hannah #123";
            MessageHeader operatorIDMessageHeader(RID_OperatorID);
            OperatorIDMessage operatorIDMessage(operatorIdType, operatorId);
            setRSimMsg(operatorIDMessageHeader, operatorIDMessage);
            break;
        }*/
        
        default :
            ready = false;
    }

    // no packet to send
    if (!ready) return;

    sock.sendto(&msg_out, msg_out_len, BRCHANNEL, multicopter->get_broadcast_port());

    //logFile << messagePack->toJson().dump(4) << std::endl;
    
}

void RemoteIDManager::processMessagePack(const uint8_t* data) {
    MessagePack p(data);

    int i;

    // auth must be the first packet in message pack.
    MessageHeader f(p.pack.messages[0]);

    if (f.header.MessageType == RID_Auth) {
        // process authentication
        // get the signature
        fp_t sig;
        uint8_t* sigptr = (uint8_t*)&sig;
        // page 0
        MessageHeader h_a0(p.pack.messages[0]);
        AuthenticationMessage a0(p.pack.messages[0]+1);
        // validate page 0
        assert(a0.authentication.authType == RID_Auth_Type_UasID || a0.authentication.authType == RID_Auth_Type_Message);
        assert(a0.authentication.pageCount == 2);
        assert(a0.authentication.pageNumber == 0);
        assert(a0.authentication.length == 32);
        // page 1
        MessageHeader h_a1(p.pack.messages[1]);
        AuthenticationMessagePage a1(p.pack.messages[1]+1);
        // validate page 1
        assert(a1.authenticationPage.authType == a0.authentication.authType);
        assert(a1.authenticationPage.pageNumber == 1);

        // TODO: a0.authentication.timestamp

        // copy the signature from page 0 and page 1
        memcpy(sigptr, a0.authentication.authenticationData, 17);
        memcpy(sigptr+17, a1.authenticationPage.authenticationData, 15);

        // get id from message[2]
        BasicIDMessage m(p.pack.messages[2]+1);
        uint8_t rid[RIDLEN];
        memcpy(rid, m.basicIDMessage.uasId, RIDLEN);

        // from message[2], all data, but need to be concatenated

        uint8_t* d = p.pack.messages[2];
        uint8_t l = (p.pack.numberMessages - 2) * 25;
        
        // verify
        //int v;
        // method 1
        verify1(sig, rid, RIDLEN, d, l);
        // method 2
        //v = verify2(sig, rid, RIDLEN, d, l);

    } else {
        // not auth messages
        for (i=0; i<p.pack.numberMessages; i++) {
            MessageHeader h(p.pack.messages[i]);
            if (h.header.MessageType == RID_BasicID) {
                BasicIDMessage m(p.pack.messages[i]+1);
                // processBasicID(m);  // TODO
            } else if (h.header.MessageType == RID_Location) {
                LocationVectorMessage m(p.pack.messages[i]+1);
                // processLocationVector(m); // TODO
            }
        }
    }

    return;
}


int RemoteIDManager::makeBasicID(RSimMessage& msg) {
    uint8_t idType = RID_ID_Type_Serial;
    uint8_t uaType = RID_UA_Type_Helicopter;
    MessageHeader basicMessageHeader(RID_BasicID);
    BasicIDMessage basicIdMessage(idType, uaType, multicopter->get_remoteid());
    return setRSimMsg(msg, basicMessageHeader, basicIdMessage);
}

int RemoteIDManager::makeLocationVector(RSimMessage& msg) {
    uint8_t status = RID_OP_Status_Airborne;
    uint8_t flag_height_type = RID_Height_Type_AGL;
    uint8_t flag_ew_direction;
    uint8_t trackDirection;
    encodeDirection(gps->ground_course(), flag_ew_direction, trackDirection);
    uint8_t flag_speed_multiplier = RID_Speed_Multiplier_1Q;
    uint8_t speed;
    encodeSpeed(gps->ground_speed(), flag_speed_multiplier, speed);
    int8_t verticalSpeed;
    encodeVerticalSpeed(gps->velocity().z, verticalSpeed);
    int32_t latitude = gps->location().lat;
    int32_t longitude = gps->location().lng;
    uint16_t pressureAltitude;
    encodePressureAltitude(baro->get_altitude(), pressureAltitude);
    uint16_t geodeticAltitude;
    encodePressureAltitude(gps->location().alt/100, geodeticAltitude);
    uint16_t height = geodeticAltitude;
    uint8_t verticalAccuracy;
    float accuracy;
    if (gps->vertical_accuracy(accuracy)) encodeVerticalAccuracy(accuracy, verticalAccuracy);
    else verticalAccuracy = RID_Vertical_Accuracy_Unknown;
    uint8_t horizontalAccuracy;
    if (gps->horizontal_accuracy(accuracy)) encodeHorizontalAccuracy(accuracy, horizontalAccuracy);
    else horizontalAccuracy = RID_Horizontal_Accuracy_Unknown;
    uint8_t baroAltitudeAccuracy = RID_Vertical_Accuracy_3m;
    uint8_t speedAccuracy;
    if (gps->speed_accuracy(accuracy)) ;
    else speedAccuracy = RID_Speed_Accuracy_Unknown;
    uint16_t timestamp = (current_time/100000)%36000; // current_time in us, timestamp in 0.1s
    uint8_t timestampAccuracy = 1;
    MessageHeader locationVectorMessageHeader(RID_Location);
    LocationVectorMessage locationVectorMessage(
        status,
        flag_height_type,
        flag_ew_direction,
        flag_speed_multiplier,
        trackDirection,
        speed,
        verticalSpeed,
        latitude,
        longitude,
        pressureAltitude,
        geodeticAltitude,
        height,
        verticalAccuracy,
        horizontalAccuracy,
        baroAltitudeAccuracy,
        speedAccuracy,
        timestamp,
        timestampAccuracy
    );
    return setRSimMsg(msg, locationVectorMessageHeader, locationVectorMessage);
}

int RemoteIDManager::makeAuthBasicID(RSimMessage& msg) {
    uint8_t data[1][25];
    RSimMessage _m;
    makeBasicID(_m);
    memcpy(data[0], _m.payload, 25);
    return makeAuth(msg, RID_Auth_Type_UasID, data, 1);
}

int RemoteIDManager::makeAuthLocationVector(RSimMessage& msg) {
    uint8_t data[2][25];
    RSimMessage _m;
    makeBasicID(_m);
    memcpy(data[0], _m.payload, 25);
    makeLocationVector(_m);
    memcpy(data[1], _m.payload, 25);
    return makeAuth(msg, RID_Auth_Type_Message, data, 2);
} 

/**
 * Auth packet is carried in MessagePack.
 * The first two messages are auth page 0 and page 1, including the signature.
 * The rest of messages are the data.
 */
int RemoteIDManager::makeAuth(RSimMessage& msg, RID_Auth_Type type, uint8_t data[][25], uint8_t count) {
    MessageHeader authPackHeader(RID_MsgPack);
    MessagePack authPack;

    // sign
    fp_t sig;
    // method 1
    sign1(sig, (uint8_t*)&(data[0]), 25*count);
    // method 2
    //sign2(sig, m.payload, 25);

    // auth page 0
    uint8_t authType = type;
    uint8_t pageNumber = 0;
    uint8_t pageCount = 2;
    uint8_t length = 32;
    uint32_t timestamp = current_time;  // TODO, to fix according to spec
    uint8_t* signature = (uint8_t*)sig;
    MessageHeader authenticationMessageHeader(RID_Auth);
    AuthenticationMessage authenticationMessage(
        authType, 
        pageNumber,
        pageCount,
        length,
        timestamp,
        17,
        signature
    );
    RSimMessage _m;
    setRSimMsg(_m, authenticationMessageHeader, authenticationMessage);
    authPack.addMessage(_m.payload);

    // auth page 1
    pageNumber = 1;
    AuthenticationMessagePage authenticationMessagePage(
        authType, 
        pageNumber,
        sizeof(fp_t)-17,
        signature+17
    );
    setRSimMsg(_m, authenticationMessageHeader, authenticationMessagePage);
    authPack.addMessage(_m.payload);

    // add data
    for (int i=0; i<count; i++) authPack.addMessage(data[i]);

    return setRSimMsg(msg, authPackHeader, authPack);
}

// Direction expressed as the route course measured clockwise from true north. Encode as 0–179.
// If E/W Direction bit is set, then 180 should be added to the value.
void RemoteIDManager::encodeDirection(float value, uint8_t& flag_ew_direction, uint8_t& trackDirection) {
    int16_t direction = (int16_t)round(value);
    direction = direction % 360;
    if (0 <= direction && direction < 180) {
        flag_ew_direction = RID_EW_Direction_East;
        trackDirection = direction;
    } else {
        flag_ew_direction = RID_EW_Direction_West;
        trackDirection = direction-180;
    }
}

// If value <= 225*0.25, encodedValue = value/0.25, set multiplier flag to 0
// Else if value > 225*0.25 and value <254.25, encodedValue = (value – (225*0.25))/0.75, set multiplier flag to 1
// Else value >= 254.25 m/s, encodedValue = 254, set multiplier flag to 1
void RemoteIDManager::encodeSpeed(float value, uint8_t& flag_speed_multiplier, uint8_t& speed) {
    if (value < 0) value = 0;
    if (value <= 225*0.25) {
        flag_speed_multiplier = RID_Speed_Multiplier_1Q;
        speed = (uint8_t)value*4;
    } else if (value < 254.25) {
        flag_speed_multiplier = RID_Speed_Multiplier_3Q;
        speed = (uint8_t)((value - 225*0.25) / 0.75);
    } else {
        flag_speed_multiplier = RID_Speed_Multiplier_3Q;
        speed = 254;
    }
}

// EncodedValue = Value(m/s) / 0.5, up to ±63.5 m/s (12.5k ft/min)
void RemoteIDManager::encodeVerticalSpeed(float value, int8_t& verticalSpeed) {
    if (value > 63.5) value = 63.5;
    if (value < -63.5) value = -63.5;
    verticalSpeed = (int8_t)(value * 2);
}             

// EncodedValue = (Value(m) + 1000) / 0.5
void RemoteIDManager::encodePressureAltitude(float value, uint16_t& pressureAltitude) {
    if (value<-1000) value = -1000;
    if (value>31767.5) value = 31767.5;
    pressureAltitude = (uint16_t)((value + 1000) * 2);
}

// enum RID_Vertical_Accuracy
void RemoteIDManager::encodeVerticalAccuracy(float value, uint8_t& verticalAccuracy) {
    if (value < 0) verticalAccuracy = RID_Vertical_Accuracy_Unknown;
    else if (value < 1) verticalAccuracy = RID_Vertical_Accuracy_1m;
    else if (value < 3) verticalAccuracy = RID_Vertical_Accuracy_3m;
    else if (value < 10) verticalAccuracy = RID_Vertical_Accuracy_10m;
    else if (value < 25) verticalAccuracy = RID_Vertical_Accuracy_25m;
    else if (value < 45) verticalAccuracy = RID_Vertical_Accuracy_45m;
    else if (value < 150) verticalAccuracy = RID_Vertical_Accuracy_150m;
    else verticalAccuracy = RID_Vertical_Accuracy_Unknown;
}

// enum RID_Horizontal_Accuracy
void RemoteIDManager::encodeHorizontalAccuracy(float value, uint8_t& horizontalAccuracy) {
    if (value < 0) horizontalAccuracy = RID_Horizontal_Accuracy_Unknown;
    else if (value < 1) horizontalAccuracy = RID_Horizontal_Accuracy_1m;
    else if (value < 3) horizontalAccuracy = RID_Horizontal_Accuracy_3m;
    else if (value < 10) horizontalAccuracy = RID_Horizontal_Accuracy_10m;
    else if (value < 30) horizontalAccuracy = RID_Horizontal_Accuracy_30m;
    else if (value < 92.6) horizontalAccuracy = RID_Horizontal_Accuracy_005NM;
    else if (value < 185.2) horizontalAccuracy = RID_Horizontal_Accuracy_01NM;
    else if (value < 555.6) horizontalAccuracy = RID_Horizontal_Accuracy_03NM;
    else if (value < 926) horizontalAccuracy = RID_Horizontal_Accuracy_05NM;
    else if (value < 1852) horizontalAccuracy = RID_Horizontal_Accuracy_1NM;
    else if (value < 3704) horizontalAccuracy = RID_Horizontal_Accuracy_2NM;
    else if (value < 7408) horizontalAccuracy = RID_Horizontal_Accuracy_4NM;
    else if (value < 18520) horizontalAccuracy = RID_Horizontal_Accuracy_10NM;
    else horizontalAccuracy = RID_Horizontal_Accuracy_Unknown;
}

// enum RID_Speed_Accuracy
void RemoteIDManager::encodeSpeedAccuracy(float value, uint8_t& speedAccuracy) {
    if (value < 0) speedAccuracy = RID_Speed_Accuracy_Unknown;
    else if (value < 0.3) speedAccuracy = RID_Speed_Accuracy_03m;
    else if (value < 1) speedAccuracy = RID_Speed_Accuracy_1m;
    else if (value < 3) speedAccuracy = RID_Speed_Accuracy_3m;
    else if (value < 10) speedAccuracy = RID_Speed_Accuracy_10m;
    else speedAccuracy = RID_Speed_Accuracy_Unknown;
}
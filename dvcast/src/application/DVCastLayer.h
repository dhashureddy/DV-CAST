/*
 MIT License

 Copyright (c) 2017 Valentine Nwachukwu <valdiz777@gmail.com>

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#ifndef DVCastLayer_H
#define DVCastLayer_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "messages/DVCast_m.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;

/**
 * DV-Cast VANET using 11p
 */
class DVCastLayer: public BaseWaveApplLayer {
public:
    virtual void initialize(int stage);
    virtual void receiveSignal(cComponent* source, simsignal_t signalID,
            cObject* obj, cObject* details);
protected:
    TraCIMobility* mobility;
    TraCICommandInterface* traci;
    TraCICommandInterface::Vehicle* traciVehicle;
    AnnotationManager* annotations;
    simtime_t lastDroveAt;

    std::deque<int> NB_FRONT, NB_BACK, NB_OPPOSITE;
    std::deque<int> rcvdMessages;
    std::deque<int> sentMessages;
    std::map<int, std::string> delayedRB;
    bool sentAccidentMessage;
    bool isParking;
    bool sendWhileParking;
    bool ODC =true;
    bool MDC = false;
    bool Dflg = false;
    static const simsignalwrap_t parkingStateChangedSignal;
protected:
    virtual void onBeacon(WaveShortMessage* wsm);
    virtual void onData(WaveShortMessage* wsm);
    virtual void handlePositionUpdate(cObject* obj);
    virtual void handleParkingUpdate(cObject* obj);
    virtual void handleLowerMsg(cMessage* msg);
    virtual void sendWSM(WaveShortMessage* wsm);

    void onHello(DVCast* wsm);
    void neigbors_tables(Coord senderPosition, int senderId, int senderAngle);
    void sendHello(DVCast* wsm);
    void sendMessage(std::string blockedRoadId, int recipient, int serial);
    DVCast* prepareHello(std::string name, int lengthBits, t_channel channel,
            int priority, int rcvId, int serial);
public:

};

template<class InputIterator, class T>
static InputIterator find(InputIterator first, InputIterator last,
        const T& val) {
    while (first != last) {
        if (*first == val)
            return first;
        ++first;
    }
    return last;
}

template<class ForwardIt, class T>
ForwardIt remove(ForwardIt first, ForwardIt last, const T& value) {
    first = find(first, last, value);
    if (first != last)
        for (ForwardIt i = first; ++i != last;)
            if (!(*i == value))
                *first++ = std::move(*i);
    return first;
}
#endif

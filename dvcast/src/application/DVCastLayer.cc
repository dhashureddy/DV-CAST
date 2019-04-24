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

#include "DVCastLayer.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t DVCastLayer::parkingStateChangedSignal = simsignalwrap_t(
TRACI_SIGNAL_PARKING_CHANGE_NAME);

// Utility functions
void add_to_queue(std::deque<int> * TargetQueue, std::deque<int> * OtherQueue1,
        std::deque<int> * OtherQueue2, int key);
int to_positive_angle(double angle);
bool contains(std::deque<int> * queue, int key);
bool contains(std::map<int, std::string> delayedRB, int key);
void remove(std::deque<int> * queue, int key);

// ROI = 2 * clusterRadius
int clusterRadius = 2500;

Define_Module(DVCastLayer);
void DVCastLayer::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        EV << "****initialize" << endl;
        mobility = TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        sentAccidentMessage = false;
        lastDroveAt = simTime();
        findHost()->subscribe(parkingStateChangedSignal, this);
        isParking = false;
        sendWhileParking = par("sendWhileParking").boolValue();

        EV << "Car [" << getParentModule()->getIndex()
                  << "] Entering CPE646 DVCast Simulation " << endl;
    }
}

void DVCastLayer::onBeacon(WaveShortMessage* wsm) {
}

// route messages manually, bypass wsm routing to on data automatically
void DVCastLayer::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    EV << "****handleLowerMsg" << endl;
    if (std::string(wsm->getName()) == "data") {
        ASSERT(wsm);
        onData(wsm);
    } else if (std::string(wsm->getName()) == "hello") {
        DVCast* dv_wsm = dynamic_cast<DVCast*>(wsm);
        ASSERT(dv_wsm);
        onHello(dv_wsm);
    } else {
        DBG << "unknown message (" << wsm->getName() << ")  received\n";
    }
    delete (msg);
}

// received accident data, process dv-cast
void DVCastLayer::onData(WaveShortMessage* wsm) {
    EV << "****onData" << endl;

    findHost()->getDisplayString().updateWith("r=16,green");
    // ignore messages that I sent out
    if (contains(&sentMessages, wsm->getSerial())) {
        return;
    }

    if (!NB_OPPOSITE.empty()) {
        if (NB_OPPOSITE.size() == 1
                && (NB_OPPOSITE.front() == wsm->getSenderAddress())) {
            ODC = false;
        }
        ODC = true;
    } else {
        ODC = false;
    }

    Dflg = (wsm->getRecipientAddress() == getParentModule()->getIndex()) ?
            true : false;
    MDC = (NB_FRONT.empty() || NB_BACK.empty()) ? false : true;

    EV << "MDC:" << MDC << " ODC:" << ODC << " Dflg:" << Dflg << endl;

    if (!MDC) {
        // no broadcast suppression yet
        if (ODC) {
            sendMessage(wsm->getWsmData(), -1, wsm->getSerial());
            if (!Dflg) {
                findHost()->getDisplayString().updateWith("r=16,pink");
                if (!contains(delayedRB, wsm->getSerial())) {
                    delayedRB.insert(
                            std::pair<int, std::string>(wsm->getSerial(),
                                    wsm->getWsmData()));
                }
            }
        } else {
            findHost()->getDisplayString().updateWith("r=16,blue");
            delayedRB.insert(
                    std::pair<int, std::string>(wsm->getSerial(),
                            wsm->getWsmData()));
        }
    }
}

void DVCastLayer::sendMessage(std::string blockedRoadId, int recipient,
        int serial) {
    EV << "****sendMessage" << endl;
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel,
            dataPriority, recipient, serial);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
    sentMessages.push_back(wsm->getSerial());
}

void DVCastLayer::receiveSignal(cComponent* source, simsignal_t signalID,
        cObject* obj, cObject* details) {
    EV << "****receiveSignal" << endl;
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    } else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void DVCastLayer::handleParkingUpdate(cObject* obj) {
    EV << "****handleParkingUpdate" << endl;
    isParking = mobility->getParkingState();
    if (sendWhileParking == false) {
        if (isParking == true) {
            (FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(
                    this->getParentModule()->getSubmodule("nic"));
        } else {
            Coord pos = mobility->getCurrentPosition();
            (FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(
                    this->getParentModule()->getSubmodule("nic"),
                    (ChannelAccess*) this->getParentModule()->getSubmodule(
                            "nic")->getSubmodule("phy80211p"), &pos);
        }
    }
}

// send hellos evertime we move 50 meters in x or y
void DVCastLayer::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);

    // Send hello in 1Hz freqency
    if (simTime() - lastDroveAt > 1) {
        DVCast* wsm = prepareHello("hello", beaconLengthBits, type_CCH,
                beaconPriority, -1, 72);
        sendHello(wsm);
    }

    if (mobility->getSpeed() < 1) {
        // stopped for for at least 10s?
        if (simTime() - lastDroveAt >= 10) {
            findHost()->getDisplayString().updateWith("r=16,red");
            if (!sentAccidentMessage) {
                int serial = rand() % 101;
                sendMessage(mobility->getRoadId(), -1, serial);
                sentMessages.push_back(serial);
                sentAccidentMessage = true;
            }
        }
    } else {
        lastDroveAt = simTime();
    }
}

void DVCastLayer::sendWSM(WaveShortMessage* wsm) {
    if (isParking && !sendWhileParking)
        return;
    sendDelayedDown(wsm, individualOffset);
}

void DVCastLayer::sendHello(DVCast* wsm) {

    if (isParking && !sendWhileParking)
        return;
    EV << "****send_WSM" << endl;
    sendDelayedDown(wsm, individualOffset);
}

// Received periodic hello from possible neighbors, update neighborhood table
void DVCastLayer::onHello(DVCast* wsm) {
    if (MDC) {
        findHost()->getDisplayString().updateWith("r=16,yellow");
    } else if (ODC) {
        findHost()->getDisplayString().updateWith("r=16,purple");
    }

    EV << "****onHello" << endl;
    EV << "My Position x:" << mobility->getCurrentPosition().x
              << " My Position y:" << mobility->getCurrentPosition().y
              << " My Position z:" << mobility->getCurrentPosition().z
              << " My Id: " << getParentModule()->getIndex() << " angle: "
              << to_positive_angle(mobility->getAngleRad()) << endl;
    EV << "Sender Position x:" << wsm->getSenderPos().x << " Sender Position y:"
              << wsm->getSenderPos().y << " Sender Position z:"
              << wsm->getSenderPos().z << " Sender id: "
              << wsm->getSenderAddress() << " angle: "
              << to_positive_angle(wsm->getAngle()) << endl;
    if (mobility->getCurrentPosition().x < wsm->getRoi_up().x
            && mobility->getCurrentPosition().y < wsm->getRoi_up().y
            && mobility->getCurrentPosition().x > wsm->getRoi_down().x
            && mobility->getCurrentPosition().y > wsm->getRoi_down().y) {
        neigbors_tables(wsm->getSenderPos(), wsm->getSenderAddress(),
                to_positive_angle(wsm->getAngle()));
    }
}

//Main algorithm used to decide MDC, OPC, and Dflg
void DVCastLayer::neigbors_tables(Coord senderPosition, int senderId,
        int senderAngle) {
    EV << "**********neigbors_tables Message has arrived MyId:"
              << getParentModule()->getIndex() << " SenderId:" << senderId
              << endl;

    int myAngle = to_positive_angle(mobility->getAngleRad());
    int angleDiff = std::abs(senderAngle - myAngle);
    if (angleDiff > 180)
        angleDiff = 360 - angleDiff;
    EV << "My angle is:" << myAngle << " Neighbor's Angle is:" << senderAngle
              << endl;
    if (angleDiff <= 45) {
        // same direction, calculate front and back
        if ((senderAngle >= 0 && senderAngle < 45)
                || (senderAngle >= 315 && senderAngle < 360)) {
            //east
            if (senderPosition.x > mobility->getCurrentPosition().x) {
                add_to_queue(&NB_FRONT, &NB_BACK, &NB_OPPOSITE, senderId);
            } else {
                add_to_queue(&NB_BACK, &NB_FRONT, &NB_OPPOSITE, senderId);
            }
        } else if ((senderAngle >= 45 && senderAngle < 90)
                || (senderAngle >= 90 && senderAngle < 135)) {
            //north
            if (senderPosition.y > mobility->getCurrentPosition().y) {
                add_to_queue(&NB_FRONT, &NB_BACK, &NB_OPPOSITE, senderId);
            } else {
                add_to_queue(&NB_BACK, &NB_FRONT, &NB_OPPOSITE, senderId);
            }
        } else if ((senderAngle >= 135 && senderAngle < 180)
                || (senderAngle >= 180 && senderAngle < 225)) {
            // west
            if (senderPosition.x < mobility->getCurrentPosition().x) {
                add_to_queue(&NB_FRONT, &NB_BACK, &NB_OPPOSITE, senderId);
            } else {
                add_to_queue(&NB_BACK, &NB_FRONT, &NB_OPPOSITE, senderId);
            }
        } else if ((senderAngle >= 225 && senderAngle < 270)
                || (senderAngle >= 270 && senderAngle < 315)) {
            // south
            if (senderPosition.y < mobility->getCurrentPosition().y) {
                add_to_queue(&NB_FRONT, &NB_BACK, &NB_OPPOSITE, senderId);
            } else {
                add_to_queue(&NB_BACK, &NB_FRONT, &NB_OPPOSITE, senderId);
            }
        }

    } else if (angleDiff > 90 && angleDiff <= 180) {
        // opposite direction
        NB_OPPOSITE.push_back(senderId);
        add_to_queue(&NB_OPPOSITE, &NB_FRONT, &NB_BACK, senderId);
    }

    MDC = (NB_FRONT.empty() || NB_BACK.empty()) ? false : true;

    if (!NB_OPPOSITE.empty()) {
        ODC = true;
    } else {
        ODC = false;
    }

    EV << "NB_FRONT [ ";
    for (std::deque<int>::const_iterator i = NB_FRONT.begin();
            i != NB_FRONT.end(); ++i)
        EV << *i << ' ';
    EV << "]" << endl << " NB_BACK [ ";
    for (std::deque<int>::const_iterator i = NB_BACK.begin();
            i != NB_BACK.end(); ++i)
        EV << *i << ' ';
    EV << "]" << endl << " NB_OPPOSITE [ ";
    for (std::deque<int>::const_iterator i = NB_OPPOSITE.begin();
            i != NB_OPPOSITE.end(); ++i)
        EV << *i << ' ';
    EV << "]" << endl;

    if (!delayedRB.empty() && !ODC) {
        for (auto const& x : delayedRB) {
            if (!MDC) { //verify MDC is still false
                sendMessage(x.second, -1, x.first);
            }

        }
        while (!delayedRB.empty())
          {
            delayedRB.erase(delayedRB.begin());
          }
        ODC = true;
    }
}

DVCast* DVCastLayer::prepareHello(std::string name, int lengthBits,
        t_channel channel, int priority, int rcvId, int serial) {
    EV << "****preparemyWSM" << endl;
    DVCast* wsm = new DVCast(name.c_str(), 0);
    wsm->addBitLength(headerLength);
    wsm->addBitLength(lengthBits);

    switch (channel) {
    case type_SCH:
        wsm->setChannelNumber(Channels::SCH1);
        break; //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
    case type_CCH:
        wsm->setChannelNumber(Channels::CCH);
        break;
    }
    wsm->setPsid(0);
    wsm->setPriority(priority);
    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(getParentModule()->getIndex());
    wsm->setSenderPos(curPosition);

    wsm->setSerial(serial);

    if (name == "beacon") {
        DBG << "Creating Beacon with Priority " << priority
                   << " at Applayer at " << wsm->getTimestamp() << std::endl;
    }
    if (name == "data") {
        DBG << "Creating Data with Priority " << priority << " at Applayer at "
                   << wsm->getTimestamp() << std::endl;
    }

    if (name == "hello") {
        DBG << "Creating Hello with Priority " << priority << " at Applayer at "
                   << wsm->getTimestamp() << std::endl;
        wsm->setRoi_up(
                Coord(curPosition.x + clusterRadius,
                        curPosition.y + clusterRadius));
        wsm->setRoi_down(
                Coord(curPosition.x - clusterRadius,
                        curPosition.y - clusterRadius));
        wsm->setKind(3);
        wsm->setAngle(mobility->getAngleRad());
    }

    wsm->setId(getParentModule()->getIndex());
    return wsm;
}

int to_positive_angle(double angle) {
    angle = (180 / 3.14) * angle;
    angle = fmod(angle, 360);
    if (angle < 0)
        angle += 360;
    return (int) angle;
}

void add_to_queue(std::deque<int> * TargetQueue, std::deque<int> * OtherQueue1,
        std::deque<int> * OtherQueue2, int key) {

    // Remove redundant nodes
    remove(TargetQueue, key);
    remove(OtherQueue1, key);
    remove(OtherQueue2, key);

    // Add our new neighbor
    TargetQueue->push_back(key);

    // For ensure MAXnb = 5
    while (TargetQueue->size() > 5) {
        TargetQueue->pop_front();
    }

}

void remove(std::deque<int> * queue, int key) {
    queue->erase(remove(queue->begin(), queue->end(), key), queue->end());
}

bool contains(std::deque<int> * queue, int key) {
    return (find(queue->begin(), queue->end(), key) != queue->end()) ?
            true : false;
}

bool contains(std::map<int, std::string> delayedRB, int key) {
    for (auto const& x : delayedRB) {
        if (x.first == key)
            return true;
    }
    return false;
}

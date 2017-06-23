//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __AID_AID_H_
#define __AID_AID_H_

#include "BaseWaveApplLayer.h"
#include "WaveShortMessage_m.h"
#include "DataMessage_m.h"
#include "TraCIMobility.h"
#include <algorithm>

#include "FindModule.h"
#include "Mac1609_4.h"
//TODO: Added for Game theory Solution
#include "DeciderResult80211.h"
#include "PhyToMacControlInfo.h"
#include "PhyControlMessage_m.h"

/**
 * TODO - Generated class
 */
class AID : public BaseWaveApplLayer {

private:
    struct NeighborEntry {
        int senderAddress;
        Coord position; // Neighbor position

        cMessage* beaconHoldTimer;
    };

    struct MessageInfoEntry {
        int messageID; // Message unique ID

        Coord messageOriginPosition; // Message origin
        double messageROI; // Region of Interest in meters
        simtime_t messageOriginTime; // Time of message inception at source host
        simtime_t messageTTL; // Message's time-to-live
        int hops; // Message's number of hops

        simtime_t receptionTime;
        int messageLength;
        double distanceToOrigin;

        int c;
        simtime_t lastReceptionTime;
        std::vector<simtime_t> deltaTimes;

        cMessage* broadcastTimer; // Self-message for broadcast suppression
    };

    enum AIDMessageKinds {
        BROADCAST_TIMEOUT = SEND_BEACON_EVT + 1, // Timer used to control the wait time for vehicles. When it fires, then send message
        BACK_TRAFFIC_ENTRY_TIMEOUT,
        CCH_START,
        SCH_START,
    };

protected:
    //TODO: Added for Game theory Solution
    double curTxPower;
    int powerLevel;
    double myUtilityValue;

    bool wasInROI;

    bool disseminationStarted;

    simtime_t disseminationStartTime;

    Mac1609_4* mac;

    cMessage* CCHStartTimer;

    cMessage* SCHStartTimer;

    long lastNumPktLost;

    long lastNumCollisions;

    long totalCollisions;

    static const simsignalwrap_t mobilityStateChangedSignal;

    Veins::TraCIMobility* traci;

    // Used by the back-traffic application. Every time a node receives a beacon that has not been received in the last 3 seconds, then
    // it stores the info about the transmitter of the beacon and transmits 60 packets of 1000B. This simulates other applications
    // sharing the channel with the primary data dissemination.
    std::map<int, NeighborEntry*> lastRequesters;

    std::map<int, MessageInfoEntry*> messagesRcvd;

    virtual void initialize(int);
    virtual void finish();
    virtual void handleSelfMsg(cMessage* msg);
    virtual void onBeacon(WaveShortMessage* wsm);
    virtual void onData(WaveShortMessage* wsm);
    virtual bool isCCHActive();
    virtual double desynchronize(double delay);

    virtual bool isDuplicateMsg(int messageID);
    virtual bool isInsideROI(MessageInfoEntry* info);
    virtual bool isMessageAlive(MessageInfoEntry* info);
    virtual WaveShortMessage* createDataMsg(MessageInfoEntry* info);
    virtual void processBackTraffic(int senderAddr);

    virtual void addGPSError();
    virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, const SimTime& t);
    //TODO: Added for Game theory Solution
    virtual void adjustTxPower(WaveShortMessage* wsm);
    virtual void decreaseTxPower();
    virtual void increaseTxPower();

public:
    //TODO: Added for Game theory Solution
    simsignal_t lastTxPower;
    simsignal_t meanSNR;

    simsignal_t sentDownMACInCCH; // Indicate the total number of messages sent down to MAC when the CCH was active
    simsignal_t collisions; // Indicate the total number of collisions
    simsignal_t messagesTransmittedSCF; // Indicate the number of messages transmitted by a vehicle in the SCF state
    simsignal_t messagesReceivedSCF; // Indicate whether the message was received or not using indirect forwarding
    simsignal_t duplicatedMessages; // Indicate the number of duplicate messages received by a vehicle
    simsignal_t messagesTransmitted; // Indicate the number of messages transmitted by a vehicle
    simsignal_t messagesReceived; // Indicate whether the message was received or not
    simsignal_t isInROI; // Indicate whether the vehicle is inside the ROI at the time the dissemination starts

    virtual ~AID();
};

#endif

// Copyright (c) 2007-2017 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef WAVE_APP_H
#define WAVE_APP_H

#define SIZE_OF_ARRAY(array)    (sizeof(array)/sizeof(array[0]))

// The standard specified a message set, and its data elements specifically
// for use by applications intended to utilize the 5.9GHz Dedicated Short
// Range Communcations for Wireless Access in Vehicular Environments(DSRC/WAVE)

// DSRC = "Dedicated Short Range Communications" = (DSRC/WAVE)

#include "wave_net.h"
//追加
#include<fstream>
#include <algorithm>
#include <functional>
#include <array>

#include <cfenv>
#include <cmath>
#include <random>
//#include "tmp.h"

//unsigned int tmpId = 0;

namespace Wave {

typedef uint8_t DsrcMessageIdType;
enum {

    // Currently supported messages
    // 0. A La Carte
    // 1. Basic Safety Message

    DSRC_MESSAGE_A_LA_CARTE = 0,
    DSRC_MESSAGE_BASIC_SAFETY,


    // not yet
    DSRC_MESSAGE_COMMON_SAFETY_REQUEST,
    DSRC_MESSAGE_EMERGENCY_VEHICLE_ALERT,
    DSRC_MESSAGE_INTERSECTION_COLLISION_AVOIDANCE,
    DSRC_MESSAGE_MAP_DATA,
    DSRC_MESSAGE_NMEA_CORRECTIONS,
    DSRC_MESSAGE_PROBE_DATA_MANAGEMENT,
    DSRC_MESSAGE_PROBE_VEHICLE_DATA,
    DSRC_MESSAGE_ROADSIDE_ALERT,
    DSRC_MESSAGE_RTCM_CORRECTIONS,
    DSRC_MESSAGE_SIGNAL_PHASE_AND_TIMING,
    DSRC_MESSAGE_SIGNAL_REQUEST,
    DSRC_MESSAGE_SIGNAL_STATUS,
    DSRC_MESSAGE_TRAVELER_INFORMATION,
};//DsrcMessageIdType//

struct DsrcAccelerationSetType {
    uint16_t accelerationX;
    uint16_t accelerationY;
    uint8_t accelerationV;
    uint16_t yawRate;

    DsrcAccelerationSetType()
        :
        accelerationX(0),
        accelerationY(0),
        accelerationV(0),
        yawRate(0)
    {}

    DsrcAccelerationSetType(const char* payload)
        :
        accelerationX(*reinterpret_cast<const uint16_t* >(&payload[0])),
        accelerationY(*reinterpret_cast<const uint16_t* >(&payload[2])),
        accelerationV(*reinterpret_cast<const uint8_t* >(&payload[4])),
        yawRate(*reinterpret_cast<const uint16_t* >(&payload[5]))
    {}

    void Write(char* payload) const {
        *reinterpret_cast<uint16_t* >(&payload[0]) = uint16_t(accelerationX);
        *reinterpret_cast<uint16_t* >(&payload[2]) = uint16_t(accelerationY);
        *reinterpret_cast<uint8_t* >(&payload[4]) = uint8_t(accelerationV);
        *reinterpret_cast<uint16_t* >(&payload[5]) = uint16_t(yawRate);
    }
};//DsrcAccelerationSetType//

struct DsrcPositionAccurancyType {
    uint8_t semiMajorAccurancyMeters;
    uint8_t semiMinorAccurancyMeters;
    double orientationDegrees;

    DsrcPositionAccurancyType()
    :
        semiMajorAccurancyMeters(0),
        semiMinorAccurancyMeters(0),
        orientationDegrees(0)
    {}

    DsrcPositionAccurancyType(const char* payload)
        :
        semiMajorAccurancyMeters(*reinterpret_cast<const uint8_t* >(&payload[0]) / 2),
        semiMinorAccurancyMeters(*reinterpret_cast<const uint8_t* >(&payload[1]) / 2),
        orientationDegrees(*reinterpret_cast<const uint16_t* >(&payload[2]) * (360./65535))
    {}

    void Write(char* payload) const {
        *reinterpret_cast<uint8_t* >(&payload[0]) = uint8_t(semiMajorAccurancyMeters * 2);
        *reinterpret_cast<uint8_t* >(&payload[1]) = uint8_t(semiMinorAccurancyMeters * 2);
        *reinterpret_cast<uint16_t* >(&payload[2]) = uint16_t(orientationDegrees / (360./65535));
    }
};//DsrcPositionAccurancyType//

struct DsrcTransmissionAndSpeedType {
    uint16_t transmissionState;
    double speedMeters;

    struct FieldIoType {
        uint16_t state:2;
        uint16_t speed:14;
    };

    DsrcTransmissionAndSpeedType()
        :
        transmissionState(0),
        speedMeters(0)
    {}

    DsrcTransmissionAndSpeedType(const char* payload) {
        const FieldIoType& fieldIo = *reinterpret_cast<const FieldIoType* >(payload);
        transmissionState = fieldIo.state;
        speedMeters = fieldIo.speed * 0.02;
    }

    void Write(char* payload) const {
        FieldIoType& fieldIo = *reinterpret_cast<FieldIoType* >(payload);
        fieldIo.state = transmissionState;
        fieldIo.speed = uint16_t(speedMeters / 0.02);
    }
};//DsrcTransmissionAndSpeedType//

struct DsrcBrakeSystemStatusType {
    uint16_t wheelBrakes:4;
    uint16_t wheelBrakesUnavailable:1;
    uint16_t spareBit:1;
    uint16_t traction:2;
    uint16_t abs:2;
    uint16_t scs:2;
    uint16_t braksBoost:2;
    uint16_t auxBrakes:2;

    DsrcBrakeSystemStatusType()
        :
        wheelBrakes(0),
        wheelBrakesUnavailable(0),
        spareBit(0),
        traction(0),
        abs(0),
        scs(0),
        braksBoost(0),
        auxBrakes(0)
    {}
};//DsrcBrakeSystemStatusType//


struct DsrcVehicleSizeType {
    double widthMeters;
    double lengthMeters;

    DsrcVehicleSizeType()
        :
        widthMeters(0),
        lengthMeters(0)
    {}

    DsrcVehicleSizeType(const char* payload)
        :
        widthMeters(*reinterpret_cast<const uint8_t* >(&payload[0])), // simplified for simulation
        lengthMeters(*reinterpret_cast<const uint8_t* >(&payload[1]))
    {}

    void Write(char* payload) const {
        *reinterpret_cast<uint8_t* >(&payload[0]) = uint8_t(widthMeters);
        *reinterpret_cast<uint8_t* >(&payload[1]) = uint8_t(lengthMeters);
    }
};//DsrcVehicleSizeType//

struct DsrcALaCarteMessageType {

    DsrcMessageIdType messageId;
    char tailCrc[2];

    DsrcALaCarteMessageType() : messageId(DSRC_MESSAGE_A_LA_CARTE) {}
};//DsrcALaCarteMessageType//

struct DsrcBasicSafetyMessagePart1Type {

    DsrcMessageIdType messageId;
    char blob[38];

    uint8_t GetMessageCount() const { return *reinterpret_cast<const uint8_t* >(&blob[0]); }
    string GetTemporaryId() const { return string()+ blob[1]+blob[2]+blob[3]+blob[4]; }
    SimTime GetSecondMark() const { return SimTime(*reinterpret_cast<const uint16_t* >(&blob[5])) * MILLI_SECOND; }
    float GetXMeters() const { return *reinterpret_cast<const float* >(&blob[7]); }
    float GetYMeters() const { return *reinterpret_cast<const float* >(&blob[11]); }
    uint16_t GetElevationMeters() const { return (*reinterpret_cast<const uint16_t* >(&blob[15])) / 10; }
    DsrcPositionAccurancyType GetAccurancy() const { return DsrcPositionAccurancyType(&blob[17]); }
    DsrcTransmissionAndSpeedType GetSpeed() const { return DsrcTransmissionAndSpeedType(&blob[21]); }
    double GetHeading() const { return (*reinterpret_cast<const uint16_t* >(&blob[23]))*0.0125; }
    int GetAngleDegreesPerSec() const { return int(*reinterpret_cast<const int8_t* >(&blob[25]))*3; }
    DsrcAccelerationSetType GetAccelSet() const { return DsrcAccelerationSetType(&blob[26]); }
    DsrcBrakeSystemStatusType GetBrakes() const { return *reinterpret_cast<const DsrcBrakeSystemStatusType* >(&blob[33]); }
    DsrcVehicleSizeType GetSize() const { return DsrcVehicleSizeType(&blob[35]); }

    void SetMessageCount(const uint8_t messageCount) { *reinterpret_cast<uint8_t* >(&blob[0]) = messageCount; }
    void SetTemporaryId(const string& id) { assert(id.size() == 4); for(size_t i = 0; i < 4; i++) {blob[i+1] = id[i];} }
    void SetSecondMark(const SimTime& secondMark) { *reinterpret_cast<uint16_t* >(&blob[5]) = uint16_t(secondMark * MILLI_SECOND); }
    void SetXMeters(const float xMeters) { *reinterpret_cast<float* >(&blob[7]) = xMeters; }
    void SetYMeters(const float yMeters) { *reinterpret_cast<float* >(&blob[11]) = yMeters; }
    void SetElevationMeters(const uint16_t elevMeters) { *reinterpret_cast<uint16_t* >(&blob[15]) = uint16_t(elevMeters * 10); }
    void SetAccurancy(const DsrcPositionAccurancyType& arccurancy) { arccurancy.Write(&blob[17]); }
    void SetSpeed(const DsrcTransmissionAndSpeedType& speed) { speed.Write(&blob[21]); }
    void SetHeading(const double heading) { *reinterpret_cast<uint16_t* >(&blob[23]) = uint16_t(heading/0.0125); }
    void SetAngle(const int angleDegreesPerSec) { *reinterpret_cast<int8_t* >(&blob[25]) = int8_t(angleDegreesPerSec/3); }
    void SetAccelSet(const DsrcAccelerationSetType& accelSet) { accelSet.Write(&blob[26]); }
    void SetBrakes(const DsrcBrakeSystemStatusType& brakes) { *reinterpret_cast<DsrcBrakeSystemStatusType* >(&blob[33]) = brakes; }
    void SetSize(const DsrcVehicleSizeType& size) { size.Write(&blob[35]); }

    //ここにIDをblobにいれる関数を置く

    DsrcBasicSafetyMessagePart1Type();
};//DsrcBasicSafetyMessagePart1Type//

//追加
typedef struct{
    bool flag;
    SimTime transmissiontime;
    unsigned int destPri;
    unsigned int number[4];
    unsigned int numbercritical;
    unsigned int speed;
    double timetointersection;
}vehicular;

class DsrcPacketExtrinsicInformation : public ExtrinsicPacketInformation {
public:
    static const ExtrinsicPacketInfoId id;

    DsrcPacketExtrinsicInformation() : sequenceNumber(0), transmissionTime(ZERO_TIME) {}
    DsrcPacketExtrinsicInformation(
        const uint32_t initSequenceNumber,
        const SimTime& initTransmissionTime)
        :
        sequenceNumber(initSequenceNumber),
        transmissionTime(initTransmissionTime)
    {}

    virtual shared_ptr<ExtrinsicPacketInformation> Clone() {
        return shared_ptr<ExtrinsicPacketInformation>(
            new DsrcPacketExtrinsicInformation(*this));
    }

    uint32_t sequenceNumber;
    SimTime transmissionTime;
};//DsrcPacketExtrinsicInformation//

class DsrcMessageApplication : public Application {
public:
    DsrcMessageApplication(
        const ParameterDatabaseReader& theParameterDatabaseReader,
        const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
        const shared_ptr<WsmpLayer>& initWsmpLayerPtr,
        const NodeId& initNodeId,
        const RandomNumberGeneratorSeed& initNodeSeed,
        const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr);

    void SendALaCarteMessage(
        unique_ptr<Packet>& packetPtr,
        const ChannelNumberIndexType& channelNumberId,
        const string& providerServiceId,
        const PacketPriority& priority);

    void SendBasicSafetyMessage(
        const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1Type);

    void SendBasicSafetyMessage(
        const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1Type,
        const unsigned char* part2Payload,
        const size_t part2PayloadSize);


private:
    class PeriodicBasicSafetyMessageTransmissionEvent : public SimulationEvent {
    public:
        PeriodicBasicSafetyMessageTransmissionEvent(
            DsrcMessageApplication* initDsrcMessageApp) : dsrcMessageApp(initDsrcMessageApp) {}
        virtual void ExecuteEvent() { dsrcMessageApp->PeriodicallyTransmitBasicSafetyMessage(); }
    private:
        DsrcMessageApplication* dsrcMessageApp;
    };

    class PacketHandler: public WsmpLayer::WsmApplicationHandler {
    public:
        PacketHandler(DsrcMessageApplication* initDsrcMessageApp) : dsrcMessageApp(initDsrcMessageApp) {}

        virtual void ReceiveWsm(unique_ptr<Packet>& packetPtr) { dsrcMessageApp->ReceivePacketFromLowerLayer(packetPtr); }
    private:
        DsrcMessageApplication* dsrcMessageApp;
    };

    static const string basicSafetyAppModelName;
    static const ApplicationId theApplicationId;
    static const long long SEED_HASH = 1758612;

    shared_ptr<WsmpLayer> wsmpLayerPtr;
    shared_ptr<ObjectMobilityModel> nodeMobilityModelPtr;
    RandomNumberGenerator aRandomNumberGenerator;

    struct BasicSafetyMessageInfo {
        SimTime startTime;
        SimTime endTime;
        SimTime transmissionInterval;
        PacketPriority priority;
        string providerServiceId;
        size_t extendedPayloadSizeBytes;
        ChannelNumberIndexType channelNumberId;

        unsigned int currentSequenceNumber;
        unsigned int numberPacketsReceived;
        //追加
        unsigned int numberPacketsReceived2;
        //下追加
        NodeId MyNodeId;
        //追加
        unsigned int numberPacketSend;
        unsigned int numberPacketSendPCR[4];
        unsigned int numberPacketSendPCRinter[4];
        //unsigned int numberPacketSendSpeed[5];
        //unsigned int numberPacketReceivedSpeed[5];
        unsigned int numberPacketSendSpeed[6];
        unsigned int numberPacketSendSpeedInter[6];
        unsigned int numberPacketReceivedSpeed[6];
        unsigned int numberPacketReceivedSpeedInter[6];
        unsigned int numberPacketReceivedPriInter[4];

        unsigned int numberCriticalPacketSend;
        unsigned int numberPacketSendinintersection;
        unsigned int numberPacketReceivedinintersection;
        unsigned int sumnear;
        //vehicular list[800];
        vehicular list[1600];


        SimTime delaysum;
        SimTime delaysumP[4];
        SimTime delaysumPI[4];
        //SimTime delaysumS[5];
        SimTime delaysumS[6];
        SimTime delaysumSI[6];
        SimTime delayave;
        SimTime delayaveP[4];
        SimTime delayavePI[4];
        //SimTime delayaveS[5];
        SimTime delayaveS[6];
        SimTime delayaveSI[6];
        SimTime delaysum2;
        SimTime delayave2;
        SimTime delaymax;
        SimTime delaymax2;
        SimTime delaysumInter;
        SimTime delayaveInter;
        unsigned int delaydistribution[11];
        unsigned int delaydistributionS0[11];
        unsigned int delaydistributionS1[11];
        unsigned int delaydistributionS2[11];
        unsigned int delaydistributionS3[11];
        unsigned int delaydistributionS4[11];
        unsigned int delaydistributionS5[11];

        unsigned int priorityininter[4];
        unsigned int priorityin30_40[4];
        unsigned int priorityin40_50[4];
        unsigned int priorityin50_60[4];

        unsigned int priorityininterall[4];
        unsigned int priorityin30_40all[4];
        unsigned int priorityin40_50all[4];
        unsigned int priorityin50_60all[4];


        unsigned int nearvehi[4];
        unsigned int sumvehi[4];
        unsigned int totalnear;
        unsigned int totalinter;
        unsigned int totalnotinter;
        unsigned int totalmax;
        unsigned int sevencnt;
        unsigned int criticalMessagetime;
        bool alart;
        unsigned int upcnt;
        unsigned int downcnt;
        unsigned int TPC;


        shared_ptr<CounterStatistic> packetsSentStatPtr;
        shared_ptr<CounterStatistic> bytesSentStatPtr;
        shared_ptr<CounterStatistic> packetsReceivedStatPtr;
        shared_ptr<CounterStatistic> bytesReceivedStatPtr;
        shared_ptr<RealStatistic> endToEndDelayStatPtr;

        //メッセージ自体の情報?(車両情報などは除く?)
        BasicSafetyMessageInfo()
            :
            startTime(ZERO_TIME),
            endTime(ZERO_TIME),
            transmissionInterval(INFINITE_TIME),
            priority(0),
            providerServiceId("0"),
            extendedPayloadSizeBytes(0),
            channelNumberId(CHANNEL_NUMBER_178),
            currentSequenceNumber(0),
            numberPacketsReceived(0),
            //追加
            numberPacketsReceived2(0),
            MyNodeId(0),
            numberPacketSend(0),
            numberCriticalPacketSend(0),
            numberPacketSendinintersection(0),
            numberPacketReceivedinintersection(0),
            sumnear(0),
            delaysum(ZERO_TIME),
            delayave(ZERO_TIME),
            delaysum2(ZERO_TIME),
            delaymax(0),
            delaymax2(0),
            delayave2(ZERO_TIME),
            delaysumInter(ZERO_TIME),
            delayaveInter(ZERO_TIME),
            totalnear(0),
            totalinter(0),
            totalnotinter(0),
            totalmax(0),
            sevencnt(0),
            alart(false),
            criticalMessagetime(0),
            upcnt(0),
            downcnt(0),
            TPC(0)


        {
            //for (int i = 0; i < 800; i++) {
            for (int i = 0; i < 1600; i++) {
                list[i].flag = false;
                list[i].transmissiontime = ZERO_TIME;
                list[i].destPri = 0;
                for(int j = 0; j < 4; j++){
                    list[i].number[j] = 0;
                }
                list[i].numbercritical = 0;
                list[i].speed = 0;
                list[i].timetointersection = 999999;
            }
            for (int i = 0; i < 4; i++){
                nearvehi[i] = 0;
                sumvehi[i] = 0;
                numberPacketSendPCR[i] = 0;
                numberPacketSendPCRinter[i] = 0;
                numberPacketReceivedPriInter[i] = 0;
                delaysumP[i] = 0;
                delaysumPI[i] = 0;
                delayaveP[i] = 0;
                delayavePI[i] = 0;
                priorityininter[i] = 0;
                priorityin30_40[i] = 0;
                priorityin40_50[i] = 0;
                priorityin50_60[i] = 0;
                priorityininterall[i] = 0;
                priorityin30_40all[i] = 0;
                priorityin40_50all[i] = 0;
                priorityin50_60all[i] = 0;
            }
            /*for (int i = 0; i < 5; i++){
                numberPacketSendSpeed[i] = 0;
                numberPacketReceivedSpeed[i] = 0;
                delaysumS[i] = 0;
                delayaveS[i] = 0;
            }*/
            for (int i = 0; i < 6; i++){
                numberPacketSendSpeed[i] = 0;
                numberPacketSendSpeedInter[i] = 0;
                numberPacketReceivedSpeed[i] = 0;
                numberPacketReceivedSpeedInter[i] = 0;
                delaysumS[i] = 0;
                delaysumSI[i] = 0;
                delayaveS[i] = 0;
                delayaveSI[i] = 0;
            }
            for(int i = 0; i < 11; i++){
                delaydistribution[i] = 0;
                delaydistributionS0[i] = 0;
                delaydistributionS1[i] = 0;
                delaydistributionS2[i] = 0;
                delaydistributionS3[i] = 0;
                delaydistributionS4[i] = 0;
                delaydistributionS5[i] = 0;

            }
        }
    };

    BasicSafetyMessageInfo basicSafetyMessageInfo;


    void ReceivePacketFromLowerLayer(unique_ptr<Packet>& packetPtr);
    void ReceiveALaCarteMessage(unique_ptr<Packet>& packetPtr);
    void ReceiveBasicSafetyMessage(unique_ptr<Packet>& packetPtr);

    void PeriodicallyTransmitBasicSafetyMessage();

    void OutputTraceAndStatsForSendBasicSafetyMessage(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const size_t packetLengthBytes);

    void OutputTraceAndStatsForReceiveBasicSafetyMessage(
        const unsigned int sequenceNumber,
        const PacketId& thePacketId,
        const size_t packetLengthBytes,
        const SimTime& delay);
};//DsrcMessageApplication//

//--------------------------------------------------------------------------

inline
DsrcBasicSafetyMessagePart1Type::DsrcBasicSafetyMessagePart1Type()
    :
    messageId(DSRC_MESSAGE_BASIC_SAFETY)
{
    (*this).SetMessageCount(0);
    (*this).SetTemporaryId("0000");
    (*this).SetSecondMark(ZERO_TIME);
    (*this).SetXMeters(0);
    (*this).SetYMeters(0);
    (*this).SetElevationMeters(0);
    (*this).SetAccurancy(DsrcPositionAccurancyType());
    (*this).SetSpeed(DsrcTransmissionAndSpeedType());
    (*this).SetHeading(0);
    (*this).SetAngle(0);
    (*this).SetAccelSet(DsrcAccelerationSetType());
    (*this).SetBrakes(DsrcBrakeSystemStatusType());
    (*this).SetSize(DsrcVehicleSizeType());

    //ここでIDを持ってきて上に作る関数に入れる

}//DsrcBasicSafetyMessagePart1Type//

//下のコンストラクタは各車両で開始時に呼ばれる
inline
DsrcMessageApplication::DsrcMessageApplication(
    const ParameterDatabaseReader& theParameterDatabaseReader,
    const shared_ptr<SimulationEngineInterface>& initSimEngineInterfacePtr,
    const shared_ptr<WsmpLayer>& initWsmpLayerPtr,
    const NodeId& initNodeId,
    const RandomNumberGeneratorSeed& initNodeSeed,
    const shared_ptr<ObjectMobilityModel>& initNodeMobilityModelPtr)
    :
    Application(initSimEngineInterfacePtr, theApplicationId),
    wsmpLayerPtr(initWsmpLayerPtr),
    nodeMobilityModelPtr(initNodeMobilityModelPtr),
    aRandomNumberGenerator(HashInputsToMakeSeed(initNodeSeed, SEED_HASH))
{
    const SimTime jitter = static_cast<SimTime>(
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-start-time-max-jitter", initNodeId) *
        aRandomNumberGenerator.GenerateRandomDouble());

    basicSafetyMessageInfo.startTime =
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-start-time", initNodeId) + jitter;

    basicSafetyMessageInfo.endTime =
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-end-time", initNodeId) + jitter;

    //ここは通ってる
    //std::cout << "コンポーネント" << endl;
    //インターバルの設定を読み込んでいる

    basicSafetyMessageInfo.transmissionInterval =
        theParameterDatabaseReader.ReadTime("its-bsm-app-traffic-interval", initNodeId);

    basicSafetyMessageInfo.priority =
        ConvertToUChar(
            theParameterDatabaseReader.ReadNonNegativeInt("its-bsm-app-packet-priority", initNodeId),
            "Error in parameter: \"its-bsm-app-packet-priority\"");

    if (theParameterDatabaseReader.ParameterExists("its-bsm-app-service-provider-id", initNodeId)) {
        basicSafetyMessageInfo.providerServiceId =
            ConvertToProviderServiceIdString(theParameterDatabaseReader.ReadNonNegativeInt("its-bsm-app-service-provider-id", initNodeId));
    }//if//

    //パラメータがコンポーネントにない場合は規定のデータを使用する.
    basicSafetyMessageInfo.extendedPayloadSizeBytes = sizeof(DsrcBasicSafetyMessagePart1Type);

    if (theParameterDatabaseReader.ParameterExists("its-bsm-app-packet-payload-size-bytes", initNodeId)) {
        basicSafetyMessageInfo.extendedPayloadSizeBytes =
            theParameterDatabaseReader.ReadNonNegativeInt("its-bsm-app-packet-payload-size-bytes", initNodeId);

        if (basicSafetyMessageInfo.extendedPayloadSizeBytes < sizeof(DsrcBasicSafetyMessagePart1Type)) {
            cerr << "its-bsm-app-packet-payload-size-bytes must be more than "
                 << "BasicSafetyMessagePayloadPart1:"
                 << sizeof(DsrcBasicSafetyMessagePart1Type) << endl;
            exit(1);
        }//if//
    }//if//

    if (theParameterDatabaseReader.ParameterExists("its-bsm-channel-number", initNodeId)) {
        basicSafetyMessageInfo.channelNumberId =
            ConvertToChannelNumberIndex(
                theParameterDatabaseReader.ReadInt("its-bsm-channel-number", initNodeId));
    }//if//

    //追加
    basicSafetyMessageInfo.MyNodeId = initNodeId;

    basicSafetyMessageInfo.packetsSentStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName + "_PacketsSent"));

    basicSafetyMessageInfo.bytesSentStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName + "_BytesSent"));

    basicSafetyMessageInfo.packetsReceivedStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName + "_PacketsReceived"));

    basicSafetyMessageInfo.bytesReceivedStatPtr =
        simulationEngineInterfacePtr->CreateCounterStat(
            (basicSafetyAppModelName  + "_BytesReceived"));

    basicSafetyMessageInfo.endToEndDelayStatPtr =
        simulationEngineInterfacePtr->CreateRealStat(
            (basicSafetyAppModelName + "_EndToEndDelay"));

    wsmpLayerPtr->SetWsmApplicationHandler(
        basicSafetyMessageInfo.providerServiceId,
        shared_ptr<WsmpLayer::WsmApplicationHandler>(
            new PacketHandler(this)));

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();

    SimTime nextTransmissionTime = basicSafetyMessageInfo.startTime;

    if (currentTime > basicSafetyMessageInfo.startTime) {
        const size_t numberPassedTransmissionTimes =
            size_t(std::ceil(double(currentTime - basicSafetyMessageInfo.startTime) /
                             basicSafetyMessageInfo.transmissionInterval));

        //------------------------------------------------------------------------------
       
        //-------------------------------------------------------------------------------
        nextTransmissionTime =
            basicSafetyMessageInfo.startTime +
            numberPassedTransmissionTimes*basicSafetyMessageInfo.transmissionInterval;
            //ここのnextTransmissionTimeをを変えるべき__違った
            //std::cout << "次の送信時間＝" << (int)nextTransmissionTime << endl;
            //通ってない

    }//if//

    //std::cout << "次の送信時間＝" << (int)nextTransmissionTime << endl;
    //通ってる

    //次の送信が終わり時間を超えていなければ
    if (nextTransmissionTime < basicSafetyMessageInfo.endTime) {
        //ここ怪しい
        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new PeriodicBasicSafetyMessageTransmissionEvent(this)),
            nextTransmissionTime);
            //std::cout << "次の送信時間＝" << (int)nextTransmissionTime << endl;
            //通ってる
    }//if//
}//DsrcMessageApplication//

inline
void DsrcMessageApplication::SendALaCarteMessage(
    unique_ptr<Packet>& packetPtr,
    const ChannelNumberIndexType& channelNumberId,
    const string& providerServiceId,
    const PacketPriority& priority)
{
    packetPtr->AddPlainStructHeader(DsrcALaCarteMessageType());
    wsmpLayerPtr->SendWsm(
        packetPtr,
        NetworkAddress::broadcastAddress,
        channelNumberId,
        providerServiceId,
        priority);
}//SendALaCarteMessage//

inline
void DsrcMessageApplication::SendBasicSafetyMessage(
    const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1)
{
    (*this).SendBasicSafetyMessage(basicSafetyMessagePart1, nullptr, 0);
}//SendBasicSafetyMessage//

inline
void DsrcMessageApplication::SendBasicSafetyMessage(
    const DsrcBasicSafetyMessagePart1Type& basicSafetyMessagePart1,
    const unsigned char* part2Payload,
    const size_t part2PayloadSize)
{


    //ここでペイロードの中身を表示したい
    //cout << basicSafetyMessagePart1. << endl;

    const unsigned char* part1Payload =
        reinterpret_cast<const unsigned char* >(&basicSafetyMessagePart1);

    const size_t part1PayloadSize =
        sizeof(DsrcBasicSafetyMessagePart1Type);


    vector<unsigned char> payload(part1PayloadSize + part2PayloadSize);

    for(size_t i = 0; i < part1PayloadSize; i++) {
        payload[i] = part1Payload[i];
    }//for
    for(size_t i = 0; i < part2PayloadSize; i++) {
        payload[part1PayloadSize + i] = part2Payload[i];
    }//for


    unique_ptr<Packet> packetPtr = Packet::CreatePacket(*simulationEngineInterfacePtr, payload);

    /*if(basicSafetyMessageInfo.priority == 7){
        char fname[30];
        sprintf(fname,"tmpCritical_%d.txt",basicSafetyMessageInfo.MyNodeId);
        std::ofstream outputfile(fname);
        PacketId CriticalID = packetPtr->GetPacketId();
        unsigned long long int Sequencenum = CriticalID.GetSourceNodeSequenceNumber();*/
        /*if(basicSafetyMessageInfo.MyNodeId == 1){
            std::cout << "sequence num = " << Sequencenum << endl;
        }*/
        /*outputfile << Sequencenum << std::endl;
        outputfile.close();
    }*/

   

    
    packetPtr->AddExtrinsicPacketInformation(
        DsrcPacketExtrinsicInformation::id,
        shared_ptr<DsrcPacketExtrinsicInformation>(
            new DsrcPacketExtrinsicInformation(
                basicSafetyMessageInfo.currentSequenceNumber,
                simulationEngineInterfacePtr->CurrentTime())));

    (*this).OutputTraceAndStatsForSendBasicSafetyMessage(
        basicSafetyMessageInfo.currentSequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes());

    //std::cout << basicSafetyMessageInfo.MyNodeId <<" : messagepriority = " << (int)basicSafetyMessageInfo.priority << endl;
    //表示される


    //宛先アドレスはブロードキャストアドレス.途中からnextHopアドレスと名前を変えていた
    wsmpLayerPtr->SendWsm(
        packetPtr,
        NetworkAddress::broadcastAddress,
        basicSafetyMessageInfo.channelNumberId,
        basicSafetyMessageInfo.providerServiceId,
        basicSafetyMessageInfo.priority);

    basicSafetyMessageInfo.currentSequenceNumber++;
}//SendBasicSafetyMessage//


//定期的に送信
inline
void DsrcMessageApplication::PeriodicallyTransmitBasicSafetyMessage()
{
    //ScenSim::ObjectMobilityPosition omp;
    //short int currentSpeed = static_cast<short int>(omp.VelocityMetersPerSecond());
    //取得できてない:下のでいけた

    const SimTime currentTime = simulationEngineInterfacePtr->CurrentTime();
    /*if(basicSafetyMessageInfo.MyNodeId == 1){
        std::cout << "currentTime = " << currentTime << endl;
    }*/
    ObjectMobilityPosition position;
    DsrcBasicSafetyMessagePart1Type basicSafetyMessagePart1;

    basicSafetyMessagePart1.SetMessageCount(uint8_t(basicSafetyMessageInfo.currentSequenceNumber));
    basicSafetyMessagePart1.SetXMeters(float(position.X_PositionMeters()));
    basicSafetyMessagePart1.SetYMeters(float(position.Y_PositionMeters()));
    basicSafetyMessagePart1.SetElevationMeters(uint16_t(position.HeightFromGroundMeters()));
    //ここでbasicSaftyMaggagePart1.***でIDを取得したい
    //basicSafetyMessagePart1.SetHeading(double(position.VelocityAzimuthFromNorthClockwiseDegrees()));

    //下２行で速度取得可能
    nodeMobilityModelPtr->GetPositionForTime(currentTime, position);
    unsigned int currentSpeed = static_cast<int>(position.VelocityMetersPerSecond()*1000);

    //下２行はまだ試していない
    //shortははずしても良い？
    int currentX = static_cast<int>(position.X_PositionMeters()*1000);
    int currentY = static_cast<int>(position.Y_PositionMeters()*1000);

    int currenthead = static_cast<int>(position.VelocityAzimuthFromNorthClockwiseDegrees());

    //書き込み×３
    
    char fname2[30];
    sprintf(fname2,"tmpX_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfile2(fname2);
    outputfile2 << currentX << std::endl;
    outputfile2.close();

    char fname3[30];
    sprintf(fname3,"tmpY_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfile3(fname3);
    outputfile3 << currentY << std::endl;
    outputfile3.close();

    //パターン2
    /*char fname4[30];
    sprintf(fname4,"tmpHead_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfile4(fname4);
    outputfile4 << currenthead << std::endl;
    outputfile4.close();*/
    //パターン2

    /*if(basicSafetyMessageInfo.MyNodeId == 1){
        std::cout << "heading = " << currenthead << endl;
    }*/
    //自車の場所取得
    //std::cout << "X = " << currentX << endl;
    //std::cout << "Y = " << currentY << endl; 
    //------------------------------------------------------------------------------------

    //多分成功
    for(int i = 0; i < 4; i++){
        basicSafetyMessageInfo.nearvehi[i] = 0;
    }

    basicSafetyMessageInfo.sevencnt = 0;

    //double sorted[800];
    double sorted[1600];
    double intersectiontime;
            
    basicSafetyMessageInfo.totalinter = 0;
    basicSafetyMessageInfo.totalnotinter = 0;

    for(int i = 0; i < 1600; i++){
        sorted[i] = 0;
    }

    //for(int i = 0; i < 800; i++){
    for(int i = 0; i < 1600; i++){
        if(basicSafetyMessageInfo.list[i].flag == true){
            //std::cout << "1入り" << endl;
            if(abs(currentTime - basicSafetyMessageInfo.list[i].transmissiontime) > (basicSafetyMessageInfo.transmissionInterval * 10)){
                //1000台で5なら平均90台くらい,3なら80くらい
                //750台で3なら70くらい
                //500台で3なら55くらい
                //250台で3なら35くらい

                //833台で3なら72くらい
                //666台で3なら69くらい
                //500台で3なら55くらい
                //333
                //167

                //666台で3なら69くらい
                //333

                //improved
                //52以上で上昇？
                //44以下で下降？

                //normal
                //32以上
                //40以上
                //16以下
                //12以下

                //600台ave75 max85くらい

                /*std::cout << "i = " << i+1 << endl;
                std::cout << "vehi = " << basicSafetyMessageInfo.list[i].transmissiontime << endl;
                std::cout << "Current = " << currentTime << endl;
                std::cout << "interval = " << basicSafetyMessageInfo.transmissionInterval << endl;
                std::cout << "受け取り時間差 = " << abs(basicSafetyMessageInfo.list[i].transmissiontime - currentTime) << endl;*/
                basicSafetyMessageInfo.list[i].flag = false;
                basicSafetyMessageInfo.list[i].transmissiontime = ZERO_TIME;
                basicSafetyMessageInfo.list[i].destPri = 0;
                basicSafetyMessageInfo.list[i].speed = 0;
                basicSafetyMessageInfo.list[i].timetointersection = 999999;
            }else{
                /*std::cout << "i2 = " << i+1 << endl;
                std::cout << "vehi2 = " << basicSafetyMessageInfo.list[i].transmissiontime << endl;
                std::cout << "Current2 = " << currentTime << endl;
                std::cout << "interval2 = " << basicSafetyMessageInfo.transmissionInterval << endl;
                std::cout << "受け取り時間差2 = " << abs(basicSafetyMessageInfo.list[i].transmissiontime - currentTime) << endl;*/
                if(basicSafetyMessageInfo.list[i].destPri == 1 || basicSafetyMessageInfo.list[i].destPri == 2){
                    basicSafetyMessageInfo.nearvehi[0]++;
                    basicSafetyMessageInfo.totalnotinter++;
                }else if(basicSafetyMessageInfo.list[i].destPri == 0 || basicSafetyMessageInfo.list[i].destPri == 3){
                    basicSafetyMessageInfo.nearvehi[1]++;
                    basicSafetyMessageInfo.totalnotinter++;
                /*}else if(basicSafetyMessageInfo.list[i].destPri == 4 && basicSafetyMessageInfo.list[i].timetointersection == 0){
                    basicSafetyMessageInfo.nearvehi[2]++;
                    basicSafetyMessageInfo.sevencnt++;
                }else if(basicSafetyMessageInfo.list[i].destPri == 4 || basicSafetyMessageInfo.list[i].destPri == 5){
                    basicSafetyMessageInfo.nearvehi[2]++;
                }else if(basicSafetyMessageInfo.list[i].destPri == 6){
                    basicSafetyMessageInfo.nearvehi[3]++;
                }*/

                }else if(basicSafetyMessageInfo.list[i].destPri == 4 && basicSafetyMessageInfo.list[i].timetointersection == 0){
                    basicSafetyMessageInfo.nearvehi[2]++;
                    basicSafetyMessageInfo.sevencnt++;
                    basicSafetyMessageInfo.totalinter++;
                }else if(basicSafetyMessageInfo.list[i].destPri == 4 || basicSafetyMessageInfo.list[i].destPri == 5){
                    basicSafetyMessageInfo.nearvehi[2]++;
                    basicSafetyMessageInfo.totalnotinter++;
               //}else if(basicSafetyMessageInfo.list[i].destPri == 6 || basicSafetyMessageInfo.list[i].destPri == 7){
                }else if(basicSafetyMessageInfo.list[i].destPri == 6 && basicSafetyMessageInfo.list[i].timetointersection == 0){
                    basicSafetyMessageInfo.nearvehi[3]++;
                    basicSafetyMessageInfo.sevencnt++;
                    basicSafetyMessageInfo.totalinter++;
                }else if(basicSafetyMessageInfo.list[i].destPri == 6){
                    basicSafetyMessageInfo.nearvehi[3]++;
                    basicSafetyMessageInfo.totalnotinter++;
                }else if(basicSafetyMessageInfo.list[i].destPri == 7){
                    basicSafetyMessageInfo.nearvehi[3]++;
                    basicSafetyMessageInfo.totalnotinter++;
                }
                basicSafetyMessageInfo.totalnear = basicSafetyMessageInfo.nearvehi[0] + basicSafetyMessageInfo.nearvehi[1] + basicSafetyMessageInfo.nearvehi[2] + basicSafetyMessageInfo.nearvehi[3];

                if(basicSafetyMessageInfo.totalmax < basicSafetyMessageInfo.totalnear){
                    basicSafetyMessageInfo.totalmax = basicSafetyMessageInfo.totalnear;
                }
                
            }
        }else{
            //std::cout << "入ってない" << endl;
        }
        
        //パターン1
        /*if(basicSafetyMessageInfo.list[i].destPri == 6 && basicSafetyMessageInfo.list[i].timetointersection == 0){
            sorted[i] = 100000;
        }else if(basicSafetyMessageInfo.list[i].destPri == 4 && basicSafetyMessageInfo.list[i].timetointersection == 0){
            sorted[i] = 90000;
        }else*/
        
        
            if((i + 1) == basicSafetyMessageInfo.MyNodeId){
                sorted[i] = currentSpeed;
            }else if(basicSafetyMessageInfo.list[i].timetointersection != 0 && basicSafetyMessageInfo.list[i].flag == true){
                sorted[i] = basicSafetyMessageInfo.list[i].speed;
            }
           
        

            
        //パターン1

        //パターン2
        /*if(i + 1 == basicSafetyMessageInfo.MyNodeId){

            double Speed = currentSpeed * 1000;

            if(currenthead == 0){
                if(currentY >= 10000 && currentY <= 190000){
                    double YD = 190000 - currentY;
                    intersectiontime = YD / Speed;

                }else if(currentY <= -10000 && currentY >= -190000){
                    double YD = -10000 + (currentY * (-1));
                    intersectiontime = YD / Speed;

                }else{
                    intersectiontime = 0;

                }

            }else if(currenthead == 90 || currenthead == -270){
                if(currentX >= 10000 && currentX <= 190000){
                    double XD = 190000 - currentX;
                    intersectiontime = XD / Speed;

                }else if(currentX <= -10000 && currentX >= -190000){
                    double XD = -10000 + (currentX * (-1));
                    intersectiontime = XD / Speed;

                }else{
                    intersectiontime = 0;
                }

            }else if(currenthead == 180 || currenthead == -180){
                if(currentY >= 10000 && currentY <= 190000){
                    double YD = currentY - 10000;
                    intersectiontime = YD / Speed;

                }else if(currentY <= -10000 && currentY >= -190000){
                    double YD = currentY + 190000;
                    intersectiontime = YD / Speed;

                }else{
                    intersectiontime = 0;
                }

            }else if(currenthead == -90 || currenthead == 270){
                if(currentX >= 10000 && currentX <= 190000){
                    double XD = currentX - 10000;
                    intersectiontime = XD / Speed;

                }else if(currentX <= -10000 && currentX >= -190000){
                    double XD =currentX + 190000;
                    intersectiontime = XD / Speed;

                }else{
                    intersectiontime = 0;
                }

            }else{
                intersectiontime = 0;
            }

            if(basicSafetyMessageInfo.MyNodeId == 1){
                //std::cout << "intersectiontime = " << intersectiontime * 1000 << endl;
                //std::cout << "speed = " << currentSpeed << endl;

            }

            sorted[i] = intersectiontime;

        }else{
            sorted[i] = basicSafetyMessageInfo.list[i].timetointersection;
        }*/
        //パターン2
    }
    
    /*if(basicSafetyMessageInfo.MyNodeId == 1){
        //std::cout << "intersectiontime = " << intersectiontime * 1000 << endl;
        std::cout << "speed = " << currentSpeed << endl;

    }*/
    //書き込み
    /*char fname5[30];
    sprintf(fname5,"tmptotalnear_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfile5(fname5);
    outputfile5 << basicSafetyMessageInfo.totalnear << std::endl;
    outputfile5.close();*/
    /*if(basicSafetyMessageInfo.MyNodeId == 1){
        std::cout << "nearvehi1 = " << basicSafetyMessageInfo.nearvehi[0] << endl;
        std::cout << "nearvehi2 = " << basicSafetyMessageInfo.nearvehi[1] << endl;
        std::cout << "nearvehi3 = " << basicSafetyMessageInfo.nearvehi[2] << endl;
        std::cout << "nearvehi4 = " << basicSafetyMessageInfo.nearvehi[3] << endl;
        std::cout << "totalnear = " << basicSafetyMessageInfo.totalnear << endl;
    }*/

    bool intersectionflag = false;

    //if((basicSafetyMessageInfo.priority != 2) && (basicSafetyMessageInfo.priority != 3) && (basicSafetyMessageInfo.priority != 5) && (basicSafetyMessageInfo.priority != 6)){
    //if((basicSafetyMessageInfo.MyNodeId != 1) && (basicSafetyMessageInfo.priority != 3)){
    if((basicSafetyMessageInfo.priority != 3) && (basicSafetyMessageInfo.priority != 5) && (basicSafetyMessageInfo.priority != 7)){
        //std::cout << "aaaaa" << endl;

        /*if(basicSafetyMessageInfo.totalnear > xx){
            basicSafetyMessageInfo.upcnt++;
            basicSafetyMessageInfo.downcnt = 0;
        }else if(basicSafetyMessageInfo.totalnear < xx){
            basicSafetyMessageInfo.downcnt++;
            basicSafetyMessageInfo.upcnt = 0;
        }else{
            basicSafetyMessageInfo.upcnt = 0;
            basicSafetyMessageInfo.downcnt = 0;
        }

        if(basicSafetyMessageInfo.upcnt == 10 && basicSafetyMessageInfo.TPC < 2){
            basicSafetyMessageInfo.TPC++;
            basicSafetyMessageInfo.upcnt = 0;
        }else if(basicSafetyMessageInfo.upcnt == 10){
            basicSafetyMessageInfo.upcnt = 0;
        }
        if(basicSafetyMessageInfo.downcnt == 50 && basicSafetyMessageInfo.TPC > 0){
            basicSafetyMessageInfo.TPC--;
            basicSafetyMessageInfo.downcnt = 0;
        }else if(basicSafetyMessageInfo.downcnt == 50){
            basicSafetyMessageInfo.downcnt = 0;
        }

        if(basicSafetyMessageInfo.MyNodeId == 1){
            std::cout << "upcnt = " << basicSafetyMessageInfo.upcnt << endl;
            std::cout << "downcnt = " << basicSafetyMessageInfo.downcnt << endl;
            std::cout << "TPC = " << basicSafetyMessageInfo.TPC << endl;
        }

        //TPC書き込み
        char fname6[30];
        sprintf(fname6,"TPC_%d.txt",basicSafetyMessageInfo.MyNodeId);
        std::ofstream outputfile6(fname6);
        outputfile6 << basicSafetyMessageInfo.TPC << std::endl;
        outputfile6.close();
        */
   
        //priority調整
        unsigned int currentCW[4];

        for(int i = 0; i < 4; i++){
            char fname7[30];
            std::string myCWS;
            sprintf(fname7,"tmpCW_%d_%d.txt", i, basicSafetyMessageInfo.MyNodeId);
            std::ifstream inputfile7(fname7);
            //今のところ優先度を取得
            inputfile7 >> myCWS;
            currentCW[i] = atoi(myCWS.c_str());
            if(!inputfile7){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                currentCW[0] = 15;
                currentCW[1] = 15;
                currentCW[2] = 7;
                currentCW[3] = 3;
            }
            /*
            if(basicSafetyMessageInfo.MyNodeId == 1){
                std::cout << "CW" << i << " = " << currentCW[i] << endl;
            }
            */
            //15,15,7,3
            inputfile7.close();
        }

        

        
        //パターン1
        std::sort(sorted, sorted + SIZE_OF_ARRAY(sorted), std::greater<unsigned int>());
        //パターン1

        //パターン2
        //std::sort(sorted, sorted + SIZE_OF_ARRAY(sorted), std::less<unsigned int>());
        //パターン2

        double per[4];
        
        per[0] = (currentCW[0]/2) + 9;
        //16 40%
        per[1] = (currentCW[1]/2) + 6;
        //13 35%
        per[2] = (currentCW[2]/2) + 3;
        //10 16.6%
        per[3] = (currentCW[3]/2) + 2;
        //3 8%
        

        
        /*per[0] = 9;//45%
        per[1] = 6;//30%
        per[2] = 3;//15%
        per[3] = 2;//10%*/

        /*per[0] = 15;//37.5%
        per[1] = 15;//37.5%
        per[2] = 7;//17.5%
        per[3] = 3;//7.5%*/

        /*per[0] = 4000;//40%
        per[1] = 4000;//40%
        per[2] = 1500;//15%
        per[3] = 500;//5%*/

        per[0] = 4500;//40%
        per[1] = 3500;//40%
        per[2] = 1500;//15%
        per[3] = 500;//5%

        /*per[0] = 4250;//37.5%
        per[1] = 3250;//37.5%
        per[2] = 1750;//17.5%
        per[3] = 750;//7.5%*/

        /*per[0] = 6000;//37.5%
        per[1] = 2400;//37.5%
        per[2] = 1180;//17.5%
        per[3] = 480;*/

        /*per[0] = 4250;
        per[1] = 3250;
        per[2] = 1500;
        per[3] = 1000;*/

        /*per[0] = 4000;
        per[1] = 3000;
        per[2] = 2000;
        per[3] = 1000;*/

        /*per[0] = 3500;
        per[1] = 3000;
        per[2] = 2000;
        per[3] = 1500;*/

        /*per[0] = 2500;//37.5%
        per[1] = 2500;//37.5%
        per[2] = 2500;//17.5%
        per[3] = 2500;//7.5%*/

        /*per[0] = 15;//37.5%
        per[1] = 15;//37.5%
        per[2] = 7;//17.5%
        per[3] = 0;//7.5%*/

        /*per[0] = 4;//37.5%
        per[1] = 3;//37.5%
        per[2] = 2;//17.5%
        per[3] = 1;//7.5%*/


        /*if(basicSafetyMessageInfo.MyNodeId == 1){
            std::cout << "totalnear = " << basicSafetyMessageInfo.totalnear << endl;
            for(int i = 0; i < 4; i++){
                std::cout << "per" << i << " = " << per[i] << endl;
            }
        }*/

        double backoffper[4];
        backoffper[0] = per[0] / (per[0] + per[1] + per[2] + per[3]);
        backoffper[1] = per[1] / (per[0] + per[1] + per[2] + per[3]);
        backoffper[2] = per[2] / (per[0] + per[1] + per[2] + per[3]);
        backoffper[3] = per[3] / (per[0] + per[1] + per[2] + per[3]);

        /*backoffper[0] = (2.0/6.0);
        backoffper[1] = (2.0/6.0);
        backoffper[2] = (1.0/6.0);
        backoffper[3] = (1.0/6.0);*/ //16.6%

        /*if(basicSafetyMessageInfo.MyNodeId == 1){
            std::cout << "totalnear = " << basicSafetyMessageInfo.totalnear << endl;
            for(int i = 0; i < 4; i++){
                std::cout << "backoffper" << i << " = " << backoffper[i] << endl;
            }
        }*/

        int nearper[4];
        //切り上げ
        //nearper[0] = std::ceil(basicSafetyMessageInfo.totalnotinter * backoffper[3]);
        //nearper[1] = std::ceil(basicSafetyMessageInfo.totalnotinter * backoffper[2]);
        nearper[2] = std::ceil(basicSafetyMessageInfo.totalnotinter * backoffper[1]);
        nearper[3] = std::ceil(basicSafetyMessageInfo.totalnotinter * backoffper[0]);

        //切り捨て
        nearper[0] = basicSafetyMessageInfo.totalnotinter * backoffper[3];
        nearper[1] = basicSafetyMessageInfo.totalnotinter * backoffper[2];
        //nearper[2] = basicSafetyMessageInfo.totalnotinter * backoffper[1];
        //nearper[3] = basicSafetyMessageInfo.totalnotinter * backoffper[0];



        /*//nearper[0] = std::ceil(basicSafetyMessageInfo.totalnear * backoffper[3]);
        //nearper[1] = std::ceil(basicSafetyMessageInfo.totalnear * backoffper[2]);
        nearper[2] = std::ceil(basicSafetyMessageInfo.totalnear * backoffper[1]);
        nearper[3] = std::ceil(basicSafetyMessageInfo.totalnear * backoffper[0]);

        //切り捨て
        nearper[0] = basicSafetyMessageInfo.totalnear * backoffper[3];
        nearper[1] = basicSafetyMessageInfo.totalnear * backoffper[2];
        //nearper[2] = basicSafetyMessageInfo.totalnear * backoffper[1];
        //nearper[3] = basicSafetyMessageInfo.totalnear * backoffper[0];*/

        for(int i = 0; i < 4; i++){
            if(nearper[i] == 0){
                nearper[i] = 1;
            }
        }
        /*for(int i = 1; i < 4; i++){
            if(nearper[i] == 0){
                nearper[i] = 1;
            }
        }*/

        basicSafetyMessageInfo.numberPacketSend++;
        basicSafetyMessageInfo.sumnear += basicSafetyMessageInfo.totalnear;

        /*if(basicSafetyMessageInfo.MyNodeId == 1){
            //std::cout << "totalnear" << basicSafetyMessageInfo.MyNodeId << " = " << basicSafetyMessageInfo.totalnear << endl;
            std::cout << "nearave" << basicSafetyMessageInfo.MyNodeId << " = "<< basicSafetyMessageInfo.sumnear / basicSafetyMessageInfo.numberPacketSend << endl;
            std::cout << "nearmax" << basicSafetyMessageInfo.MyNodeId << " = " << basicSafetyMessageInfo.totalmax << endl;
    
            for(int i = 0; i < 4; i++){
                std::cout << "nearvehi" << i << " = " << basicSafetyMessageInfo.nearvehi[i] << endl;
                basicSafetyMessageInfo.sumvehi[i] += basicSafetyMessageInfo.nearvehi[i];
                std::cout << "avevehi" << i << " = " << basicSafetyMessageInfo.sumvehi[i]/basicSafetyMessageInfo.numberPacketSend << endl;
            }
            if(basicSafetyMessageInfo.numberPacketSend == 1000){
                char fname33[30];
                sprintf(fname33,"result_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfile33(fname33);
                outputfile33 << "nearave" << basicSafetyMessageInfo.MyNodeId << " = "<< basicSafetyMessageInfo.sumnear / basicSafetyMessageInfo.numberPacketSend << std::endl;
                outputfile33 << "nearmax" << basicSafetyMessageInfo.MyNodeId << " = " << basicSafetyMessageInfo.totalmax << std::endl;
                for(int i = 0;i < 4; i++){
                    outputfile33 << "avevehi" << i << " = " << basicSafetyMessageInfo.sumvehi[i]/basicSafetyMessageInfo.numberPacketSend << std::endl;
                }
                outputfile33.close();

            }*/
            //std::cout << LLONG_MAX + 1 << endl;
            /*for(int i = 0; i < 4; i++){
                std::cout << "nearper" << i << " = " << nearper[i] << endl;
                //std::cout << "near vehi" << i << " = " << basicSafetyMessageInfo.nearvehi[i] << endl;
            }*/
        //}

        //パターン1
        unsigned int currentSpeedPos = 0;

        /*for(int i = 1440; i > 0; i--){
            if(sorted[i] == currentSpeed){
                currentSpeedPos = i;
                if(currentSpeedPos <= nearper[0]){
                    basicSafetyMessageInfo.priority = 6;
                }else if(currentSpeedPos <= (nearper[0] + nearper[1])){
                    basicSafetyMessageInfo.priority = 4;
                }else if(currentSpeedPos <= (nearper[0] + nearper[1] + nearper[2])){
                    basicSafetyMessageInfo.priority = 0;
                }else{
                    basicSafetyMessageInfo.priority = 1;
                }
                break;
            }
        }*/
        //パターン1
        
        //パターン2
        //int v1 = rand() % 2 + 1;
        /*unsigned int currentInterPos;*/

        //if(v1 == 1){
            /*for(int i = 0; i < 1440; i++){
                if(sorted[i] == intersectiontime){
                    currentInterPos = i;
                    if(currentInterPos <= nearper[0]){
                        basicSafetyMessageInfo.priority = 6;
                    }else if(currentInterPos <= (nearper[0] + nearper[1])){
                        basicSafetyMessageInfo.priority = 4;
                    }else if(currentInterPos <= (nearper[0] + nearper[1] + nearper[2])){
                        basicSafetyMessageInfo.priority = 0;
                    }else{
                        basicSafetyMessageInfo.priority = 1;
                    }
                    break;
                }
            }*/
        //}else{
            //パターン2
            //パターン1
            //if((nearper[0] + nearper[1]) < basicSafetyMessageInfo.sevencnt){
            //if((nearper[0] + nearper[1]) < basicSafetyMessageInfo.sevencnt){

                /*backoffper[0] = (2.0/5.0);
                backoffper[1] = (2.0/5.0);
                backoffper[2] = (1.0/5.0);*/


                /*backoffper[0] = per[0] / (per[0] + per[1] + per[2]);
                backoffper[1] = per[1] / (per[0] + per[1] + per[2]);
                backoffper[2] = per[2] / (per[0] + per[1] + per[2]);*/

                /*backoffper[0] = per[0] / (per[0] + per[1]);
                backoffper[1] = per[1] / (per[0] + per[1]);*/

                /*backoffper[0] = per[0] / (per[0] + per[1] + per[2]);
                backoffper[1] = per[1] / (per[0] + per[1] + per[2]);*/

                /*nearper[1] = std::ceil((basicSafetyMessageInfo.totalnear - basicSafetyMessageInfo.sevencnt) * backoffper[2]);
                nearper[2] = (basicSafetyMessageInfo.totalnear - basicSafetyMessageInfo.sevencnt) * backoffper[1];
                nearper[3] = (basicSafetyMessageInfo.totalnear - basicSafetyMessageInfo.sevencnt) * backoffper[0];*/

                /*nearper[2] = (basicSafetyMessageInfo.totalnear - basicSafetyMessageInfo.sevencnt) * backoffper[1];
                nearper[3] = (basicSafetyMessageInfo.totalnear - basicSafetyMessageInfo.sevencnt) * backoffper[0];*/

                /*nearper[2] = std::ceil((basicSafetyMessageInfo.totalnear - basicSafetyMessageInfo.sevencnt) * backoffper[1]);
                nearper[3] = (basicSafetyMessageInfo.totalnear - basicSafetyMessageInfo.sevencnt) * backoffper[0];*/

                /*nearper[1] = (basicSafetyMessageInfo.totalnear - basicSafetyMessageInfo.sevencnt) * backoffper[2];
                nearper[2] = std::ceil((basicSafetyMessageInfo.totalnear - basicSafetyMessageInfo.sevencnt) * backoffper[1]);
                nearper[3] = std::ceil((basicSafetyMessageInfo.totalnear - basicSafetyMessageInfo.sevencnt) * backoffper[0]);*/

                /*for(int i = 0; i < 4; i++){
                    if(nearper[i] == 0){
                        nearper[i] = 1;
                    }
                }*/

                //for(int i = 799; i >= 0; i--){
                /*for(int i = 1599; i >= 0; i--){
                    if(sorted[i] == currentSpeed){
                        currentSpeedPos = i;
                        if(currentSpeedPos <= (basicSafetyMessageInfo.sevencnt + nearper[2])){
                            basicSafetyMessageInfo.priority = 0;
                        }else{
                            basicSafetyMessageInfo.priority = 1;
                        }
                        break;
                    }
                }*/
                
                /*for(int i = 1599; i >= 0; i--){
                    if(sorted[i] == currentSpeed){
                        currentSpeedPos = i;
                        if(currentSpeedPos <= (basicSafetyMessageInfo.sevencnt + nearper[2])){
                            basicSafetyMessageInfo.priority = 0;
                        }else{
                            basicSafetyMessageInfo.priority = 1;
                        }
                        break;
                    }
                }*/
            //}else{

            /*if(basicSafetyMessageInfo.MyNodeId == 1){
                for(int i = 1599; i >= 0; i--){
                    std::cout << sorted[i] << endl;
                }
                for(int i = 0;i < 4; i++){
                    std::cout << nearper[i] << endl;
                }
                std::cout << "speed = " << currentSpeed << endl;
                std::cout << "not inter = " << basicSafetyMessageInfo.totalnotinter << endl;

            }*/
                //for(int i = 799; i >= 0; i--){
                for(int i = 1599; i >= 0; i--){
                    if(sorted[i] == currentSpeed){
                        currentSpeedPos = i;
                        if(currentSpeedPos <= nearper[0]){
                            basicSafetyMessageInfo.priority = 6;
                        }else if(currentSpeedPos <= (nearper[0] + nearper[1])){
                            basicSafetyMessageInfo.priority = 4;
                        }else if(currentSpeedPos <= (nearper[0] + nearper[1] + nearper[2])){
                            basicSafetyMessageInfo.priority = 0;
                        }else{
                            basicSafetyMessageInfo.priority = 1;
                        }
                        break;
                    }
                }

                /*if(basicSafetyMessageInfo.MyNodeId == 1){
                    std::cout << nearper[0] << endl;
                    std::cout << currentSpeedPos << endl;
                }*/
                
            //}
            //パターン1
            //パターン2
            /*for(int i = 1439; i >= 0; i--){
                if(sorted[i] == intersectiontime){
                    currentInterPos = i;
                    if(currentInterPos <= nearper[0]){
                        basicSafetyMessageInfo.priority = 6;
                    }else if(currentInterPos <= (nearper[0] + nearper[1])){
                        basicSafetyMessageInfo.priority = 4;
                    }else if(currentInterPos <= (nearper[0] + nearper[1] + nearper[2])){
                        basicSafetyMessageInfo.priority = 0;
                    }else{
                        basicSafetyMessageInfo.priority = 1;
                    }
                    break;
                }
            }*/
        //}
        //パターン2


        //for(int i = 0; i < 1440; i++){
            //if(basicSafetyMessageInfo.list[i].speed != 0){
                
            //}else{
                //basicSafetyMessageInfo.priority = 0;
            //}
        //}
        
        /*if(basicSafetyMessageInfo.MyNodeId == 1){
            for(int i = 0; i < 100; i++){
                std::cout << i << " = " << sorted[i] << endl;
            }
        }*/

        

        /*if(basicSafetyMessageInfo.MyNodeId == 1){
            for(int i = 0; i < 50; i++){
                std::cout << i << "-2 = " << sorted[i] << endl;
            }
        }*/
        //ソート成功
        

        /*if(19.4 > currentSpeed >= 13.8){          //50km/h
            basicSafetyMessageInfo.priority = 4;
        }else if(currentSpeed >= 19.4){    //70km/h
            basicSafetyMessageInfo.priority = 6;
        }else if(currentSpeed <= 8.3){     //30km/h
            basicSafetyMessageInfo.priority = 1;
        }else{
            basicSafetyMessageInfo.priority = 0;
        }*/


        //パターン1small
        /*const int intersectionPos = 200000;
        const int intersectionRange = 10000;

        //交差点付近ならprioritymax
        
        if((abs(currentX) <= intersectionRange) && (abs(currentY) <= intersectionRange)){
            basicSafetyMessageInfo.priority = 6;
            //std::cout << "id = " << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "X = " << currentX << endl;
            //std::cout << "Y = " << currentY << endl;

        }else if((abs(abs(currentX) - intersectionPos) <= intersectionRange) && (abs(currentY) <= intersectionRange)){
            basicSafetyMessageInfo.priority = 6;
            //std::cout << "id = " << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "X = " << currentX << endl;
            //std::cout << "Y = " << currentY << endl;

        }else if((abs(abs(currentY) - intersectionPos) <= intersectionRange) && (abs(currentX) <= intersectionRange)){
            basicSafetyMessageInfo.priority = 6;
            //std::cout << "id = " << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "X = " << currentX << endl;
            //std::cout << "Y = " << currentY << endl;

        }else{

        }*/
        //パターン1

        //パターン1midlar
        const int intersectionPos = 200000;
        const int intersectionRange = 10000;

        //交差点付近ならprioritymax
        
        if((abs(currentX) <= intersectionRange) && (abs(currentY) <= intersectionRange)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "id = " << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "X = " << currentX << endl;
            //std::cout << "Y = " << currentY << endl;

            intersectionflag = true;

            //受信車両50のみ
            //if(0 <= currentY && currentY <= 10000){
                basicSafetyMessageInfo.numberPacketSendinintersection++;

                if(0 <= currentSpeed && currentSpeed <= 2777){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[0]++;
                }else if(2777 < currentSpeed  && currentSpeed <= 5555){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[1]++;
                }else if(5555 < currentSpeed && currentSpeed <= 8333){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[2]++;
                }else if(8333 < currentSpeed && currentSpeed <= 11111){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[3]++;
                }else if(11111 < currentSpeed && currentSpeed <= 13888){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[4]++;
                }else if(13888 < currentSpeed && currentSpeed <= 16667){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[5]++;
                }

                char fnameA[30];
                sprintf(fnameA,"tmpSendIntersectionSpeed0_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileA(fnameA);
                outputfileA << basicSafetyMessageInfo.numberPacketSendSpeedInter[0] << std::endl;
                outputfileA.close();

                char fnameB[30];
                sprintf(fnameB,"tmpSendIntersectionSpeed1_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileB(fnameB);
                outputfileB << basicSafetyMessageInfo.numberPacketSendSpeedInter[1] << std::endl;
                outputfileB.close();

                char fnameC[30];
                sprintf(fnameC,"tmpSendIntersectionSpeed2_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileC(fnameC);
                outputfileC << basicSafetyMessageInfo.numberPacketSendSpeedInter[2] << std::endl;
                outputfileC.close();

                char fnameD[30];
                sprintf(fnameD,"tmpSendIntersectionSpeed3_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileD(fnameD);
                outputfileD << basicSafetyMessageInfo.numberPacketSendSpeedInter[3] << std::endl;
                outputfileD.close();

                char fnameE[30];
                sprintf(fnameE,"tmpSendIntersectionSpeed4_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileE(fnameE);
                outputfileE << basicSafetyMessageInfo.numberPacketSendSpeedInter[4] << std::endl;
                outputfileE.close();

                char fnameF[30];
                sprintf(fnameF,"tmpSendIntersectionSpeed5_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileF(fnameF);
                outputfileF << basicSafetyMessageInfo.numberPacketSendSpeedInter[5] << std::endl;
                outputfileF.close();

                char fname8[30];
                sprintf(fname8,"tmpSendIntersection_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfile8(fname8);
                outputfile8 << basicSafetyMessageInfo.numberPacketSendinintersection << std::endl;
                outputfile8.close();
            //}

        }else if((abs(abs(currentX) - intersectionPos) <= intersectionRange) && (abs(currentY) <= intersectionRange)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "X = " << currentX << endl;
            //std::cout << "Y = " << currentY << endl;
            
            intersectionflag = true;

        }else if((abs(abs(currentY) - intersectionPos) <= intersectionRange) && (abs(currentX) <= intersectionRange)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "id = " << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "X = " << currentX << endl;
            //std::cout << "Y = " << currentY << endl;

            intersectionflag = true;

        }else if((abs(abs(currentX) - intersectionPos) <= intersectionRange) && (abs(abs(currentY) - intersectionPos) <= intersectionRange)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;

            intersectionflag = true;

        }else if((abs(abs(currentX) - (intersectionPos * 2)) <= intersectionRange) && (abs(currentY) <= intersectionRange)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;

            intersectionflag = true;

        }else if((abs(abs(currentY) - (intersectionPos * 2)) <= intersectionRange) && (abs(currentX) <= intersectionRange)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;

            intersectionflag = true;

        }else if((abs(abs(currentX) - (intersectionPos * 2)) <= intersectionRange) && (abs(abs(currentY) - intersectionPos) <= intersectionRange)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;

            intersectionflag = true;

        }else if((abs(abs(currentY) - (intersectionPos * 2)) <= intersectionRange) && (abs(abs(currentX) - intersectionPos) <= intersectionRange)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;

            intersectionflag = true;
            
        }else{

        }

        if(intersectionflag == true){
            /*per[0] = 4500;
            per[1] = 3000;
            per[2] = 1500;
            per[3] = 1000;*/

            //double backoffper[4];
            //backoffper[0] = per[0] / (per[0] + per[1] + per[2] + per[3]);
            //backoffper[1] = per[1] / (per[0] + per[1] + per[2] + per[3]);
            backoffper[2] = per[2] / (per[2] + per[3]);
            backoffper[3] = per[3] / (per[2] + per[3]);

            //int nearper[4];
            //切り上げ
            //nearper[0] = std::ceil(basicSafetyMessageInfo.totalinter * backoffper[3]);
            nearper[1] = std::ceil(basicSafetyMessageInfo.totalinter * backoffper[2]);
            //nearper[2] = std::ceil(basicSafetyMessageInfo.totalnear * backoffper[1]);
            //nearper[3] = std::ceil(basicSafetyMessageInfo.totalnear * backoffper[0]);

            //切り捨て
            nearper[0] = basicSafetyMessageInfo.totalinter * backoffper[3];
            //nearper[1] = basicSafetyMessageInfo.totalinter * backoffper[2];
            //nearper[2] = basicSafetyMessageInfo.totalinter * backoffper[1];
            //nearper[3] = basicSafetyMessageInfo.totalinter * backoffper[0];

            unsigned int sorted2[1600];
            unsigned int currentSpeedPos2;
            for(int i = 0; i < 1600; i++){
                sorted2[i] = 0;
            }

            for(int i = 0; i < 1600; i++){
                if((i + 1) == basicSafetyMessageInfo.MyNodeId){
                    sorted2[i] = currentSpeed;
                }else if(basicSafetyMessageInfo.list[i].timetointersection == 0 && basicSafetyMessageInfo.list[i].flag == true){
                    sorted2[i] = basicSafetyMessageInfo.list[i].speed;
                } 
            }
            std::sort(sorted2, sorted2 + SIZE_OF_ARRAY(sorted2), std::greater<unsigned int>());
            for(int i = 1599; i >= 0; i--){
                if(sorted2[i] == currentSpeed){
                    currentSpeedPos2 = i;
                    //std::cout << nearper[0] << endl;
                    if(currentSpeedPos2 <= nearper[0]){
                        basicSafetyMessageInfo.priority = 6;
                    /*}else if(currentSpeedPos2 <= (nearper[0] + nearper[1])){
                        basicSafetyMessageInfo.priority = 4;
                    }else if(currentSpeedPos2 <= (nearper[0] + nearper[1] + nearper[2])){
                        basicSafetyMessageInfo.priority = 0;*/
                    }else{
                        basicSafetyMessageInfo.priority = 4;
                    }
                    break;
                }
            }
        }

       /*if(intersectionflag == true && nearper[0] > basicSafetyMessageInfo.sevencnt){

            basicSafetyMessageInfo.priority = 6;

        }else if(intersectionflag == true && nearper[0] <= basicSafetyMessageInfo.sevencnt){
            for(int i = 0; i < 1600; i++){
                if(basicSafetyMessageInfo.list[i].timetointersection == 0){
                    sorted2[i] = basicSafetyMessageInfo.list[i].speed;
                }else if((i + 1) == basicSafetyMessageInfo.MyNodeId){
                    sorted2[i] = currentSpeed;
                }
            }
            std::sort(sorted2, sorted2 + SIZE_OF_ARRAY(sorted2), std::greater<unsigned int>());
            for(int i = 1599; i >= 0; i--){
                if(sorted2[i] == currentSpeed){
                    currentSpeedPos2 = i;
                    if(currentSpeedPos2 <= nearper[0]){
                        basicSafetyMessageInfo.priority = 6;
                        //basicSafetyMessageInfo.priorityininter[3]++;
                        break;
                    }else{
                        basicSafetyMessageInfo.priority = 4;
                        //basicSafetyMessageInfo.priorityininter[2]++;
                        break;
                    }
                }
            }

            
        }else{

        }*/

        if(intersectionflag == true){
            char fname18[30];
            sprintf(fname18,"tmpIntersection_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile18(fname18);
            outputfile18 << 1 << std::endl;
            outputfile18.close();

        }else{
            char fname19[30];
            sprintf(fname19,"tmpIntersection_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile19(fname19);
            outputfile19 << 0 << std::endl;
            outputfile19.close();
        }

        //パターン2
        /*if(intersectiontime == 0){
            basicSafetyMessageInfo.priority = 7;
        }*/
        //パターン2
    }else{

        
        const int intersectionPos2 = 200000;
        const int intersectionRange2 = 10000;

        if((abs(currentX) <= 10000) && (abs(currentY) <= 10000)){
        //受信車両50のみ
        //if((abs(currentX) <= 10000) && (0 <= currentY) && (currentY <= 10000)){
            //std::cout << "id = " << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "X = " << currentX << endl;
            //std::cout << "Y = " << currentY << endl;

            basicSafetyMessageInfo.numberPacketSendinintersection++;

                if(0 <= currentSpeed && currentSpeed <= 2777){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[0]++;
                }else if(2777 < currentSpeed  && currentSpeed <= 5555){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[1]++;
                }else if(5555 < currentSpeed && currentSpeed <= 8333){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[2]++;
                }else if(8333 < currentSpeed && currentSpeed <= 11111){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[3]++;
                }else if(11111 < currentSpeed && currentSpeed <= 13888){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[4]++;
                }else if(13888 < currentSpeed && currentSpeed <= 16667){
                    basicSafetyMessageInfo.numberPacketSendSpeedInter[5]++;
                }

            intersectionflag = true;

                

        }else if((abs(abs(currentX) - intersectionPos2) <= intersectionRange2) && (abs(currentY) <= intersectionRange2)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "X = " << currentX << endl;
            //std::cout << "Y = " << currentY << endl;
            
            intersectionflag = true;

        }else if((abs(abs(currentY) - intersectionPos2) <= intersectionRange2) && (abs(currentX) <= intersectionRange2)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "id = " << basicSafetyMessageInfo.MyNodeId << endl;
            //std::cout << "X = " << currentX << endl;
            //std::cout << "Y = " << currentY << endl;

            intersectionflag = true;

        }else if((abs(abs(currentX) - intersectionPos2) <= intersectionRange2) && (abs(abs(currentY) - intersectionPos2) <= intersectionRange2)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;

            intersectionflag = true;

        }else if((abs(abs(currentX) - (intersectionPos2 * 2)) <= intersectionRange2) && (abs(currentY) <= intersectionRange2)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;

            intersectionflag = true;

        }else if((abs(abs(currentY) - (intersectionPos2 * 2)) <= intersectionRange2) && (abs(currentX) <= intersectionRange2)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;

            intersectionflag = true;

        }else if((abs(abs(currentX) - (intersectionPos2 * 2)) <= intersectionRange2) && (abs(abs(currentY) - intersectionPos2) <= intersectionRange2)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;

            intersectionflag = true;

        }else if((abs(abs(currentY) - (intersectionPos2 * 2)) <= intersectionRange2) && (abs(abs(currentX) - intersectionPos2) <= intersectionRange2)){
            //basicSafetyMessageInfo.priority = 6;
            //basicSafetyMessageInfo.priority = 4;
            //std::cout << "intersection now" << basicSafetyMessageInfo.MyNodeId << endl;

            intersectionflag = true;
            
        }else{

        }
                char fnameG[30];
                sprintf(fnameG,"tmpSendIntersectionSpeed0_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileG(fnameG);
                outputfileG << basicSafetyMessageInfo.numberPacketSendSpeedInter[0] << std::endl;
                outputfileG.close();

                char fnameH[30];
                sprintf(fnameH,"tmpSendIntersectionSpeed1_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileH(fnameH);
                outputfileH << basicSafetyMessageInfo.numberPacketSendSpeedInter[1] << std::endl;
                outputfileH.close();

                char fnameI[30];
                sprintf(fnameI,"tmpSendIntersectionSpeed2_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileI(fnameI);
                outputfileI << basicSafetyMessageInfo.numberPacketSendSpeedInter[2] << std::endl;
                outputfileI.close();

                char fnameJ[30];
                sprintf(fnameJ,"tmpSendIntersectionSpeed3_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileJ(fnameJ);
                outputfileJ << basicSafetyMessageInfo.numberPacketSendSpeedInter[3] << std::endl;
                outputfileJ.close();

                char fnameK[30];
                sprintf(fnameK,"tmpSendIntersectionSpeed4_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileK(fnameK);
                outputfileK << basicSafetyMessageInfo.numberPacketSendSpeedInter[4] << std::endl;
                outputfileK.close();

                char fnameL[30];
                sprintf(fnameL,"tmpSendIntersectionSpeed5_%d.txt",basicSafetyMessageInfo.MyNodeId);
                std::ofstream outputfileL(fnameL);
                outputfileL << basicSafetyMessageInfo.numberPacketSendSpeedInter[5] << std::endl;
                outputfileL.close();

            char fname34[30];
            sprintf(fname34,"tmpSendIntersection_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile34(fname34);
            outputfile34 << basicSafetyMessageInfo.numberPacketSendinintersection << std::endl;
            outputfile34.close();

        basicSafetyMessageInfo.numberPacketSend++;
        basicSafetyMessageInfo.sumnear += basicSafetyMessageInfo.totalnear;

        if(basicSafetyMessageInfo.MyNodeId == 1){
            //std::cout << "totalnear" << basicSafetyMessageInfo.MyNodeId << " = " << basicSafetyMessageInfo.totalnear << endl;
            for(int i = 0; i < 4; i++){
                std::cout << "nearvehi" << i << " = " << basicSafetyMessageInfo.nearvehi[i] << endl;
                basicSafetyMessageInfo.sumvehi[i] += basicSafetyMessageInfo.nearvehi[i];
                std::cout << "avevehi" << i << " = " << basicSafetyMessageInfo.sumvehi[i]/basicSafetyMessageInfo.numberPacketSend << endl;
            }
            std::cout << "nearave" << basicSafetyMessageInfo.MyNodeId << " = "<< basicSafetyMessageInfo.sumnear / basicSafetyMessageInfo.numberPacketSend << endl;
            std::cout << "nearmax" << basicSafetyMessageInfo.MyNodeId << " = " << basicSafetyMessageInfo.totalmax << endl;
            
            //std::cout << LLONG_MAX + 1 << endl;

            //読み取り
            /*char fname[30];
            std::string destPriorityS;
            unsigned int destPriority;
            unsigned int cnt[4];
            for(int i = 1; i <= 1440; i++){
                sprintf(fname,"tmpPri_%d.txt",i);
                std::ifstream inputfile(fname);
                inputfile >> destPriorityS;
                destPriority = atoi(destPriorityS.c_str());

                if(destPriority == 1 || destPriority == 2){
                    cnt[0]++;
                }else if(destPriority == 0 || destPriority == 3){
                    cnt[1]++;
                }else if(destPriority == 4 || destPriority == 5){
                    cnt[2]++;
                }else{
                    cnt[3]++;
                }
            }

            for(int i = 0; i < 4; i++){
                std::cout << "Pri" << i << " = " << cnt[i] << endl;
            }*/


    
        }
    }//if
    

    
    assert(basicSafetyMessageInfo.extendedPayloadSizeBytes >=
           sizeof(DsrcBasicSafetyMessagePart1Type));

    //std::maxは最大値,設定サイズがペイロードを下回っていた時にpart2,つまり追加分のデータを0byteにしている
    size_t part2PayloadSize = std::max<size_t>(0, basicSafetyMessageInfo.extendedPayloadSizeBytes - sizeof(DsrcBasicSafetyMessagePart1Type));
    //part2Payloadはtype1に加えて規定サイズに増やすためのもの
    unsigned char* part2Payload = new unsigned char[part2PayloadSize];
    /*if(basicSafetyMessageInfo.MyNodeId == 2){
        std::cout << "現在のスピード=" << currentSpeed << endl;
    }*/
    //表示成功
    //std::cout << "Currenttime = " << currentTime << endl;
    //ここで優先度分けかなぁ

    //良さそう
    //if(basicSafetyMessageInfo.MyNodeId == 1){
        //std::cout << "currentPriority" << basicSafetyMessageInfo.MyNodeId << " = " << (int)basicSafetyMessageInfo.priority << endl;
        //std::cout << "currentX = " << currentX << endl;
        //std::cout << "currentY = " << currentY << endl;
    //}
    //if(intersectionflag == true){
    if((abs(currentX) <= 10000) && (abs(currentY) <= 10000)){
        if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
            basicSafetyMessageInfo.priorityininter[0]++;
        }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
            basicSafetyMessageInfo.priorityininter[1]++;
        }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
            basicSafetyMessageInfo.priorityininter[2]++;
        }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
            basicSafetyMessageInfo.priorityininter[3]++;
        }
        char fnameAA[30];
        sprintf(fnameAA,"tmpPriIntersection0_%d.txt",basicSafetyMessageInfo.MyNodeId);
        std::ofstream outputfileAA(fnameAA);
        outputfileAA << basicSafetyMessageInfo.priorityininter[0] << std::endl;
        outputfileAA.close();

        char fnameAB[30];
        sprintf(fnameAB,"tmpPriIntersection1_%d.txt",basicSafetyMessageInfo.MyNodeId);
        std::ofstream outputfileAB(fnameAB);
        outputfileAB << basicSafetyMessageInfo.priorityininter[1] << std::endl;
        outputfileAB.close();

        char fnameAC[30];
        sprintf(fnameAC,"tmpPriIntersection2_%d.txt",basicSafetyMessageInfo.MyNodeId);
        std::ofstream outputfileAC(fnameAC);
        outputfileAC << basicSafetyMessageInfo.priorityininter[2] << std::endl;
        outputfileAC.close();

        char fnameAD[30];
        sprintf(fnameAD,"tmpPriIntersection3_%d.txt",basicSafetyMessageInfo.MyNodeId);
        std::ofstream outputfileAD(fnameAD);
        outputfileAD << basicSafetyMessageInfo.priorityininter[3] << std::endl;
        outputfileAD.close();
    }

    if(intersectionflag == true){
        if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
            basicSafetyMessageInfo.priorityininterall[0]++;
        }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
            basicSafetyMessageInfo.priorityininterall[1]++;
        }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
            basicSafetyMessageInfo.priorityininterall[2]++;

        }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
            basicSafetyMessageInfo.priorityininterall[3]++;
        }
        char fnameCA[30];
        sprintf(fnameCA,"tmpPriIntersectionA0_%d.txt",basicSafetyMessageInfo.MyNodeId);
        std::ofstream outputfileCA(fnameCA);
        outputfileCA << basicSafetyMessageInfo.priorityininterall[0] << std::endl;
        outputfileCA.close();

        char fnameCB[30];
        sprintf(fnameCB,"tmpPriIntersectionA1_%d.txt",basicSafetyMessageInfo.MyNodeId);
        std::ofstream outputfileCB(fnameCB);
        outputfileCB << basicSafetyMessageInfo.priorityininterall[1] << std::endl;
        outputfileCB.close();

        char fnameCC[30];
        sprintf(fnameCC,"tmpPriIntersectionA2_%d.txt",basicSafetyMessageInfo.MyNodeId);
        std::ofstream outputfileCC(fnameCC);
        outputfileCC << basicSafetyMessageInfo.priorityininterall[2] << std::endl;
        outputfileCC.close();

        char fnameCD[30];
        sprintf(fnameCD,"tmpPriIntersectionA3_%d.txt",basicSafetyMessageInfo.MyNodeId);
        std::ofstream outputfileCD(fnameCD);
        outputfileCD << basicSafetyMessageInfo.priorityininterall[3] << std::endl;
        outputfileCD.close();
    }

    if(intersectionflag != true){
        if(0 <= currentSpeed && currentSpeed <= 2777){

        }else if(2777 < currentSpeed  && currentSpeed <= 5555){

        }else if(5555 < currentSpeed && currentSpeed <= 8333){

        }else if(8333 < currentSpeed && currentSpeed <= 11111){

            if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                basicSafetyMessageInfo.priorityin30_40all[0]++;
            }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                basicSafetyMessageInfo.priorityin30_40all[1]++;
            }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                basicSafetyMessageInfo.priorityin30_40all[2]++;
            }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                basicSafetyMessageInfo.priorityin30_40all[3]++;
            }
        }else if(11111 < currentSpeed && currentSpeed <= 13888){
            if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                basicSafetyMessageInfo.priorityin40_50all[0]++;
            }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                basicSafetyMessageInfo.priorityin40_50all[1]++;
            }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                basicSafetyMessageInfo.priorityin40_50all[2]++;
            }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                basicSafetyMessageInfo.priorityin40_50all[3]++;
            }
        }else if(13888 < currentSpeed && currentSpeed <= 16667){
            if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                basicSafetyMessageInfo.priorityin50_60all[0]++;
            }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                basicSafetyMessageInfo.priorityin50_60all[1]++;
            }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                basicSafetyMessageInfo.priorityin50_60all[2]++;
            }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                basicSafetyMessageInfo.priorityin50_60all[3]++;
            }
        }
    }     
            

    if(intersectionflag != true){
        if(((-10000 <= currentX) && (currentX <= 10000) && (-50000 <= currentY) && (currentY <= 0)) || ((-10000 <= currentX) && (currentX <= 10000) && (0 <= currentY) && (currentY <= 50000))){
        //if(((-10000 <= currentX) && (currentX <= 10000) && (50000 <= currentY) && (currentY <= 100000)) || ((-10000 <= currentX) && (currentX <= 10000) && (100000 <= currentY) && (currentY <= 150000))){
        //if(((-10000 <= currentX) && (currentX <= 10000) && (-10000 <= currentY) && (currentY <= 40000)) || ((-10000 <= currentX) && (currentX <= 10000) && (40000 <= currentY) && (currentY <= 90000))){
        //if(((-10000 <= currentY) && (currentY <= 10000) && (50000 <= currentX) && (currentX <= 100000)) || ((-10000 <= currentY) && (currentY <= 10000) && (100000 <= currentX) && (currentX <= 150000))){
        //if(((-10000 <= currentX) && (currentX <= 10000) && (-50000 >= currentY) && (currentY >= -100000)) || ((-10000 <= currentX) && (currentX <= 10000) && (-100000 >= currentY) && (currentY >= -150000))){
        //if(((-10000 <= currentY) && (currentY <= 10000) && (-50000 >= currentX) && (currentX >= -100000)) || ((-10000 <= currentY) && (currentY <= 10000) && (-100000 >= currentX) && (currentX >= -150000))){
            if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                basicSafetyMessageInfo.numberPacketSendPCR[0]++;
            }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                basicSafetyMessageInfo.numberPacketSendPCR[1]++;
            }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                basicSafetyMessageInfo.numberPacketSendPCR[2]++;
            }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                basicSafetyMessageInfo.numberPacketSendPCR[3]++;
            }


            //if(intersectionflag == false){
                /*if(2777 <= currentSpeed  && currentSpeed <= 5555){
                    basicSafetyMessageInfo.numberPacketSendSpeed[0]++;
                }else if(5555 < currentSpeed && currentSpeed <= 8333){
                    basicSafetyMessageInfo.numberPacketSendSpeed[1]++;
                }else if(8333 < currentSpeed && currentSpeed <= 11111){
                    basicSafetyMessageInfo.numberPacketSendSpeed[2]++;
                }else if(11111 < currentSpeed && currentSpeed <= 13888){
                    basicSafetyMessageInfo.numberPacketSendSpeed[3]++;
                }else if(13888 < currentSpeed && currentSpeed <= 16667){
                    basicSafetyMessageInfo.numberPacketSendSpeed[4]++;
                }*/
            //}

                if(0 <= currentSpeed && currentSpeed <= 2777){
                    basicSafetyMessageInfo.numberPacketSendSpeed[0]++;
                }else if(2777 < currentSpeed  && currentSpeed <= 5555){
                    basicSafetyMessageInfo.numberPacketSendSpeed[1]++;
                    if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                        basicSafetyMessageInfo.priorityin30_40[0]++;
                    }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                        basicSafetyMessageInfo.priorityin30_40[1]++;
                    }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                        basicSafetyMessageInfo.priorityin30_40[2]++;
                    }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                        basicSafetyMessageInfo.priorityin30_40[3]++;
                    }
                }else if(5555 < currentSpeed && currentSpeed <= 8333){
                    basicSafetyMessageInfo.numberPacketSendSpeed[2]++;
                    if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                        basicSafetyMessageInfo.priorityin40_50[0]++;
                    }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                        basicSafetyMessageInfo.priorityin40_50[1]++;
                    }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                        basicSafetyMessageInfo.priorityin40_50[2]++;
                    }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                        basicSafetyMessageInfo.priorityin40_50[3]++;
                    }
                }else if(8333 < currentSpeed && currentSpeed <= 11111){
                    basicSafetyMessageInfo.numberPacketSendSpeed[3]++;
                    /*if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                        basicSafetyMessageInfo.priorityin30_40[0]++;
                    }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                        basicSafetyMessageInfo.priorityin30_40[1]++;
                    }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                        basicSafetyMessageInfo.priorityin30_40[2]++;
                    }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                        basicSafetyMessageInfo.priorityin30_40[3]++;
                    }*/
                }else if(11111 < currentSpeed && currentSpeed <= 13888){
                    basicSafetyMessageInfo.numberPacketSendSpeed[4]++;
                    /*if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                        basicSafetyMessageInfo.priorityin40_50[0]++;
                    }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                        basicSafetyMessageInfo.priorityin40_50[1]++;
                    }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                        basicSafetyMessageInfo.priorityin40_50[2]++;
                    }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                        basicSafetyMessageInfo.priorityin40_50[3]++;
                    }*/
                }else if(13888 < currentSpeed && currentSpeed <= 16667){
                    basicSafetyMessageInfo.numberPacketSendSpeed[5]++;
                    if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                        basicSafetyMessageInfo.priorityin50_60[0]++;
                    }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                        basicSafetyMessageInfo.priorityin50_60[1]++;
                    }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                        basicSafetyMessageInfo.priorityin50_60[2]++;
                    }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                        basicSafetyMessageInfo.priorityin50_60[3]++;
                    }
                }

            char fname9[30];
            sprintf(fname9,"tmpnumberPCR0_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile9(fname9);
            outputfile9 << basicSafetyMessageInfo.numberPacketSendPCR[0] << std::endl;
            outputfile9.close();


            char fname10[30];
            sprintf(fname10,"tmpnumberPCR1_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile10(fname10);
            outputfile10 << basicSafetyMessageInfo.numberPacketSendPCR[1] << std::endl;
            outputfile10.close();

            char fname11[30];
            sprintf(fname11,"tmpnumberPCR2_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile11(fname11);
            outputfile11 << basicSafetyMessageInfo.numberPacketSendPCR[2] << std::endl;
            outputfile11.close();

            char fname12[30];
            sprintf(fname12,"tmpnumberPCR3_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile12(fname12);
            outputfile12 << basicSafetyMessageInfo.numberPacketSendPCR[3] << std::endl;
            outputfile12.close();

            char fname35[30];
            sprintf(fname35,"tmpnumberSpeed0_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile35(fname35);
            outputfile35 << basicSafetyMessageInfo.numberPacketSendSpeed[0] << std::endl;
            outputfile35.close();

            char fname36[30];
            sprintf(fname36,"tmpnumberSpeed1_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile36(fname36);
            outputfile36 << basicSafetyMessageInfo.numberPacketSendSpeed[1] << std::endl;
            outputfile36.close();

            char fname37[30];
            sprintf(fname37,"tmpnumberSpeed2_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile37(fname37);
            outputfile37 << basicSafetyMessageInfo.numberPacketSendSpeed[2] << std::endl;
            outputfile37.close();

            char fname38[30];
            sprintf(fname38,"tmpnumberSpeed3_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile38(fname38);
            outputfile38 << basicSafetyMessageInfo.numberPacketSendSpeed[3] << std::endl;
            outputfile38.close();

            char fname39[30];
            sprintf(fname39,"tmpnumberSpeed4_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile39(fname39);
            outputfile39 << basicSafetyMessageInfo.numberPacketSendSpeed[4] << std::endl;
            outputfile39.close();

            char fnameEx3[30];
            sprintf(fnameEx3,"tmpnumberSpeed5_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfileEx3(fnameEx3);
            outputfileEx3 << basicSafetyMessageInfo.numberPacketSendSpeed[5] << std::endl;
            outputfileEx3.close();

            //if(basicSafetyMessageInfo.MyNodeId == 1){
                //std::cout << "Send From " << basicSafetyMessageInfo.MyNodeId << " = " << basicSafetyMessageInfo.numberPacketSendPCR << endl;
            //}
        //受信車両交差点の時のみ
        }else if(((-10000 <= currentY) && (currentY <= 10000) && (-50000 <= currentX) && (currentX <= 0)) || ((-10000 <= currentY) && (currentY <= 10000) && (0 <= currentX) && (currentX <= 50000))){

            if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                basicSafetyMessageInfo.numberPacketSendPCR[0]++;
            }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                basicSafetyMessageInfo.numberPacketSendPCR[1]++;
            }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                basicSafetyMessageInfo.numberPacketSendPCR[2]++;
            }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                basicSafetyMessageInfo.numberPacketSendPCR[3]++;
            }

            //if(intersectionflag == false){
                /*if(2777 <= currentSpeed && currentSpeed <= 5555){
                    basicSafetyMessageInfo.numberPacketSendSpeed[0]++;
                }else if(5555 < currentSpeed && currentSpeed <= 8333){
                    basicSafetyMessageInfo.numberPacketSendSpeed[1]++;
                }else if(8333 < currentSpeed && currentSpeed <= 11111){
                    basicSafetyMessageInfo.numberPacketSendSpeed[2]++;
                }else if(11111 < currentSpeed && currentSpeed <= 13888){
                    basicSafetyMessageInfo.numberPacketSendSpeed[3]++;
                }else if(13888 < currentSpeed && currentSpeed <= 16667){
                    basicSafetyMessageInfo.numberPacketSendSpeed[4]++;
                }*/
            //}
                //ここから
                if(0 <= currentSpeed && currentSpeed <= 2777){
                    basicSafetyMessageInfo.numberPacketSendSpeed[0]++;
                }else if(2777 < currentSpeed  && currentSpeed <= 5555){
                    basicSafetyMessageInfo.numberPacketSendSpeed[1]++;
                }else if(5555 < currentSpeed && currentSpeed <= 8333){
                    basicSafetyMessageInfo.numberPacketSendSpeed[2]++;
                }else if(8333 < currentSpeed && currentSpeed <= 11111){
                    basicSafetyMessageInfo.numberPacketSendSpeed[3]++;
                    if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                        basicSafetyMessageInfo.priorityin30_40[0]++;
                    }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                        basicSafetyMessageInfo.priorityin30_40[1]++;
                    }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                        basicSafetyMessageInfo.priorityin30_40[2]++;
                    }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                        basicSafetyMessageInfo.priorityin30_40[3]++;
                    }
                }else if(11111 < currentSpeed && currentSpeed <= 13888){
                    basicSafetyMessageInfo.numberPacketSendSpeed[4]++;
                    if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                        basicSafetyMessageInfo.priorityin40_50[0]++;
                    }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                        basicSafetyMessageInfo.priorityin40_50[1]++;
                    }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                        basicSafetyMessageInfo.priorityin40_50[2]++;
                    }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                        basicSafetyMessageInfo.priorityin40_50[3]++;
                    }
                }else if(13888 < currentSpeed && currentSpeed <= 16667){
                    basicSafetyMessageInfo.numberPacketSendSpeed[5]++;
                    if(basicSafetyMessageInfo.priority == 1 || basicSafetyMessageInfo.priority == 2){
                        basicSafetyMessageInfo.priorityin50_60[0]++;
                    }else if(basicSafetyMessageInfo.priority == 0 || basicSafetyMessageInfo.priority == 3){
                        basicSafetyMessageInfo.priorityin50_60[1]++;
                    }else if(basicSafetyMessageInfo.priority == 4 || basicSafetyMessageInfo.priority == 5){
                        basicSafetyMessageInfo.priorityin50_60[2]++;
                    }else if(basicSafetyMessageInfo.priority == 6 || basicSafetyMessageInfo.priority == 7){
                        basicSafetyMessageInfo.priorityin50_60[3]++;
                    }
                }

            char fname13[30];
            sprintf(fname13,"tmpnumberPCR0_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile13(fname13);
            outputfile13 << basicSafetyMessageInfo.numberPacketSendPCR[0] << std::endl;
            outputfile13.close();

            char fname14[30];
            sprintf(fname14,"tmpnumberPCR1_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile14(fname14);
            outputfile14 << basicSafetyMessageInfo.numberPacketSendPCR[1] << std::endl;
            outputfile14.close();

            char fname15[30];
            sprintf(fname15,"tmpnumberPCR2_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile15(fname15);
            outputfile15 << basicSafetyMessageInfo.numberPacketSendPCR[2] << std::endl;
            outputfile15.close();

            char fname16[30];
            sprintf(fname16,"tmpnumberPCR3_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile16(fname16);
            outputfile16 << basicSafetyMessageInfo.numberPacketSendPCR[3] << std::endl;
            outputfile16.close();

            char fname38[30];
            sprintf(fname38,"tmpnumberSpeed0_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile38(fname38);
            outputfile38 << basicSafetyMessageInfo.numberPacketSendSpeed[0] << std::endl;
            outputfile38.close();

            char fname39[30];
            sprintf(fname39,"tmpnumberSpeed1_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile39(fname39);
            outputfile39 << basicSafetyMessageInfo.numberPacketSendSpeed[1] << std::endl;
            outputfile39.close();

            char fname40[30];
            sprintf(fname40,"tmpnumberSpeed2_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile40(fname40);
            outputfile40 << basicSafetyMessageInfo.numberPacketSendSpeed[2] << std::endl;
            outputfile40.close();

            char fname41[30];
            sprintf(fname41,"tmpnumberSpeed3_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile41(fname41);
            outputfile41 << basicSafetyMessageInfo.numberPacketSendSpeed[3] << std::endl;
            outputfile41.close();

            char fname42[30];
            sprintf(fname42,"tmpnumberSpeed4_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile42(fname42);
            outputfile42 << basicSafetyMessageInfo.numberPacketSendSpeed[4] << std::endl;
            outputfile42.close();

            char fnameEx[30];
            sprintf(fnameEx,"tmpnumberSpeed5_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfileEx(fnameEx);
            outputfileEx << basicSafetyMessageInfo.numberPacketSendSpeed[5] << std::endl;
            outputfileEx.close();

            //if(basicSafetyMessageInfo.MyNodeId == 1){
                //std::cout << "Send From " << basicSafetyMessageInfo.MyNodeId << " = " << basicSafetyMessageInfo.numberPacketSendPCR << endl;
            //}*/
        }
    }

    char fnameAE[30];
    sprintf(fnameAE,"tmpPriSpeed3_0_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAE(fnameAE);
    outputfileAE << basicSafetyMessageInfo.priorityin30_40[0] << std::endl;
    outputfileAE.close();

    char fnameAF[30];
    sprintf(fnameAF,"tmpPriSpeed3_1_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAF(fnameAF);
    outputfileAF << basicSafetyMessageInfo.priorityin30_40[1] << std::endl;
    outputfileAF.close();

    char fnameAG[30];
    sprintf(fnameAG,"tmpPriSpeed3_2_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAG(fnameAG);
    outputfileAG << basicSafetyMessageInfo.priorityin30_40[2] << std::endl;
    outputfileAG.close();

    char fnameAH[30];
    sprintf(fnameAH,"tmpPriSpeed3_3_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAH(fnameAH);
    outputfileAH << basicSafetyMessageInfo.priorityin30_40[3] << std::endl;
    outputfileAH.close();

    char fnameAI[30];
    sprintf(fnameAI,"tmpPriSpeed4_0_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAI(fnameAI);
    outputfileAI << basicSafetyMessageInfo.priorityin40_50[0] << std::endl;
    outputfileAI.close();

    char fnameAJ[30];
    sprintf(fnameAJ,"tmpPriSpeed4_1_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAJ(fnameAJ);
    outputfileAJ << basicSafetyMessageInfo.priorityin40_50[1] << std::endl;
    outputfileAJ.close();

    char fnameAK[30];
    sprintf(fnameAK,"tmpPriSpeed4_2_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAK(fnameAK);
    outputfileAK << basicSafetyMessageInfo.priorityin40_50[2] << std::endl;
    outputfileAK.close();

    char fnameAL[30];
    sprintf(fnameAL,"tmpPriSpeed4_3_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAL(fnameAL);
    outputfileAL << basicSafetyMessageInfo.priorityin40_50[3] << std::endl;
    outputfileAL.close();

    char fnameAM[30];
    sprintf(fnameAM,"tmpPriSpeed5_0_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAM(fnameAM);
    outputfileAM << basicSafetyMessageInfo.priorityin50_60[0] << std::endl;
    outputfileAM.close();

    char fnameAN[30];
    sprintf(fnameAN,"tmpPriSpeed5_1_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAN(fnameAN);
    outputfileAN << basicSafetyMessageInfo.priorityin50_60[1] << std::endl;
    outputfileAN.close();

    char fnameAO[30];
    sprintf(fnameAO,"tmpPriSpeed5_2_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAO(fnameAO);
    outputfileAO << basicSafetyMessageInfo.priorityin50_60[2] << std::endl;
    outputfileAO.close();

    char fnameAP[30];
    sprintf(fnameAP,"tmpPriSpeed5_3_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileAP(fnameAP);
    outputfileAP << basicSafetyMessageInfo.priorityin50_60[3] << std::endl;
    outputfileAP.close();


    char fnameDE[30];
    sprintf(fnameDE,"tmpPriSpeedA3_0_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDE(fnameDE);
    outputfileDE << basicSafetyMessageInfo.priorityin30_40all[0] << std::endl;
    outputfileDE.close();

    char fnameDF[30];
    sprintf(fnameDF,"tmpPriSpeedA3_1_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDF(fnameDF);
    outputfileDF << basicSafetyMessageInfo.priorityin30_40all[1] << std::endl;
    outputfileDF.close();

    char fnameDG[30];
    sprintf(fnameDG,"tmpPriSpeedA3_2_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDG(fnameDG);
    outputfileDG << basicSafetyMessageInfo.priorityin30_40all[2] << std::endl;
    outputfileDG.close();

    char fnameDH[30];
    sprintf(fnameDH,"tmpPriSpeedA3_3_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDH(fnameDH);
    outputfileDH << basicSafetyMessageInfo.priorityin30_40all[3] << std::endl;
    outputfileDH.close();

    char fnameDI[30];
    sprintf(fnameDI,"tmpPriSpeedA4_0_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDI(fnameDI);
    outputfileDI << basicSafetyMessageInfo.priorityin40_50all[0] << std::endl;
    outputfileDI.close();

    char fnameDJ[30];
    sprintf(fnameDJ,"tmpPriSpeedA4_1_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDJ(fnameDJ);
    outputfileDJ << basicSafetyMessageInfo.priorityin40_50all[1] << std::endl;
    outputfileDJ.close();

    char fnameDK[30];
    sprintf(fnameDK,"tmpPriSpeedA4_2_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDK(fnameDK);
    outputfileDK << basicSafetyMessageInfo.priorityin40_50all[2] << std::endl;
    outputfileDK.close();

    char fnameDL[30];
    sprintf(fnameDL,"tmpPriSpeedA4_3_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDL(fnameDL);
    outputfileDL << basicSafetyMessageInfo.priorityin40_50all[3] << std::endl;
    outputfileDL.close();

    char fnameDM[30];
    sprintf(fnameDM,"tmpPriSpeedA5_0_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDM(fnameDM);
    outputfileDM << basicSafetyMessageInfo.priorityin50_60all[0] << std::endl;
    outputfileDM.close();

    char fnameDN[30];
    sprintf(fnameDN,"tmpPriSpeedA5_1_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDN(fnameDN);
    outputfileDN << basicSafetyMessageInfo.priorityin50_60all[1] << std::endl;
    outputfileDN.close();

    char fnameDO[30];
    sprintf(fnameDO,"tmpPriSpeedA5_2_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDO(fnameDO);
    outputfileDO << basicSafetyMessageInfo.priorityin50_60all[2] << std::endl;
    outputfileDO.close();

    char fnameDP[30];
    sprintf(fnameDP,"tmpPriSpeedA5_3_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfileDP(fnameDP);
    outputfileDP << basicSafetyMessageInfo.priorityin50_60all[3] << std::endl;
    outputfileDP.close();



    //if(basicSafetyMessageInfo.MyNodeId == 801){
    if(basicSafetyMessageInfo.MyNodeId == 1601){
        std::cout << "message = " << basicSafetyMessageInfo.numberPacketSend << endl;
    }
    if(basicSafetyMessageInfo.numberPacketSend == 1000){
        //if(basicSafetyMessageInfo.MyNodeId == 801){
        if(basicSafetyMessageInfo.MyNodeId == 1601){
            unsigned int sumR = 0;
            unsigned int sumS = 0;
            unsigned int sumSInter = 0;
            unsigned int sum1 = 0;
            unsigned int sumPR[4];
            unsigned int sumPS[4];
            //unsigned int sumSR[5];
            //unsigned int sumSS[5];
            unsigned int sumSR[6];
            unsigned int sumSS[6];
            unsigned int sumSRI[6];
            unsigned int sumSSI[6];
            unsigned int sumSRT[6];
            unsigned int sumSST[6];
            unsigned int sumSIP[4];
            unsigned int sumRIP[4];
            unsigned int sumSSP3[4];
            unsigned int sumSSP4[4];
            unsigned int sumSSP5[4];
            unsigned int sumSIPA[4];
            unsigned int sumSSP3A[4];
            unsigned int sumSSP4A[4];
            unsigned int sumSSP5A[4];
            unsigned int delaydistributiontotal = 0;
            unsigned int delaydistributiontotalS[6];
            unsigned int delayaveT[6];
            unsigned int delaynotinter = 0;
            unsigned int pricnt[4];
            unsigned int pricntI[4];
            unsigned int sumS1 = 0;
            for(int i = 0; i < 4; i++){
                sumPR[i] = 0;
                sumPS[i] = 0;
                pricnt[i] = 0;
                pricntI[i] = 0;
                sumSIP[i] = 0;
                sumRIP[i] = 0;
                sumSSP3[i] = 0;
                sumSSP4[i] = 0;
                sumSSP5[i] = 0;
                sumSIPA[i] = 0;
                sumSSP3A[i] = 0;
                sumSSP4A[i] = 0;
                sumSSP5A[i] = 0;

            }
            /*for(int i = 0; i < 5; i++){
                sumSR[i] = 0;
                sumSS[i] = 0;
            }*/
            for(int i = 0; i < 6; i++){
                sumSR[i] = 0;
                sumSS[i] = 0;
                sumSRI[i] = 0;
                sumSSI[i] = 0;
                sumSRT[i] = 0;
                sumSST[i] = 0;
                delaydistributiontotalS[i] = 0;
                delayaveT[i] = 0;
            }
            /*for(int j = 0; j < 5; j++){
                    sumSR[j] += basicSafetyMessageInfo.numberPacketReceivedSpeed[j];
            }*/
            for(int j = 0; j < 6; j++){
                sumSR[j] += basicSafetyMessageInfo.numberPacketReceivedSpeed[j];
                sumSRI[j] += basicSafetyMessageInfo.numberPacketReceivedSpeedInter[j];
                sumSRT[j] = sumSRT[j] + basicSafetyMessageInfo.numberPacketReceivedSpeed[j] + basicSafetyMessageInfo.numberPacketReceivedSpeedInter[j];
            }
            for(int k = 0; k < 4; k++){
                sumRIP[k] += basicSafetyMessageInfo.numberPacketReceivedPriInter[k];
            }
            //for(int i = 0;i < 800;i++){
            for(int i = 0; i < 1600; i++){
                sumR += basicSafetyMessageInfo.list[i].number[0] + basicSafetyMessageInfo.list[i].number[1] + basicSafetyMessageInfo.list[i].number[2] + basicSafetyMessageInfo.list[i].number[3];
                for(int j = 0; j < 4; j++){
                    sumPR[j] += basicSafetyMessageInfo.list[i].number[j];
                }
                
                
                char fname17[30];
                std::string numPCRS;
                int numPCR;
                sprintf(fname17,"tmpnumberPCR0_%d.txt",i + 1);
                std::ifstream inputfile17(fname17);
                inputfile17 >> numPCRS;
                numPCR = atoi(numPCRS.c_str());
                if(!inputfile17){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numPCR = 0;
                }

                char fname20[30];
                std::string numPCRS2;
                int numPCR2;
                sprintf(fname20,"tmpnumberPCR1_%d.txt",i + 1);
                std::ifstream inputfile20(fname20);
                inputfile20 >> numPCRS2;
                numPCR2 = atoi(numPCRS2.c_str());
                if(!inputfile20){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numPCR2 = 0;
                }

                char fname21[30];
                std::string numPCRS3;
                int numPCR3;
                sprintf(fname21,"tmpnumberPCR2_%d.txt",i + 1);
                std::ifstream inputfile21(fname21);
                inputfile21 >> numPCRS3;
                numPCR3 = atoi(numPCRS3.c_str());
                if(!inputfile21){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numPCR3 = 0;
                }

                char fname22[30];
                std::string numPCRS4;
                int numPCR4;
                sprintf(fname22,"tmpnumberPCR3_%d.txt",i + 1);
                std::ifstream inputfile22(fname22);
                inputfile22 >> numPCRS4;
                numPCR4 = atoi(numPCRS4.c_str());
                if(!inputfile22){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numPCR4 = 0;
                }

                char fname32[30];
                std::string numSendInterS;
                int numSendInter;
                sprintf(fname32,"tmpSendIntersection_%d.txt",i + 1);
                std::ifstream inputfile32(fname32);
                inputfile32 >> numSendInterS;
                numSendInter = atoi(numSendInterS.c_str());
                if(!inputfile32){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInter = 0;
                }


                char fnameBA[30];
                std::string numSendInterPri0S;
                int numSendInterPri0;
                sprintf(fnameBA,"tmpPriIntersection0_%d.txt",i + 1);
                std::ifstream inputfileBA(fnameBA);
                inputfileBA >> numSendInterPri0S;
                numSendInterPri0 = atoi(numSendInterPri0S.c_str());
                if(!inputfileBA){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterPri0 = 0;
                }

                char fnameBB[30];
                std::string numSendInterPri1S;
                int numSendInterPri1;
                sprintf(fnameBB,"tmpPriIntersection1_%d.txt",i + 1);
                std::ifstream inputfileBB(fnameBB);
                inputfileBB >> numSendInterPri1S;
                numSendInterPri1 = atoi(numSendInterPri1S.c_str());
                if(!inputfileBB){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterPri1 = 0;
                }

                char fnameBC[30];
                std::string numSendInterPri2S;
                int numSendInterPri2;
                sprintf(fnameBC,"tmpPriIntersection2_%d.txt",i + 1);
                std::ifstream inputfileBC(fnameBC);
                inputfileBC >> numSendInterPri2S;
                numSendInterPri2 = atoi(numSendInterPri2S.c_str());
                if(!inputfileBC){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterPri2 = 0;
                }

                char fnameBD[30];
                std::string numSendInterPri3S;
                int numSendInterPri3;
                sprintf(fnameBD,"tmpPriIntersection3_%d.txt",i + 1);
                std::ifstream inputfileBD(fnameBD);
                inputfileBD >> numSendInterPri3S;
                numSendInterPri3 = atoi(numSendInterPri3S.c_str());
                if(!inputfileBD){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterPri3 = 0;
                }

                char fnameBE[30];
                std::string numSendSpeed3Pri0S;
                int numSendSpeed3Pri0;
                sprintf(fnameBE,"tmpPriSpeed3_0_%d.txt",i + 1);
                std::ifstream inputfileBE(fnameBE);
                inputfileBE >> numSendSpeed3Pri0S;
                numSendSpeed3Pri0 = atoi(numSendSpeed3Pri0S.c_str());
                if(!inputfileBE){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed3Pri0 = 0;
                }

                char fnameBF[30];
                std::string numSendSpeed3Pri1S;
                int numSendSpeed3Pri1;
                sprintf(fnameBF,"tmpPriSpeed3_1_%d.txt",i + 1);
                std::ifstream inputfileBF(fnameBF);
                inputfileBF >> numSendSpeed3Pri1S;
                numSendSpeed3Pri1 = atoi(numSendSpeed3Pri1S.c_str());
                if(!inputfileBF){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed3Pri1 = 0;
                }

                char fnameBG[30];
                std::string numSendSpeed3Pri2S;
                int numSendSpeed3Pri2;
                sprintf(fnameBG,"tmpPriSpeed3_2_%d.txt",i + 1);
                std::ifstream inputfileBG(fnameBG);
                inputfileBG >> numSendSpeed3Pri2S;
                numSendSpeed3Pri2 = atoi(numSendSpeed3Pri2S.c_str());
                if(!inputfileBG){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed3Pri2 = 0;
                }

                char fnameBH[30];
                std::string numSendSpeed3Pri3S;
                int numSendSpeed3Pri3;
                sprintf(fnameBH,"tmpPriSpeed3_3_%d.txt",i + 1);
                std::ifstream inputfileBH(fnameBH);
                inputfileBH >> numSendSpeed3Pri3S;
                numSendSpeed3Pri3 = atoi(numSendSpeed3Pri3S.c_str());
                if(!inputfileBH){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed3Pri3 = 0;
                }

                char fnameBI[30];
                std::string numSendSpeed4Pri0S;
                int numSendSpeed4Pri0;
                sprintf(fnameBI,"tmpPriSpeed4_0_%d.txt",i + 1);
                std::ifstream inputfileBI(fnameBI);
                inputfileBI >> numSendSpeed4Pri0S;
                numSendSpeed4Pri0 = atoi(numSendSpeed4Pri0S.c_str());
                if(!inputfileBI){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed4Pri0 = 0;
                }

                char fnameBJ[30];
                std::string numSendSpeed4Pri1S;
                int numSendSpeed4Pri1;
                sprintf(fnameBJ,"tmpPriSpeed4_1_%d.txt",i + 1);
                std::ifstream inputfileBJ(fnameBJ);
                inputfileBJ >> numSendSpeed4Pri1S;
                numSendSpeed4Pri1 = atoi(numSendSpeed4Pri1S.c_str());
                if(!inputfileBJ){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed4Pri1 = 0;
                }

                char fnameBK[30];
                std::string numSendSpeed4Pri2S;
                int numSendSpeed4Pri2;
                sprintf(fnameBK,"tmpPriSpeed4_2_%d.txt",i + 1);
                std::ifstream inputfileBK(fnameBK);
                inputfileBK >> numSendSpeed4Pri2S;
                numSendSpeed4Pri2 = atoi(numSendSpeed4Pri2S.c_str());
                if(!inputfileBK){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed4Pri2 = 0;
                }

                char fnameBL[30];
                std::string numSendSpeed4Pri3S;
                int numSendSpeed4Pri3;
                sprintf(fnameBL,"tmpPriSpeed4_3_%d.txt",i + 1);
                std::ifstream inputfileBL(fnameBL);
                inputfileBL >> numSendSpeed4Pri3S;
                numSendSpeed4Pri3 = atoi(numSendSpeed4Pri3S.c_str());
                if(!inputfileBL){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed4Pri3 = 0;
                }

                char fnameBM[30];
                std::string numSendSpeed5Pri0S;
                int numSendSpeed5Pri0;
                sprintf(fnameBM,"tmpPriSpeed5_0_%d.txt",i + 1);
                std::ifstream inputfileBM(fnameBM);
                inputfileBM >> numSendSpeed5Pri0S;
                numSendSpeed5Pri0 = atoi(numSendSpeed5Pri0S.c_str());
                if(!inputfileBM){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed5Pri0 = 0;
                }

                char fnameBN[30];
                std::string numSendSpeed5Pri1S;
                int numSendSpeed5Pri1;
                sprintf(fnameBN,"tmpPriSpeed5_1_%d.txt",i + 1);
                std::ifstream inputfileBN(fnameBN);
                inputfileBN >> numSendSpeed5Pri1S;
                numSendSpeed5Pri1 = atoi(numSendSpeed5Pri1S.c_str());
                if(!inputfileBN){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed5Pri1 = 0;
                }

                char fnameBO[30];
                std::string numSendSpeed5Pri2S;
                int numSendSpeed5Pri2;
                sprintf(fnameBO,"tmpPriSpeed5_2_%d.txt",i + 1);
                std::ifstream inputfileBO(fnameBO);
                inputfileBO >> numSendSpeed5Pri2S;
                numSendSpeed5Pri2 = atoi(numSendSpeed5Pri2S.c_str());
                if(!inputfileBO){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed5Pri2 = 0;
                }

                char fnameBP[30];
                std::string numSendSpeed5Pri3S;
                int numSendSpeed5Pri3;
                sprintf(fnameBP,"tmpPriSpeed5_3_%d.txt",i + 1);
                std::ifstream inputfileBP(fnameBP);
                inputfileBP >> numSendSpeed5Pri3S;
                numSendSpeed5Pri3 = atoi(numSendSpeed5Pri3S.c_str());
                if(!inputfileBP){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed5Pri3 = 0;
                }



                char fnameEA[30];
                std::string numSendInterAPri0S;
                int numSendInterAPri0;
                sprintf(fnameEA,"tmpPriIntersectionA0_%d.txt",i + 1);
                std::ifstream inputfileEA(fnameEA);
                inputfileEA >> numSendInterAPri0S;
                numSendInterAPri0 = atoi(numSendInterAPri0S.c_str());
                if(!inputfileEA){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterAPri0 = 0;
                }

                char fnameEB[30];
                std::string numSendInterAPri1S;
                int numSendInterAPri1;
                sprintf(fnameEB,"tmpPriIntersectionA1_%d.txt",i + 1);
                std::ifstream inputfileEB(fnameEB);
                inputfileEB >> numSendInterAPri1S;
                numSendInterAPri1 = atoi(numSendInterAPri1S.c_str());
                if(!inputfileEB){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterAPri1 = 0;
                }

                char fnameEC[30];
                std::string numSendInterAPri2S;
                int numSendInterAPri2;
                sprintf(fnameEC,"tmpPriIntersectionA2_%d.txt",i + 1);
                std::ifstream inputfileEC(fnameEC);
                inputfileEC >> numSendInterAPri2S;
                numSendInterAPri2 = atoi(numSendInterAPri2S.c_str());
                if(!inputfileEC){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterAPri2 = 0;
                }

                char fnameED[30];
                std::string numSendInterAPri3S;
                int numSendInterAPri3;
                sprintf(fnameED,"tmpPriIntersectionA3_%d.txt",i + 1);
                std::ifstream inputfileED(fnameED);
                inputfileED >> numSendInterAPri3S;
                numSendInterAPri3 = atoi(numSendInterAPri3S.c_str());
                if(!inputfileED){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterAPri3 = 0;
                }

                char fnameEE[30];
                std::string numSendSpeed3APri0S;
                int numSendSpeed3APri0;
                sprintf(fnameEE,"tmpPriSpeedA3_0_%d.txt",i + 1);
                std::ifstream inputfileEE(fnameEE);
                inputfileEE >> numSendSpeed3APri0S;
                numSendSpeed3APri0 = atoi(numSendSpeed3APri0S.c_str());
                if(!inputfileEE){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed3APri0 = 0;
                }

                char fnameEF[30];
                std::string numSendSpeed3APri1S;
                int numSendSpeed3APri1;
                sprintf(fnameEF,"tmpPriSpeedA3_1_%d.txt",i + 1);
                std::ifstream inputfileEF(fnameEF);
                inputfileEF >> numSendSpeed3APri1S;
                numSendSpeed3APri1 = atoi(numSendSpeed3APri1S.c_str());
                if(!inputfileEF){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed3APri1 = 0;
                }

                char fnameEG[30];
                std::string numSendSpeed3APri2S;
                int numSendSpeed3APri2;
                sprintf(fnameEG,"tmpPriSpeedA3_2_%d.txt",i + 1);
                std::ifstream inputfileEG(fnameEG);
                inputfileEG >> numSendSpeed3APri2S;
                numSendSpeed3APri2 = atoi(numSendSpeed3APri2S.c_str());
                if(!inputfileEG){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed3APri2 = 0;
                }

                char fnameEH[30];
                std::string numSendSpeed3APri3S;
                int numSendSpeed3APri3;
                sprintf(fnameEH,"tmpPriSpeedA3_3_%d.txt",i + 1);
                std::ifstream inputfileEH(fnameEH);
                inputfileEH >> numSendSpeed3APri3S;
                numSendSpeed3APri3 = atoi(numSendSpeed3APri3S.c_str());
                if(!inputfileEH){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed3APri3 = 0;
                }

                char fnameEI[30];
                std::string numSendSpeed4APri0S;
                int numSendSpeed4APri0;
                sprintf(fnameEI,"tmpPriSpeedA4_0_%d.txt",i + 1);
                std::ifstream inputfileEI(fnameEI);
                inputfileEI >> numSendSpeed4APri0S;
                numSendSpeed4APri0 = atoi(numSendSpeed4APri0S.c_str());
                if(!inputfileEI){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed4APri0 = 0;
                }

                char fnameEJ[30];
                std::string numSendSpeed4APri1S;
                int numSendSpeed4APri1;
                sprintf(fnameEJ,"tmpPriSpeedA4_1_%d.txt",i + 1);
                std::ifstream inputfileEJ(fnameEJ);
                inputfileEJ >> numSendSpeed4APri1S;
                numSendSpeed4APri1 = atoi(numSendSpeed4APri1S.c_str());
                if(!inputfileEJ){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed4APri1 = 0;
                }

                char fnameEK[30];
                std::string numSendSpeed4APri2S;
                int numSendSpeed4APri2;
                sprintf(fnameEK,"tmpPriSpeedA4_2_%d.txt",i + 1);
                std::ifstream inputfileEK(fnameEK);
                inputfileEK >> numSendSpeed4APri2S;
                numSendSpeed4APri2 = atoi(numSendSpeed4APri2S.c_str());
                if(!inputfileEK){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed4APri2 = 0;
                }

                char fnameEL[30];
                std::string numSendSpeed4APri3S;
                int numSendSpeed4APri3;
                sprintf(fnameEL,"tmpPriSpeedA4_3_%d.txt",i + 1);
                std::ifstream inputfileEL(fnameEL);
                inputfileEL >> numSendSpeed4APri3S;
                numSendSpeed4APri3 = atoi(numSendSpeed4APri3S.c_str());
                if(!inputfileEL){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed4APri3 = 0;
                }

                char fnameEM[30];
                std::string numSendSpeed5APri0S;
                int numSendSpeed5APri0;
                sprintf(fnameEM,"tmpPriSpeedA5_0_%d.txt",i + 1);
                std::ifstream inputfileEM(fnameEM);
                inputfileEM >> numSendSpeed5APri0S;
                numSendSpeed5APri0 = atoi(numSendSpeed5APri0S.c_str());
                if(!inputfileEM){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed5APri0 = 0;
                }

                char fnameEN[30];
                std::string numSendSpeed5APri1S;
                int numSendSpeed5APri1;
                sprintf(fnameEN,"tmpPriSpeedA5_1_%d.txt",i + 1);
                std::ifstream inputfileEN(fnameEN);
                inputfileEN >> numSendSpeed5APri1S;
                numSendSpeed5APri1 = atoi(numSendSpeed5APri1S.c_str());
                if(!inputfileEN){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed5APri1 = 0;
                }

                char fnameEO[30];
                std::string numSendSpeed5APri2S;
                int numSendSpeed5APri2;
                sprintf(fnameEO,"tmpPriSpeedA5_2_%d.txt",i + 1);
                std::ifstream inputfileEO(fnameEO);
                inputfileEO >> numSendSpeed5APri2S;
                numSendSpeed5APri2 = atoi(numSendSpeed5APri2S.c_str());
                if(!inputfileEO){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed5APri2 = 0;
                }

                char fnameEP[30];
                std::string numSendSpeed5APri3S;
                int numSendSpeed5APri3;
                sprintf(fnameEP,"tmpPriSpeedA5_3_%d.txt",i + 1);
                std::ifstream inputfileEP(fnameEP);
                inputfileEP >> numSendSpeed5APri3S;
                numSendSpeed5APri3 = atoi(numSendSpeed5APri3S.c_str());
                if(!inputfileEP){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendSpeed5APri3 = 0;
                }

                

                char fnameM[30];
                std::string numSendInterSpeedS0;
                int numSendInterSpeed0;
                sprintf(fnameM,"tmpSendIntersectionSpeed0_%d.txt",i + 1);
                std::ifstream inputfileM(fnameM);
                inputfileM >> numSendInterSpeedS0;
                numSendInterSpeed0 = atoi(numSendInterSpeedS0.c_str());
                if(!inputfileM){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterSpeed0 = 0;
                }

                char fnameN[30];
                std::string numSendInterSpeedS1;
                int numSendInterSpeed1;
                sprintf(fnameN,"tmpSendIntersectionSpeed1_%d.txt",i + 1);
                std::ifstream inputfileN(fnameN);
                inputfileN >> numSendInterSpeedS1;
                numSendInterSpeed1 = atoi(numSendInterSpeedS1.c_str());
                if(!inputfileN){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterSpeed1 = 0;
                }

                char fnameO[30];
                std::string numSendInterSpeedS2;
                int numSendInterSpeed2;
                sprintf(fnameO,"tmpSendIntersectionSpeed2_%d.txt",i + 1);
                std::ifstream inputfileO(fnameO);
                inputfileO >> numSendInterSpeedS2;
                numSendInterSpeed2 = atoi(numSendInterSpeedS2.c_str());
                if(!inputfileO){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterSpeed2 = 0;
                }

                char fnameP[30];
                std::string numSendInterSpeedS3;
                int numSendInterSpeed3;
                sprintf(fnameP,"tmpSendIntersectionSpeed3_%d.txt",i + 1);
                std::ifstream inputfileP(fnameP);
                inputfileP >> numSendInterSpeedS3;
                numSendInterSpeed3 = atoi(numSendInterSpeedS3.c_str());
                if(!inputfileP){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterSpeed3 = 0;
                }

                char fnameQ[30];
                std::string numSendInterSpeedS4;
                int numSendInterSpeed4;
                sprintf(fnameQ,"tmpSendIntersectionSpeed4_%d.txt",i + 1);
                std::ifstream inputfileQ(fnameQ);
                inputfileQ >> numSendInterSpeedS4;
                numSendInterSpeed4 = atoi(numSendInterSpeedS4.c_str());
                if(!inputfileQ){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterSpeed4 = 0;
                }

                char fnameR[30];
                std::string numSendInterSpeedS5;
                int numSendInterSpeed5;
                sprintf(fnameR,"tmpSendIntersectionSpeed5_%d.txt",i + 1);
                std::ifstream inputfileR(fnameR);
                inputfileR >> numSendInterSpeedS5;
                numSendInterSpeed5 = atoi(numSendInterSpeedS5.c_str());
                if(!inputfileR){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSendInterSpeed5 = 0;
                }

                char fname43[30];
                std::string numSpeedS0;
                int numSpeed0;
                sprintf(fname43,"tmpnumberSpeed0_%d.txt",i + 1);
                std::ifstream inputfile43(fname43);
                inputfile43 >> numSpeedS0;
                numSpeed0 = atoi(numSpeedS0.c_str());
                if(!inputfile43){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSpeed0 = 0;
                }

                char fname44[30];
                std::string numSpeedS1;
                int numSpeed1;
                sprintf(fname44,"tmpnumberSpeed1_%d.txt",i + 1);
                std::ifstream inputfile44(fname44);
                inputfile44 >> numSpeedS1;
                numSpeed1 = atoi(numSpeedS1.c_str());
                if(!inputfile44){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSpeed1 = 0;
                }

                char fname45[30];
                std::string numSpeedS2;
                int numSpeed2;
                sprintf(fname45,"tmpnumberSpeed2_%d.txt",i + 1);
                std::ifstream inputfile45(fname45);
                inputfile45 >> numSpeedS2;
                numSpeed2 = atoi(numSpeedS2.c_str());
                if(!inputfile45){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSpeed2 = 0;
                }

                char fname46[30];
                std::string numSpeedS3;
                int numSpeed3;
                sprintf(fname46,"tmpnumberSpeed3_%d.txt",i + 1);
                std::ifstream inputfile46(fname46);
                inputfile46 >> numSpeedS3;
                numSpeed3 = atoi(numSpeedS3.c_str());
                if(!inputfile46){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSpeed3 = 0;
                }

                char fname47[30];
                std::string numSpeedS4;
                int numSpeed4;
                sprintf(fname47,"tmpnumberSpeed4_%d.txt",i + 1);
                std::ifstream inputfile47(fname47);
                inputfile47 >> numSpeedS4;
                numSpeed4 = atoi(numSpeedS4.c_str());
                if(!inputfile47){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSpeed4 = 0;
                }

                char fnameEx2[30];
                std::string numSpeedS5;
                int numSpeed5;
                sprintf(fnameEx2,"tmpnumberSpeed5_%d.txt",i + 1);
                std::ifstream inputfileEx2(fnameEx2);
                inputfileEx2 >> numSpeedS5;
                numSpeed5 = atoi(numSpeedS5.c_str());
                if(!inputfileEx2){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    numSpeed5 = 0;
                }

                char fnameAAA[30];
                std::string InterS;
                int Inter;
                sprintf(fnameAAA,"tmpIntersection_%d.txt",i + 1);
                std::ifstream inputfileAAA(fnameAAA);
                inputfileAAA >> InterS;
                Inter = atoi(InterS.c_str());
                if(!inputfileAAA){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                    Inter = 3;
                }
                


                char fname23[30];
                std::string tmppriS = "";
                int tmppri = 8;
                sprintf(fname23,"tmpPri_%d.txt",i + 1);
                std::ifstream inputfile23(fname23);
                inputfile23 >> tmppriS;
                tmppri = atoi(tmppriS.c_str());
                if(!inputfile23){
                //std::cout << "ファイルが開けませんでした" << endl;
                //std::cin.get();
                }else{
                    if(Inter == 0){
                        if(tmppri == 1 || tmppri == 2){
                            pricnt[0]++;
                        }else if(tmppri == 0 || tmppri == 3){
                            pricnt[1]++;
                        }else if(tmppri == 4 || tmppri == 5){
                            pricnt[2]++;
                        }else if(tmppri == 6 || tmppri == 7){
                            pricnt[3]++;
                        }else{

                        }
                    }else if(Inter == 1){
                        if(tmppri == 1 || tmppri == 2){
                            pricntI[0]++;
                        }else if(tmppri == 0 || tmppri == 3){
                            pricntI[1]++;
                        }else if(tmppri == 4 || tmppri == 5){
                            pricntI[2]++;
                        }else if(tmppri == 6 || tmppri == 7){
                            pricntI[3]++;
                        }else{

                        }
                    }else{

                    }
                }

                sumSInter += numSendInter;
                sumS += numPCR + numPCR2 + numPCR3 + numPCR4;
                if(i == 0){
                    sumS1 = numPCR + numPCR2 + numPCR3 + numPCR4;
                }

                sumPS[0] += numPCR;
                sumPS[1] += numPCR2;
                sumPS[2] += numPCR3;
                sumPS[3] += numPCR4;

                sumSS[0] += numSpeed0;
                sumSS[1] += numSpeed1;
                sumSS[2] += numSpeed2;
                sumSS[3] += numSpeed3;
                sumSS[4] += numSpeed4;
                sumSS[5] += numSpeed5;

                sumSSI[0] += numSendInterSpeed0;
                sumSSI[1] += numSendInterSpeed1;
                sumSSI[2] += numSendInterSpeed2;
                sumSSI[3] += numSendInterSpeed3;
                sumSSI[4] += numSendInterSpeed4;
                sumSSI[5] += numSendInterSpeed5;

                sumSST[0] = sumSST[0] + numSpeed0 + numSendInterSpeed0;
                sumSST[1] = sumSST[1] + numSpeed1 + numSendInterSpeed1;
                sumSST[2] = sumSST[2] + numSpeed2 + numSendInterSpeed2;
                sumSST[3] = sumSST[3] + numSpeed3 + numSendInterSpeed3;
                sumSST[4] = sumSST[4] + numSpeed4 + numSendInterSpeed4;
                sumSST[5] = sumSST[5] + numSpeed5 + numSendInterSpeed5;

                sumSIP[0] += numSendInterPri0;
                sumSIP[1] += numSendInterPri1;
                sumSIP[2] += numSendInterPri2;
                sumSIP[3] += numSendInterPri3;

                sumSSP3[0] += numSendSpeed3Pri0;
                sumSSP3[1] += numSendSpeed3Pri1;
                sumSSP3[2] += numSendSpeed3Pri2;
                sumSSP3[3] += numSendSpeed3Pri3;

                sumSSP4[0] += numSendSpeed4Pri0;
                sumSSP4[1] += numSendSpeed4Pri1;
                sumSSP4[2] += numSendSpeed4Pri2;
                sumSSP4[3] += numSendSpeed4Pri3;

                sumSSP5[0] += numSendSpeed5Pri0;
                sumSSP5[1] += numSendSpeed5Pri1;
                sumSSP5[2] += numSendSpeed5Pri2;
                sumSSP5[3] += numSendSpeed5Pri3;

                sumSIPA[0] += numSendInterAPri0;
                sumSIPA[1] += numSendInterAPri1;
                sumSIPA[2] += numSendInterAPri2;
                sumSIPA[3] += numSendInterAPri3;

                sumSSP3A[0] += numSendSpeed3APri0;
                sumSSP3A[1] += numSendSpeed3APri1;
                sumSSP3A[2] += numSendSpeed3APri2;
                sumSSP3A[3] += numSendSpeed3APri3;

                sumSSP4A[0] += numSendSpeed4APri0;
                sumSSP4A[1] += numSendSpeed4APri1;
                sumSSP4A[2] += numSendSpeed4APri2;
                sumSSP4A[3] += numSendSpeed4APri3;

                sumSSP5A[0] += numSendSpeed5APri0;
                sumSSP5A[1] += numSendSpeed5APri1;
                sumSSP5A[2] += numSendSpeed5APri2;
                sumSSP5A[3] += numSendSpeed5APri3;


            }
            for(int i = 0;i < 4; i++){
                if(sumPR[i] != 0){
                    basicSafetyMessageInfo.delayaveP[i] = basicSafetyMessageInfo.delaysumP[i] / sumPR[i];
                }
                if(sumRIP[i] != 0){
                    basicSafetyMessageInfo.delayavePI[i] = basicSafetyMessageInfo.delaysumPI[i] / sumRIP[i];
                }
                sum1 += basicSafetyMessageInfo.list[0].number[i];
            }
            /*for(int i = 0; i < 5; i++){
                if(sumSR[i] != 0){
                    basicSafetyMessageInfo.delayaveS[i] = basicSafetyMessageInfo.delaysumS[i] / sumSR[i];
                }
            }*/
            for(int i = 0; i < 6; i++){
                if(sumSR[i] != 0){
                    basicSafetyMessageInfo.delayaveS[i] = basicSafetyMessageInfo.delaysumS[i] / sumSR[i];
                }
                if(sumSRI[i] != 0){
                    basicSafetyMessageInfo.delayaveSI[i] = basicSafetyMessageInfo.delaysumSI[i] / sumSRI[i];
                }
                if((sumSR[i] + sumSRI[i]) != 0){
                    delayaveT[i] = (basicSafetyMessageInfo.delaysumS[i] + basicSafetyMessageInfo.delaysumSI[i]) / (sumSR[i] + sumSRI[i]);
                }
            }
            for(int i = 0; i < 11; i++){
                delaydistributiontotal += basicSafetyMessageInfo.delaydistribution[i];
                delaydistributiontotalS[0] += basicSafetyMessageInfo.delaydistributionS0[i];
                delaydistributiontotalS[1] += basicSafetyMessageInfo.delaydistributionS1[i];
                delaydistributiontotalS[2] += basicSafetyMessageInfo.delaydistributionS2[i];
                delaydistributiontotalS[3] += basicSafetyMessageInfo.delaydistributionS3[i];
                delaydistributiontotalS[4] += basicSafetyMessageInfo.delaydistributionS4[i];
                delaydistributiontotalS[5] += basicSafetyMessageInfo.delaydistributionS5[i];
            }
            if(basicSafetyMessageInfo.numberPacketReceivedinintersection != 0){
                basicSafetyMessageInfo.delayaveInter = basicSafetyMessageInfo.delaysumInter / basicSafetyMessageInfo.numberPacketReceivedinintersection;
            }
            if((sumR + basicSafetyMessageInfo.numberPacketReceivedinintersection) != 0){
                basicSafetyMessageInfo.delayave = (basicSafetyMessageInfo.delaysum + basicSafetyMessageInfo.delaysumInter)/ (sumR + basicSafetyMessageInfo.numberPacketReceivedinintersection);
            }
            if(sumR != 0){
                delaynotinter = basicSafetyMessageInfo.delaysum / sumR; 
            }
            if(basicSafetyMessageInfo.numberPacketsReceived != 0){
                basicSafetyMessageInfo.delayave2 = basicSafetyMessageInfo.delaysum2 / basicSafetyMessageInfo.numberPacketsReceived;
            }
            std::cout << "--------------------------------------------" << endl;
            std::cout << "MAX delay in 50 = " << basicSafetyMessageInfo.delaymax << endl;
            std::cout << "--------------------------------------------" << endl;
            std::cout << "MAX delay total = " << basicSafetyMessageInfo.delaymax2 << endl;
            std::cout << "--------------------------------------------" << endl;
            std::cout << "numberpacketSend from 1 = "<< sumS1 << endl;
            std::cout << "numberpacketreceived from 1 = "<< sum1 << endl;
            std::cout << "--------------------------------------------" << endl;
            std::cout << "numberpacketReceive from Intersection = "<< basicSafetyMessageInfo.numberPacketReceivedinintersection << endl;
            std::cout << "numberpacketSend from Intersection = "<< sumSInter << endl;
            std::cout << "delay from Intersection = " << basicSafetyMessageInfo.delayaveInter << endl;
            std::cout << "--------------------------------------------" << endl;
            std::cout << "numberpacketReceive from NOT Intersection = "<< sumR << endl;
            std::cout << "numberpacketSend from NOT Intersection = "<< sumS << endl;    
            std::cout << "delay from NOT Intersection = " << delaynotinter << endl;
            std::cout << "--------------------------------------------" << endl;
            //std::cout << "numberpacketreceived 801 = "<< sumR << endl;
            std::cout << "numberpacketreceived 1601 = "<< sumR + basicSafetyMessageInfo.numberPacketReceivedinintersection << endl;
            //std::cout << "numberpacketsend to 801 = " << sumS << endl;
            std::cout << "numberpacketsend to 1601 = " << sumS + sumSInter << endl;
            //std::cout << "delay average to 801 in xxm= " << basicSafetyMessageInfo.delayave << endl;
            std::cout << "delay average to 1601 in xxm= " << basicSafetyMessageInfo.delayave << endl;
            //std::cout << "delay average to 801 = " << basicSafetyMessageInfo.delayave2 << endl;
            std::cout << "--------------------------------------------" << endl;
            std::cout << "delay average to 1601 = " << basicSafetyMessageInfo.delayave2 << endl;
            for(int i = 0; i < 4; i++){
                std::cout << "--------------------------------------------" << endl;
                std::cout << "Not Inter AC" << i << " = " << pricnt[i] << endl;
                //std::cout << "numberpacketreceived 801 AC" << i << " = " << sumPR[i] << endl;
                std::cout << "numberpacketreceived 1601 AC" << i << " = " << sumPR[i] << endl;
                //std::cout << "numberpacketsend to 801 of AC" << i << " = " << sumPS[i] << endl;
                std::cout << "numberpacketsend to 1601 of AC" << i << " = " << sumPS[i] << endl;
                std::cout << "numberpacketdelay AC" << i << " = " << basicSafetyMessageInfo.delayaveP[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 4; i++){
                std::cout << "--------------------------------------------" << endl;
                std::cout << "Inter AC" << i << " = " << pricntI[i] << endl;
                std::cout << "numberpacketreceived Inter 1601 AC" << i << " = " << sumRIP[i] << endl;
                std::cout << "numberpacketsend Inter to 1601 of AC" << i << " = " << sumSIP[i] << endl;
                std::cout << "numberpacketdelay Inter AC" << i << " = " << basicSafetyMessageInfo.delayavePI[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            /*for(int i = 0; i < 5; i++){
                //std::cout << "numberpacketreceived 801 Speed" << i << " = " << sumSR[i] << endl;
                std::cout << "numberpacketreceived 1601 Speed" << i << " = " << sumSR[i] << endl;
                //std::cout << "numberpacketsend to 801 of Speed" << i << " = " << sumSS[i] << endl;
                std::cout << "numberpacketsend to 1601 of Speed" << i << " = " << sumSS[i] << endl;
                std::cout << "numberpacketdelay Speed" << i << " = " << basicSafetyMessageInfo.delayaveS[i] << endl;
            }*/
            
            for(int i = 0; i < 6; i++){
                //std::cout << "numberpacketreceived 801 Speed" << i << " = " << sumSR[i] << endl;
                std::cout << "--------------------------------------------" << endl;
                std::cout << "numberpacketreceived 1601 Speed Inter" << i << " = " << sumSRI[i] << endl;
                //std::cout << "numberpacketsend to 801 of Speed" << i << " = " << sumSS[i] << endl;
                std::cout << "numberpacketsend to 1601 of Speed Inter" << i << " = " << sumSSI[i] << endl;
                std::cout << "numberpacketdelay Speed Inter" << i << " = " << basicSafetyMessageInfo.delayaveSI[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 6; i++){
                //std::cout << "numberpacketreceived 801 Speed" << i << " = " << sumSR[i] << endl;
                std::cout << "--------------------------------------------" << endl;
                std::cout << "numberpacketreceived 1601 Speed NOT inter" << i << " = " << sumSR[i] << endl;
                //std::cout << "numberpacketsend to 801 of Speed" << i << " = " << sumSS[i] << endl;
                std::cout << "numberpacketsend to 1601 of Speed NOT inter" << i << " = " << sumSS[i] << endl;
                std::cout << "numberpacketdelay Speed NOT inter" << i << " = " << basicSafetyMessageInfo.delayaveS[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 6; i++){
                //std::cout << "numberpacketreceived 801 Speed" << i << " = " << sumSR[i] << endl;
                std::cout << "--------------------------------------------" << endl;
                std::cout << "numberpacketreceived 1601 Speed TOTAL" << i << " = " << sumSRT[i] << endl;
                //std::cout << "numberpacketsend to 801 of Speed" << i << " = " << sumSS[i] << endl;
                std::cout << "numberpacketsend to 1601 of Speed TOTAL" << i << " = " << sumSST[i] << endl;
                std::cout << "numberpacketdelay Speed TOTAL" << i << " = " << delayaveT[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 4; i++){
                std::cout << "--------------------------------------------" << endl;
                std::cout << "numberSendInterPriority" << i << " = " << sumSIP[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 4; i++){
                std::cout << "--------------------------------------------" << endl;
                //std::cout << "numberSendSpeed3Priority" << i << " = " << sumSSP3[i] << endl;
                std::cout << "numberSendSpeed1Priority" << i << " = " << sumSSP3[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 4; i++){
                std::cout << "--------------------------------------------" << endl;
                //std::cout << "numberSendSpeed4Priority" << i << " = " << sumSSP4[i] << endl;
                std::cout << "numberSendSpeed2Priority" << i << " = " << sumSSP3[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 4; i++){
                std::cout << "--------------------------------------------" << endl;
                std::cout << "numberSendSpeed5Priority" << i << " = " << sumSSP5[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 4; i++){
                std::cout << "--------------------------------------------" << endl;
                std::cout << "numberSendInterPriority all" << i << " = " << sumSIPA[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 4; i++){
                std::cout << "--------------------------------------------" << endl;
                std::cout << "numberSendSpeed3Priority all" << i << " = " << sumSSP3A[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 4; i++){
                std::cout << "--------------------------------------------" << endl;
                std::cout << "numberSendSpeed4Priority all" << i << " = " << sumSSP4A[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 4; i++){
                std::cout << "--------------------------------------------" << endl;
                std::cout << "numberSendSpeed5Priority all" << i << " = " << sumSSP5A[i] << endl;
                std::cout << "--------------------------------------------" << endl;
            }
            for(int i = 0; i < 11; i++){
                std::cout << "delaydistribution all " << i << " = " << basicSafetyMessageInfo.delaydistribution[i] << endl;
            }
            std::cout << "delaydistributiontotal all = " << delaydistributiontotal << endl;
            std::cout << "--------------------------------------------" << endl;

            for(int i = 0; i < 11; i++){
                std::cout << "delaydistributionS0 " << i << " = " << basicSafetyMessageInfo.delaydistributionS0[i] << endl;
            }
            std::cout << "delaydistributiontotalS0 = " << delaydistributiontotalS[0] << endl;
            std::cout << "--------------------------------------------" << endl;

            for(int i = 0; i < 11; i++){
                std::cout << "delaydistributionS1 " << i << " = " << basicSafetyMessageInfo.delaydistributionS1[i] << endl;
            }
            std::cout << "delaydistributiontotalS1 = " << delaydistributiontotalS[1] << endl;
            std::cout << "--------------------------------------------" << endl;

            for(int i = 0; i < 11; i++){
                std::cout << "delaydistributionS2 " << i << " = " << basicSafetyMessageInfo.delaydistributionS2[i] << endl;
            }
            std::cout << "delaydistributiontotalS2 = " << delaydistributiontotalS[2] << endl;
            std::cout << "--------------------------------------------" << endl;

            for(int i = 0; i < 11; i++){
                std::cout << "delaydistributionS3 " << i << " = " << basicSafetyMessageInfo.delaydistributionS3[i] << endl;
            }
            std::cout << "delaydistributiontotalS3 = " << delaydistributiontotalS[3] << endl;
            std::cout << "--------------------------------------------" << endl;

            for(int i = 0; i < 11; i++){
                std::cout << "delaydistributionS4 " << i << " = " << basicSafetyMessageInfo.delaydistributionS4[i] << endl;
            }
            std::cout << "delaydistributiontotalS4 = " << delaydistributiontotalS[4] << endl;
            std::cout << "--------------------------------------------" << endl;

            for(int i = 0; i < 11; i++){
                std::cout << "delaydistributionS5 " << i << " = " << basicSafetyMessageInfo.delaydistributionS5[i] << endl;
            }
            std::cout << "delaydistributiontotalS5  = " << delaydistributiontotalS[5] << endl;
            std::cout << "--------------------------------------------" << endl;

            if(basicSafetyMessageInfo.alart == true){
                std::cout << "overflow" << endl;
            }

            char fname32[30];
            sprintf(fname32,"result_%d.txt",basicSafetyMessageInfo.MyNodeId);
            std::ofstream outputfile32(fname32);
            outputfile32 << "numberpacketSend from 1 = "<< sumS1 << std::endl;
            outputfile32 << "numberpacketreceived from 1 = "<< sum1 << std::endl;
            outputfile32 << "numberpacketSend from Intersection = "<< sumSInter  << std::endl;
            outputfile32 << "numberpacketReceive from Intersection = "<< basicSafetyMessageInfo.numberPacketReceivedinintersection << std::endl;
            outputfile32 << "delay from Intersection = " << basicSafetyMessageInfo.delayaveInter << std::endl;

            //outputfile32 << "numberpacketreceived 801 = "<< sumR  << std::endl;
            outputfile32 << "numberpacketreceived 1601 = "<< sumR  << std::endl;
            //outputfile32 << "numberpacketsend to 801 = " << sumS  << std::endl;
            outputfile32 << "numberpacketsend to 1601 = " << sumS  << std::endl;
            //outputfile32 << "delay average to 801 in xxm= " << basicSafetyMessageInfo.delayave << std::endl;
            outputfile32 << "delay average to 1601 in xxm= " << basicSafetyMessageInfo.delayave << std::endl;
            //outputfile32 << "delay average to 801 = " << basicSafetyMessageInfo.delayave2  << std::endl;
            outputfile32 << "delay average to 1601 = " << basicSafetyMessageInfo.delayave2  << std::endl;

            for(int i = 0; i < 4; i++){
                outputfile32 << "AC" << i << " = " << pricnt[i] << std::endl;
                //outputfile32 << "numberpacketreceived 801 AC" << i << " = " << sumPR[i] << std::endl;
                outputfile32 << "numberpacketreceived 1601 AC" << i << " = " << sumPR[i] << std::endl;
                //outputfile32 << "numberpacketsend to 801 of AC" << i << " = " << sumPS[i] << std::endl;
                outputfile32 << "numberpacketsend to 1601 of AC" << i << " = " << sumPS[i] << std::endl;
                outputfile32 << "numberpacketdelay AC" << i << " = " << basicSafetyMessageInfo.delayaveP[i] << std::endl;
            }
            /*for(int i = 0; i < 5; i++){
                //outputfile32 << "numberpacketreceived 801 Speed" << i << " = " << sumSR[i] << std::endl;
                outputfile32 << "numberpacketreceived 1601 Speed" << i << " = " << sumSR[i] << std::endl;
                //outputfile32 << "numberpacketsend to 801 of Speed" << i << " = " << sumSS[i] << std::endl;
                outputfile32 << "numberpacketsend to 1601 of Speed" << i << " = " << sumSS[i] << std::endl;
                outputfile32 << "numberpacketdelay Speed" << i << " = " << basicSafetyMessageInfo.delayaveS[i] << std::endl;
            }*/
            for(int i = 0; i < 6; i++){
                //outputfile32 << "numberpacketreceived 801 Speed" << i << " = " << sumSR[i] << std::endl;
                outputfile32 << "numberpacketreceived 1601 Speed" << i << " = " << sumSR[i] << std::endl;
                //outputfile32 << "numberpacketsend to 801 of Speed" << i << " = " << sumSS[i] << std::endl;
                outputfile32 << "numberpacketsend to 1601 of Speed" << i << " = " << sumSS[i] << std::endl;
                outputfile32 << "numberpacketdelay Speed" << i << " = " << basicSafetyMessageInfo.delayaveS[i] << std::endl;
            }

            outputfile32.close();
           
            //std::cout << "critical numberpacketreceived to  from 1 = "<< basicSafetyMessageInfo.list[0].numbercritical << endl;
            /*for(int i = 0; i < 300; i++){
                std::cout << "Packet"<< i+1 << " = " << basicSafetyMessageInfo.list[i].number << endl;
            }*/
            /*for(int i = 0; i < 400; i++){
                std::cout << "numberpacketreceived to 601 from " << i << " = " << basicSafetyMessageInfo.list[i].number << endl;
                sum += basicSafetyMessageInfo.list[i].number;
            }*/
            //std::cout << "numberpacketreceived to 601 sum = " << sum << endl;

        }
    }

    //緊急メッセージ？
    /*if((basicSafetyMessageInfo.numberPacketSend % 50) == 1){
        basicSafetyMessageInfo.criticalMessagetime = rand() % 50;
    }

    if((basicSafetyMessageInfo.numberPacketSend % 50) == basicSafetyMessageInfo.criticalMessagetime){
        unsigned int tmppriority = basicSafetyMessageInfo.priority;
        basicSafetyMessageInfo.priority = 7;


        (*this).SendBasicSafetyMessage(
        basicSafetyMessagePart1,
        part2Payload,
        basicSafetyMessageInfo.extendedPayloadSizeBytes - sizeof(DsrcBasicSafetyMessagePart1Type));

        if((-10000 <= currentX) && (currentX <= 10000) && (-200000 <= currentY) && (currentY <= 200000)){
            basicSafetyMessageInfo.numberCriticalPacketSend++;
            
        }else if((-10000 <= currentY) && (currentY <= 10000) && (-200000 <= currentX) && (currentX <= 200000)){
            basicSafetyMessageInfo.numberCriticalPacketSend++;
        }


        basicSafetyMessageInfo.priority = tmppriority;

    }*/
    
    

    /*if(basicSafetyMessageInfo.MyNodeId == 1){
        std::cout << "Critical Send From " << basicSafetyMessageInfo.MyNodeId << " = " << basicSafetyMessageInfo.numberCriticalPacketSend << endl;
    }*/

    //書き込み
    //成功した
    char fname24[30];
    sprintf(fname24,"tmpPri_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfile24(fname24);
    outputfile24 << (int)basicSafetyMessageInfo.priority << std::endl;
    outputfile24.close();

    char fname25[30];
    sprintf(fname25,"tmpSpeed_%d.txt",basicSafetyMessageInfo.MyNodeId);
    std::ofstream outputfile25(fname25);
    outputfile25 << (int)currentSpeed << std::endl;
    outputfile25.close();



    (*this).SendBasicSafetyMessage(
        basicSafetyMessagePart1,
        part2Payload,
        basicSafetyMessageInfo.extendedPayloadSizeBytes - sizeof(DsrcBasicSafetyMessagePart1Type));


    delete [] part2Payload;


    if (currentTime + basicSafetyMessageInfo.transmissionInterval < basicSafetyMessageInfo.endTime) {

        //std::cout << "2" << endl;
        //表示される

        //std::cout << "CurrentTime=" << (int)currentTime << endl;
        //std::cout << "interval=" << basicSafetyMessageInfo.transmissionInterval << endl;
        //単位ns,100msなら100000000になる(0が8個)

        //次にPeriodicallyTransmitBasicSafetyMessageが呼ばれる時間をスケジュール
        //basicSafetyMessageInfo.transmissionInterval = basicSafetyMessageInfo.transmissionInterval + 10000000;
        //上を入れたらEventExecutedが約三分の一になった

        //1234...と台数分表示成功
        //std::cout << "ID:" << (int)basicSafetyMessageInfo.MyNodeId << endl;


        //書き込み
        //成功したけど場所変えた方がいいかも.=>上に変えた

        simulationEngineInterfacePtr->ScheduleEvent(
            unique_ptr<SimulationEvent>(
                new PeriodicBasicSafetyMessageTransmissionEvent(this)),
            (currentTime + basicSafetyMessageInfo.transmissionInterval));
    }//if//
}//PeriodicallyTransmitBasicSafetyMessage//

inline
void DsrcMessageApplication::ReceivePacketFromLowerLayer(unique_ptr<Packet>& packetPtr)
{
    const DsrcMessageIdType messageId = packetPtr->GetAndReinterpretPayloadData<DsrcMessageIdType>();

    switch (messageId) {
    case DSRC_MESSAGE_A_LA_CARTE:
        (*this).ReceiveALaCarteMessage(packetPtr);
        break;

    case DSRC_MESSAGE_BASIC_SAFETY:
        (*this).ReceiveBasicSafetyMessage(packetPtr);
        break;

    default:
        assert("Received not supported DSRC message type");
        break;
    }//switch//

    packetPtr = nullptr;

}//ReceivePacketFromLowerLayer//

inline
void DsrcMessageApplication::ReceiveALaCarteMessage(unique_ptr<Packet>& packetPtr)
{
    packetPtr->DeleteHeader(sizeof(DsrcALaCarteMessageType));

    // nothing to do
}//ReceiveALaCarteMessage//

inline
void DsrcMessageApplication::ReceiveBasicSafetyMessage(unique_ptr<Packet>& packetPtr)
{
    const DsrcBasicSafetyMessagePart1Type part1 =
        packetPtr->GetAndReinterpretPayloadData<DsrcBasicSafetyMessagePart1Type>();

    if (packetPtr->LengthBytes() > sizeof(part1)) {
        const unsigned char* part2 =
            packetPtr->GetRawPayloadData(sizeof(part1), packetPtr->LengthBytes() - sizeof(part1));

    }//if//

    const DsrcPacketExtrinsicInformation& extInfo =
        packetPtr->GetExtrinsicPacketInformation<DsrcPacketExtrinsicInformation>(
            DsrcPacketExtrinsicInformation::id);

    const SimTime delay =
        simulationEngineInterfacePtr->CurrentTime() - extInfo.transmissionTime;

    //追加---------------------------------------------------------------
    const PacketId tmppacketId = packetPtr->GetPacketId();
    const NodeId destinationId = tmppacketId.GetSourceNodeId();

    /*const float SourceX = part1.GetXMeters();
    const float SourceY = part1.GetYMeters();

    const int Heading = static_cast<int>(part1.GetHeading());*/

    

    
    char fname26[30];
    std::string SourceXS;
    int SourceX;
    sprintf(fname26,"tmpX_%d.txt",destinationId);
    std::ifstream inputfile26(fname26);
    inputfile26 >> SourceXS;
    SourceX = atoi(SourceXS.c_str());

    char fname27[30];
    std::string SourceYS;
    int SourceY;
    sprintf(fname27,"tmpY_%d.txt",destinationId);
    std::ifstream inputfile27(fname27);
    inputfile27 >> SourceYS;
    SourceY = atoi(SourceYS.c_str());

    //パターン2
    /*char fname15[30];
    std::string destHeadS;
    int destHead;
    sprintf(fname15,"tmpHead_%d.txt",destinationId);
    std::ifstream inputfile15(fname15);
    inputfile15 >> destHeadS;
    destHead = atoi(destHeadS.c_str());*/
    //パターン2

    char fname28[30];
    std::string destinterflagS;
    int destinterflag;
    sprintf(fname28,"tmpIntersection_%d.txt",destinationId);
    std::ifstream inputfile28(fname28);
    inputfile28 >> destinterflagS;
    destinterflag = atoi(destinterflagS.c_str());
    

    //送信主のID取得
    //std::cout << "destinationId = " << (int)destinationId << endl;
    /*if(basicSafetyMessageInfo.MyNodeId == 1){
        std::cout << "SourceX = " << SourceX << endl;
        std::cout << "SourceY = " << SourceY << endl;
        std::cout << "Heading = " << destHead << endl;
    }*/

    /*if(destinationId == 1){
        std::cout << "transmissionTime = " << extInfo.transmissionTime << endl;
    }*/

    //読み取り
    char fname29[30];
    std::string destPriorityS;
    unsigned int destPriority;
    sprintf(fname29,"tmpPri_%d.txt",destinationId);
    std::ifstream inputfile29(fname29);
    inputfile29 >> destPriorityS;
    destPriority = atoi(destPriorityS.c_str());

    char fname30[30];
    std::string destSpeedS;
    unsigned int destSpeed;
    sprintf(fname30,"tmpSpeed_%d.txt",destinationId);
    std::ifstream inputfile30(fname30);
    inputfile30 >> destSpeedS;
    destSpeed = atoi(destSpeedS.c_str());

    /*char fname31[30];
    std::string destCriticalS;
    unsigned long long int destCritical;
    sprintf(fname31,"tmpCritical_%d.txt",destinationId);
    std::ifstream inputfile31(fname31);
    inputfile31 >> destCriticalS;
    destCritical = atoi(destCriticalS.c_str());*/
    /*if(basicSafetyMessageInfo.MyNodeId == 1601){
        std::cout << "destPri = " << destPriority << endl;
        std::cout << "delay = " << delay << endl;
    }*/

    basicSafetyMessageInfo.list[destinationId - 1].flag = true;
    basicSafetyMessageInfo.list[destinationId - 1].transmissiontime = extInfo.transmissionTime;
    basicSafetyMessageInfo.list[destinationId - 1].destPri = destPriority;
    //パターン1
    basicSafetyMessageInfo.list[destinationId - 1].speed = destSpeed;
    //パターン1

    //パターン2
    /*double Speed = destSpeed * 1000;

    if(destHead == 0){
        if(SourceY >= 10000 && SourceY <= 190000){
            double YD = 190000 - SourceY;
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection = YD / Speed;

        }else if(SourceY <= -10000 && SourceY >= -190000){
            double YD = -10000 + (SourceY * (-1));
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection = YD / Speed;

        }else{
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection = 0;

        }

    }else if(destHead == 90 || destHead == -270){
        if(SourceX >= 10000 && SourceX <= 190000){
            double XD = 190000 - SourceX;
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection  = XD / Speed;

        }else if(SourceX <= -10000 && SourceX >= -190000){
            double XD = -10000 + (SourceX * (-1));
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection  = XD / Speed;

        }else{
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection = 0;
        }

    }else if(destHead == 180 || destHead == -180){  
        if(SourceY >= 10000 && SourceY <= 190000){
            double YD = SourceY - 10000;
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection  = YD / Speed;

        }else if(SourceY <= -10000 && SourceY >= -190000){
            double YD = SourceY + 190000;
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection = YD / Speed;

        }else{
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection = 0;
        }

    }else if(destHead == -90 || destHead == 270){
        if(SourceX >= 10000 && SourceX <= 190000){
            double XD = SourceX - 10000;
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection = XD / Speed;

        }else if(SourceX <= -10000 && SourceX >= -190000){
            double XD = SourceX + 190000;
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection = XD / Speed;

        }else{
            basicSafetyMessageInfo.list[destinationId - 1].timetointersection = 0;
        }

    }else{
        basicSafetyMessageInfo.list[destinationId - 1].timetointersection = 0;
    }*/

        //if(basicSafetyMessageInfo.MyNodeId == 1){
            //std::cout << "intersectiontime = " << intersectiontime * 1000 << endl;
            //std::cout << "speed = " << currentSpeed << endl;

        //}
    //パターン2

    //SimTime tmpdelay = (double)delay / 1000.0 + 0.5;
    
    if(destinterflag == 1){
        basicSafetyMessageInfo.list[destinationId - 1].timetointersection = 0;
    }else{
        basicSafetyMessageInfo.list[destinationId - 1].timetointersection = 5000;
    }

    bool intersectionflag = false;

    if((abs(SourceX) <= 10000) && (abs(SourceY) <= 10000)){
    //受信車両50のみ
    //if((abs(SourceX) <= 10000) && (0 <= SourceY) && (SourceY <= 10000)){
        basicSafetyMessageInfo.numberPacketReceivedinintersection++;
        basicSafetyMessageInfo.delaysumInter += delay;
        //basicSafetyMessageInfo.delaysumInter += tmpdelay;
        intersectionflag = true;
        if(0 <= destSpeed && destSpeed <= 2777){
            basicSafetyMessageInfo.numberPacketReceivedSpeedInter[0]++;
            basicSafetyMessageInfo.delaysumSI[0] += delay;
            //basicSafetyMessageInfo.delaysumS[0] += tmpdelay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS0[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS0[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS0[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS0[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS0[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS0[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS0[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS0[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS0[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS0[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS0[10]++;
                    }
        }else if(2777 < destSpeed && destSpeed <= 5555){
            basicSafetyMessageInfo.numberPacketReceivedSpeedInter[1]++;
            basicSafetyMessageInfo.delaysumSI[1] += delay;   
            //basicSafetyMessageInfo.delaysumS[1] += tmpdelay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS1[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS1[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS1[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS1[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS1[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS1[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS1[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS1[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS1[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS1[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS1[10]++;
                    }
        }else if(5555 < destSpeed && destSpeed <= 8333){
            basicSafetyMessageInfo.numberPacketReceivedSpeedInter[2]++;
            basicSafetyMessageInfo.delaysumSI[2] += delay;
            //basicSafetyMessageInfo.delaysumS[2] += tmpdelay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS2[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS2[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS2[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS2[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS2[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS2[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS2[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS2[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS2[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS2[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS2[10]++;
                    }
        }else if(8333 < destSpeed && destSpeed <= 11111){
            basicSafetyMessageInfo.numberPacketReceivedSpeedInter[3]++;
            basicSafetyMessageInfo.delaysumSI[3] += delay;
            //basicSafetyMessageInfo.delaysumS[3] += tmpdelay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS3[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS3[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS3[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS3[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS3[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS3[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS3[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS3[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS3[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS3[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS3[10]++;
                    }
        }else if(11111 < destSpeed && destSpeed <= 13888){
            basicSafetyMessageInfo.numberPacketReceivedSpeedInter[4]++;
            basicSafetyMessageInfo.delaysumSI[4] += delay;
            //basicSafetyMessageInfo.delaysumS[4] += tmpdelay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS4[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS4[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS4[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS4[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS4[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS4[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS4[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS4[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS4[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS4[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS4[10]++;
                    }
        }else if(13888 < destSpeed && destSpeed <= 16667){
            basicSafetyMessageInfo.numberPacketReceivedSpeedInter[5]++;
            basicSafetyMessageInfo.delaysumSI[5] += delay;
            //basicSafetyMessageInfo.delaysumS[5] += tmpdelay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS5[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS5[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS5[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS5[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS5[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS5[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS5[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS5[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS5[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS5[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS0[10]++;
                    }
        }

            if(0 <= delay && delay <= 10000000){
                basicSafetyMessageInfo.delaydistribution[0]++;
            }else if(10000000 < delay && delay <= 20000000){
                basicSafetyMessageInfo.delaydistribution[1]++;
            }else if(20000000 < delay && delay <= 30000000){
                basicSafetyMessageInfo.delaydistribution[2]++;
            }else if(30000000 < delay && delay <= 40000000){
                basicSafetyMessageInfo.delaydistribution[3]++;
            }else if(40000000 < delay && delay <= 50000000){
                basicSafetyMessageInfo.delaydistribution[4]++;
            }else if(50000000 < delay && delay <= 60000000){
                basicSafetyMessageInfo.delaydistribution[5]++;
            }else if(60000000 < delay && delay <= 70000000){
                basicSafetyMessageInfo.delaydistribution[6]++;
            }else if(70000000 < delay && delay <= 80000000){
                basicSafetyMessageInfo.delaydistribution[7]++;
            }else if(80000000 < delay && delay <= 90000000){
                basicSafetyMessageInfo.delaydistribution[8]++;
            }else if(90000000 < delay && delay <= 100000000){
                basicSafetyMessageInfo.delaydistribution[9]++;
            }else{
                basicSafetyMessageInfo.delaydistribution[10]++;
            }

        if(destPriority == 1 || destPriority == 2){
            basicSafetyMessageInfo.numberPacketReceivedPriInter[0]++;
            basicSafetyMessageInfo.delaysumPI[0] += delay;
        }else if(destPriority == 0 || destPriority == 3){
            basicSafetyMessageInfo.numberPacketReceivedPriInter[1]++;
            basicSafetyMessageInfo.delaysumPI[1] += delay;
        }else if(destPriority == 4 || destPriority == 5){
            basicSafetyMessageInfo.numberPacketReceivedPriInter[2]++;
            basicSafetyMessageInfo.delaysumPI[2] += delay;
        }else if(destPriority == 6 || destPriority == 7){
            basicSafetyMessageInfo.numberPacketReceivedPriInter[3]++;
            basicSafetyMessageInfo.delaysumPI[3] += delay;
        }

        /*if(basicSafetyMessageInfo.delaymax < delay){
            basicSafetyMessageInfo.delaymax = delay;
        }*/

    }

    //std::cout << "transmissionTIme2 = " << basicSafetyMessageInfo.vehicular[destinationId - 1][1] << endl;

    /*if(basicSafetyMessageInfo.MyNodeId == 2){
        std::cout << "0 = " << basicSafetyMessageInfo.list[destinationId - 1].flag << endl;
        std::cout << "1 = " << basicSafetyMessageInfo.list[destinationId - 1].transmissiontime << endl;
        std::cout << "0-2 = " << basicSafetyMessageInfo.list[destinationId].flag << endl;
        std::cout << "1-2 = " << basicSafetyMessageInfo.list[destinationId].transmissiontime << endl;
        //行ける
    }*/
    //std::cout << basicSafetyMessageInfo.list[destinationId - 1].number << endl;
    
    unsigned long long int SourceSeq = tmppacketId.GetSourceNodeSequenceNumber();

   

    /*if(destCritical == SourceSeq){
        basicSafetyMessageInfo.list[destinationId - 1].numbercritical++;

    }else{*/
    if(intersectionflag != true){
        if(((-10000 <= SourceX) && (SourceX <= 10000) && (-50000 <= SourceY) && (SourceY <= 0)) || ((-10000 <= SourceX) && (SourceX <= 10000) && (0 <= SourceY) && (SourceY <= 50000))){
        //f(((-10000 <= SourceX) && (SourceX <= 10000) && (50000 <= SourceY) && (SourceY <= 100000)) || ((-10000 <= SourceX) && (SourceX <= 10000) && (100000 <= SourceY) && (SourceY <= 150000))){
        //if(((-10000 <= SourceX) && (SourceX <= 10000) && (-10000 <= SourceY) && (SourceY <= 40000)) || ((-10000 <= SourceX) && (SourceX <= 10000) && (40000 <= SourceY) && (SourceY <= 90000))){
        //if(((-10000 <= SourceY) && (SourceY <= 10000) && (50000 <= SourceX) && (SourceX <= 100000)) || ((-10000 <= SourceY) && (SourceY <= 10000) && (100000 <= SourceX) && (SourceX <= 150000))){
        //if(((-10000 <= SourceX) && (SourceX <= 10000) && (-50000 >= SourceY) && (SourceY >= -100000)) || ((-10000 <= SourceX) && (SourceX <= 10000) && (-100000 >= SourceY) && (SourceY >= -150000))){
        //if(((-10000 <= SourceY) && (SourceY <= 10000) && (-50000 >= SourceX) && (SourceX >= -100000)) || ((-10000 <= SourceY) && (SourceY <= 10000) && (-100000 >= SourceX) && (SourceX >= -150000))){

            if(destPriority == 1 || destPriority == 2){
                basicSafetyMessageInfo.list[destinationId - 1].number[0]++;
                basicSafetyMessageInfo.delaysumP[0] += delay;
                //basicSafetyMessageInfo.delaysumP[0] += tmpdelay;
            }else if(destPriority == 0 || destPriority == 3){
                basicSafetyMessageInfo.list[destinationId - 1].number[1]++;
                basicSafetyMessageInfo.delaysumP[1] += delay;
                //basicSafetyMessageInfo.delaysumP[1] += tmpdelay;
            }else if(destPriority == 4 || destPriority == 5){
                basicSafetyMessageInfo.list[destinationId - 1].number[2]++;
                basicSafetyMessageInfo.delaysumP[2] += delay;
                //basicSafetyMessageInfo.delaysumP[2] += tmpdelay;
            }else if(destPriority == 6 || destPriority == 7){
                basicSafetyMessageInfo.list[destinationId - 1].number[3]++;
                basicSafetyMessageInfo.delaysumP[3] += delay;
                //basicSafetyMessageInfo.delaysumP[3] += tmpdelay;
            }
            //if(intersectionflag == false){
                /*if(2777 <= destSpeed && destSpeed <= 5555){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[0]++;
                    basicSafetyMessageInfo.delaysumS[0] += delay;
                    //basicSafetyMessageInfo.delaysumS[0] += tmpdelay;
                }else if(5555 < destSpeed && destSpeed <= 8333){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[1]++;
                    basicSafetyMessageInfo.delaysumS[1] += delay;
                    //basicSafetyMessageInfo.delaysumS[1] += tmpdelay;
                }else if(8333 < destSpeed && destSpeed <= 11111){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[2]++;
                    basicSafetyMessageInfo.delaysumS[2] += delay;
                    //basicSafetyMessageInfo.delaysumS[2] += tmpdelay;
                }else if(11111 < destSpeed && destSpeed <= 13888){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[3]++;
                    basicSafetyMessageInfo.delaysumS[3] += delay;
                    //basicSafetyMessageInfo.delaysumS[3] += tmpdelay;
                }else if(13888 < destSpeed && destSpeed <= 16667){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[4]++;
                    basicSafetyMessageInfo.delaysumS[4] += delay;
                    //basicSafetyMessageInfo.delaysumS[4] += tmpdelay;
                }*/
            //}

                if(0 <= destSpeed && destSpeed <= 2777){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[0]++;
                    basicSafetyMessageInfo.delaysumS[0] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS0[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS0[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS0[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS0[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS0[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS0[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS0[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS0[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS0[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS0[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS0[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[0] += tmpdelay;
                }else if(2777 < destSpeed && destSpeed <= 5555){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[1]++;
                    basicSafetyMessageInfo.delaysumS[1] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS1[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS1[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS1[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS1[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS1[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS1[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS1[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS1[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS1[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS1[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS1[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[1] += tmpdelay;
                }else if(5555 < destSpeed && destSpeed <= 8333){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[2]++;
                    basicSafetyMessageInfo.delaysumS[2] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS2[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS2[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS2[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS2[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS2[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS2[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS2[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS2[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS2[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS2[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS2[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[2] += tmpdelay;
                }else if(8333 < destSpeed && destSpeed <= 11111){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[3]++;
                    basicSafetyMessageInfo.delaysumS[3] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS3[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS3[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS3[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS3[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS3[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS3[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS3[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS3[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS3[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS3[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS3[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[3] += tmpdelay;
                }else if(11111 < destSpeed && destSpeed <= 13888){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[4]++;
                    basicSafetyMessageInfo.delaysumS[4] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS4[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS4[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS4[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS4[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS4[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS4[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS4[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS4[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS4[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS4[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS4[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[4] += tmpdelay;
                }else if(13888 < destSpeed && destSpeed <= 16667){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[5]++;
                    basicSafetyMessageInfo.delaysumS[5] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS5[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS5[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS5[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS5[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS5[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS5[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS5[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS5[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS5[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS5[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS5[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[5] += tmpdelay;
                }

            if(0 <= delay && delay <= 10000000){
                basicSafetyMessageInfo.delaydistribution[0]++;
            }else if(10000000 < delay && delay <= 20000000){
                basicSafetyMessageInfo.delaydistribution[1]++;
            }else if(20000000 < delay && delay <= 30000000){
                basicSafetyMessageInfo.delaydistribution[2]++;
            }else if(30000000 < delay && delay <= 40000000){
                basicSafetyMessageInfo.delaydistribution[3]++;
            }else if(40000000 < delay && delay <= 50000000){
                basicSafetyMessageInfo.delaydistribution[4]++;
            }else if(50000000 < delay && delay <= 60000000){
                basicSafetyMessageInfo.delaydistribution[5]++;
            }else if(60000000 < delay && delay <= 70000000){
                basicSafetyMessageInfo.delaydistribution[6]++;
            }else if(70000000 < delay && delay <= 80000000){
                basicSafetyMessageInfo.delaydistribution[7]++;
            }else if(80000000 < delay && delay <= 90000000){
                basicSafetyMessageInfo.delaydistribution[8]++;
            }else if(90000000 < delay && delay <= 100000000){
                basicSafetyMessageInfo.delaydistribution[9]++;
            }

            if(basicSafetyMessageInfo.delaymax < delay){
                basicSafetyMessageInfo.delaymax = delay;
            }

            basicSafetyMessageInfo.delaysum += delay;
            if(basicSafetyMessageInfo.delaysum < 0){
                basicSafetyMessageInfo.alart = true;
            }
            //basicSafetyMessageInfo.delaysum += tmpdelay;
            

        //受信車両交差点のみ
        }else if(((-10000 <= SourceY) && (SourceY <= 10000) && (-50000 <= SourceX) && (SourceX <= 0)) || ((-10000 <= SourceY) && (SourceY <= 10000) && (0 <= SourceX) && (SourceX <= 50000))){
            if(destPriority == 1 || destPriority == 2){
                basicSafetyMessageInfo.list[destinationId - 1].number[0]++;
                basicSafetyMessageInfo.delaysumP[0] += delay;
            }else if(destPriority == 0 || destPriority == 3){
                basicSafetyMessageInfo.list[destinationId - 1].number[1]++;
                basicSafetyMessageInfo.delaysumP[1] += delay;
            }else if(destPriority == 4 || destPriority == 5){
                basicSafetyMessageInfo.list[destinationId - 1].number[2]++;
                basicSafetyMessageInfo.delaysumP[2] += delay;
            }else if(destPriority == 6 || destPriority == 7){
                basicSafetyMessageInfo.list[destinationId - 1].number[3]++;
                basicSafetyMessageInfo.delaysumP[3] += delay;
            }

            //if(intersectionflag == false){
                /*if(2777 <= destSpeed && destSpeed <= 5555){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[0]++;
                    basicSafetyMessageInfo.delaysumS[0] += delay;
                }else if(5555 < destSpeed && destSpeed <= 8333){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[1]++;
                    basicSafetyMessageInfo.delaysumS[1] += delay;
                }else if(8333 < destSpeed && destSpeed <= 11111){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[2]++;
                    basicSafetyMessageInfo.delaysumS[2] += delay;
                }else if(11111 < destSpeed && destSpeed <= 13888){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[3]++;
                    basicSafetyMessageInfo.delaysumS[3] += delay;
                }else if(13888 < destSpeed && destSpeed <= 16667){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[4]++;
                    basicSafetyMessageInfo.delaysumS[4] += delay;
                }*/
            //}
                //ここから
                if(0 <= destSpeed && destSpeed <= 2777){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[0]++;
                    basicSafetyMessageInfo.delaysumS[0] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS0[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS0[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS0[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS0[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS0[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS0[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS0[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS0[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS0[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS0[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS0[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[0] += tmpdelay;
                }else if(2777 < destSpeed && destSpeed <= 5555){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[1]++;
                    basicSafetyMessageInfo.delaysumS[1] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS1[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS1[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS1[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS1[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS1[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS1[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS1[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS1[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS1[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS1[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS1[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[1] += tmpdelay;
                }else if(5555 < destSpeed && destSpeed <= 8333){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[2]++;
                    basicSafetyMessageInfo.delaysumS[2] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS2[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS2[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS2[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS2[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS2[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS2[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS2[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS2[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS2[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS2[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS2[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[2] += tmpdelay;
                }else if(8333 < destSpeed && destSpeed <= 11111){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[3]++;
                    basicSafetyMessageInfo.delaysumS[3] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS3[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS3[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS3[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS3[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS3[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS3[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS3[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS3[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS3[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS3[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS3[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[3] += tmpdelay;
                }else if(11111 < destSpeed && destSpeed <= 13888){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[4]++;
                    basicSafetyMessageInfo.delaysumS[4] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS4[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS4[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS4[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS4[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS4[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS4[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS4[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS4[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS4[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS4[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS4[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[4] += tmpdelay;
                }else if(13888 < destSpeed && destSpeed <= 16667){
                    basicSafetyMessageInfo.numberPacketReceivedSpeed[5]++;
                    basicSafetyMessageInfo.delaysumS[5] += delay;
                    if(0 <= delay && delay <= 10000000){
                        basicSafetyMessageInfo.delaydistributionS5[0]++;
                    }else if(10000000 < delay && delay <= 20000000){
                        basicSafetyMessageInfo.delaydistributionS5[1]++;
                    }else if(20000000 < delay && delay <= 30000000){
                        basicSafetyMessageInfo.delaydistributionS5[2]++;
                    }else if(30000000 < delay && delay <= 40000000){
                        basicSafetyMessageInfo.delaydistributionS5[3]++;
                    }else if(40000000 < delay && delay <= 50000000){
                        basicSafetyMessageInfo.delaydistributionS5[4]++;
                    }else if(50000000 < delay && delay <= 60000000){
                        basicSafetyMessageInfo.delaydistributionS5[5]++;
                    }else if(60000000 < delay && delay <= 70000000){
                        basicSafetyMessageInfo.delaydistributionS5[6]++;
                    }else if(70000000 < delay && delay <= 80000000){
                        basicSafetyMessageInfo.delaydistributionS5[7]++;
                    }else if(80000000 < delay && delay <= 90000000){
                        basicSafetyMessageInfo.delaydistributionS5[8]++;
                    }else if(90000000 < delay && delay <= 100000000){
                        basicSafetyMessageInfo.delaydistributionS5[9]++;
                    }else{
                        basicSafetyMessageInfo.delaydistributionS5[10]++;
                    }
                    //basicSafetyMessageInfo.delaysumS[5] += tmpdelay;
                }


            if(0 <= delay && delay <= 10000000){
                basicSafetyMessageInfo.delaydistribution[0]++;
            }else if(10000000 < delay && delay <= 20000000){
                basicSafetyMessageInfo.delaydistribution[1]++;
            }else if(20000000 < delay && delay <= 30000000){
                basicSafetyMessageInfo.delaydistribution[2]++;
            }else if(30000000 < delay && delay <= 40000000){
                basicSafetyMessageInfo.delaydistribution[3]++;
            }else if(40000000 < delay && delay <= 50000000){
                basicSafetyMessageInfo.delaydistribution[4]++;
            }else if(50000000 < delay && delay <= 60000000){
                basicSafetyMessageInfo.delaydistribution[5]++;
            }else if(60000000 < delay && delay <= 70000000){
                basicSafetyMessageInfo.delaydistribution[6]++;
            }else if(70000000 < delay && delay <= 80000000){
                basicSafetyMessageInfo.delaydistribution[7]++;
            }else if(80000000 < delay && delay <= 90000000){
                basicSafetyMessageInfo.delaydistribution[8]++;
            }else if(90000000 < delay && delay <= 100000000){
                basicSafetyMessageInfo.delaydistribution[9]++;
            }else{
                basicSafetyMessageInfo.delaydistribution[10]++;
            }

            if(basicSafetyMessageInfo.delaymax < delay){
                basicSafetyMessageInfo.delaymax = delay;
            }

            basicSafetyMessageInfo.delaysum += delay;
            if(basicSafetyMessageInfo.delaysum < 0){
                basicSafetyMessageInfo.alart = true;
            }/**/
        }
    }

    /*if(basicSafetyMessageInfo.MyNodeId == 1601){
        std::cout << "S1 = " << (int)basicSafetyMessageInfo.delaysumS[1] << endl;
        std::cout << "S2 = " << (int)basicSafetyMessageInfo.delaysumS[2] << endl;
        std::cout << "S3 = " << (int)basicSafetyMessageInfo.delaysumS[3] << endl;
        std::cout << "S4 = " << (int)basicSafetyMessageInfo.delaysumS[4] << endl;
        std::cout << "S5 = " << (int)basicSafetyMessageInfo.delaysumS[5] << endl;
    }*/

    //}
    //std::cout << basicSafetyMessageInfo.list[destinationId - 1].number << endl;
    //basicSafetyMessageInfo.numberPacketsReceived++;

    //int caltotal = 0;

    /*if(basicSafetyMessageInfo.MyNodeId == 2){
        for(int i = 0; i < 1440; i++){
            caltotal += basicSafetyMessageInfo.list[i].number;
        }
        std::cout << "3   = " << basicSafetyMessageInfo.list[2].number << endl;
        std::cout << "1440 = " << basicSafetyMessageInfo.list[599].number << endl;
        std::cout << "caltotal = " << caltotal << endl;
        std::cout << "total = " << basicSafetyMessageInfo.numberPacketsReceived << endl;
    }*/
    if(basicSafetyMessageInfo.delaymax2 < delay){
        basicSafetyMessageInfo.delaymax2 = delay;
    }
    
    basicSafetyMessageInfo.numberPacketsReceived++;
    basicSafetyMessageInfo.delaysum2 += delay;
    //basicSafetyMessageInfo.delaysum2 += tmpdelay;
    

    //送信主が１の時だけ
    /*if(destinationId == 1){

        if((-10000 <= SourceX) && (SourceX <= 10000) && basicSafetyMessageInfo.list[destinationId - 1].destPri != 7){
            //601が受け取った１のパケット数
            if(basicSafetyMessageInfo.MyNodeId == 801){
                //basicSafetyMessageInfo.numberPacketsReceived++;
                basicSafetyMessageInfo.delaysum += delay;

                //遅延仮消し
                //basicSafetyMessageInfo.delayave = basicSafetyMessageInfo.delaysum / basicSafetyMessageInfo.list[destinationId - 1].number;

                //std::cout << "numberpacketreceived to 601 from 1 = " << (int)basicSafetyMessageInfo.numberPacketsReceived << endl;
                //std::cout << "delayfrom1 = " << delay << endl;
                    
                //std::cout << "packetreceiveave = " << (int)basicSafetyMessageInfo.numberPacketsReceived << endl;
                //std::cout << "Node =  " << basicSafetyMessageInfo.MyNodeId << " : delayaverage = " << basicSafetyMessageInfo.delayave << endl;

            }

        }else if((-10000 <= SourceY) && (SourceY <= 10000) && basicSafetyMessageInfo.list[destinationId - 1].destPri != 7){
            //601が受け取った１のパケット数
            if(basicSafetyMessageInfo.MyNodeId == 801){
                //basicSafetyMessageInfo.numberPacketsReceived++;
                basicSafetyMessageInfo.delaysum += delay;

                //遅延仮消し
                //basicSafetyMessageInfo.delayave = basicSafetyMessageInfo.delaysum / basicSafetyMessageInfo.list[destinationId - 1].number;

                //std::cout << "numberpacketreceived to 601 from 1 = " << (int)basicSafetyMessageInfo.numberPacketsReceived << endl;
                //std::cout << "delayfrom1 = " << delay << endl;
                    
                //std::cout << "packetreceiveave = " << (int)basicSafetyMessageInfo.numberPacketsReceived << endl;
                //std::cout << "Node =  " << basicSafetyMessageInfo.MyNodeId << " : delayaverage = " << basicSafetyMessageInfo.delayave << endl;

            }
        }else if((-10000 <= SourceX) && (SourceX <= 10000) && basicSafetyMessageInfo.list[destinationId - 1].destPri == 7){
            if(basicSafetyMessageInfo.MyNodeId == 601){
                //basicSafetyMessageInfo.numberPacketsReceived++;
                basicSafetyMessageInfo.delaysum2 += delay;

                //遅延仮消し
                //basicSafetyMessageInfo.delayave2 = basicSafetyMessageInfo.delaysum2 / basicSafetyMessageInfo.list[destinationId - 1].numbercritical;
                
                //std::cout << "numberpacketreceived to 601 from 1 = " << (int)basicSafetyMessageInfo.numberPacketsReceived << endl;
                //std::cout << "delayfrom1 = " << delay << endl;
                    
                //std::cout << "packetreceiveave = " << (int)basicSafetyMessageInfo.numberPacketsReceived << endl;
                //std::cout << "Node =  " << basicSafetyMessageInfo.MyNodeId << " : delayaverage = " << basicSafetyMessageInfo.delayave << endl;

            }
        }else if((-10000 <= SourceY) && (SourceY <= 10000) && basicSafetyMessageInfo.list[destinationId - 1].destPri == 7){
            if(basicSafetyMessageInfo.MyNodeId == 601){
                //basicSafetyMessageInfo.numberPacketsReceived++;
                basicSafetyMessageInfo.delaysum2 += delay;

                //遅延仮消し
                //basicSafetyMessageInfo.delayave2 = basicSafetyMessageInfo.delaysum2 / basicSafetyMessageInfo.list[destinationId - 1].numbercritical;
                
                //std::cout << "numberpacketreceived to 601 from 1 = " << (int)basicSafetyMessageInfo.numberPacketsReceived << endl;
                //std::cout << "delayfrom1 = " << delay << endl;
                    
                //std::cout << "packetreceiveave = " << (int)basicSafetyMessageInfo.numberPacketsReceived << endl;
                //std::cout << "Node =  " << basicSafetyMessageInfo.MyNodeId << " : delayaverage = " << basicSafetyMessageInfo.delayave << endl;

            }
        }*/
        //11が受け取った１のパケット数
        /*if(basicSafetyMessageInfo.MyNodeId == 11){
            //std::cout << "numberpacketreceivedto11 = " << (int)basicSafetyMessageInfo.numberPacketsReceived << endl;
            //std::cout << "delayfrom2 = " << delay << endl;
             std::cout << "Node =  " << basicSafetyMessageInfo.MyNodeId << " : delayaverage = " << basicSafetyMessageInfo.delayave << endl;
        }*/
        

        
    //}

    /*if(destinationId == 3){
        
        //10が受け取った１のパケット数
        if(basicSafetyMessageInfo.MyNodeId == 2){
            basicSafetyMessageInfo.numberPacketsReceived2++;
            basicSafetyMessageInfo.delaysum2 += delay;
            basicSafetyMessageInfo.delayave2 = basicSafetyMessageInfo.delaysum2 / basicSafetyMessageInfo.numberPacketsReceived2;
            //std::cout << "numberpacketreceivedto10from2 = " << (int)basicSafetyMessageInfo.numberPacketsReceived2 << endl;
            //std::cout << "delayfrom1 = " << delay << endl;
            //basicSafetyMessageInfo.delaysum += delay;
            //basicSafetyMessageInfo.delayave = basicSafetyMessageInfo.delaysum / basicSafetyMessageInfo.numberPacketsReceived;
            //std::cout << "delayavefrom1 = " << basicSafetyMessageInfo.delayave << endl;
            std::cout << "3packetreceiveave = " << (int)basicSafetyMessageInfo.numberPacketsReceived2 << endl;
            std::cout << "3Node =  " << basicSafetyMessageInfo.MyNodeId << " : delayaverage = " << basicSafetyMessageInfo.delayave2 << endl;

        }
    }*/
    /*if(destinationId == 2){
        basicSafetyMessageInfo.numberPacketsReceived2++;
        if(basicSafetyMessageInfo.MyNodeId == 1){
            std::cout << "numberpacketreceivedfrom2 = " << (int)basicSafetyMessageInfo.numberPacketsReceived2 << endl;
            //std::cout << "delayfrom2 = " << delay << endl;
        }
    }*/

    //if文の中以外-----------------------------------------------------------

    //送信主が１の時だけ受信カウント
    //if(basicSafetyMessageInfo.MyNodeId == 10){
        //std::cout << "numberpacketreceivedfrom1 = " << (int)basicSafetyMessageInfo.numberPacketsReceived << endl;
        //std::cout << "numberpacketreceivedfrom3 = " << (int)basicSafetyMessageInfo.numberPacketsReceived2 << endl;
    //}

    (*this).OutputTraceAndStatsForReceiveBasicSafetyMessage(
        extInfo.sequenceNumber,
        packetPtr->GetPacketId(),
        packetPtr->LengthBytes(),
        delay);
}//ReceiveBasicSafetyMessage//

inline
void DsrcMessageApplication::OutputTraceAndStatsForSendBasicSafetyMessage(
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const size_t packetLengthBytes)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationSendTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.destinationNodeId = ANY_NODEID;
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();

            assert(sizeof(traceData) == APPLICATION_SEND_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                basicSafetyAppModelName,
                "",
                "BsmSend",
                traceData);

        } else {

            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId;

            simulationEngineInterfacePtr->OutputTrace(
                basicSafetyAppModelName,
                "",
                "BsmSend",
                outStream.str());
        }//if//
    }//if//

    basicSafetyMessageInfo.packetsSentStatPtr->IncrementCounter();
    basicSafetyMessageInfo.bytesSentStatPtr->IncrementCounter(packetLengthBytes);
}//OutputTraceAndStatsForSendBasicSafetyMessage//

inline
void DsrcMessageApplication::OutputTraceAndStatsForReceiveBasicSafetyMessage(
    const unsigned int sequenceNumber,
    const PacketId& thePacketId,
    const size_t packetLengthBytes,
    const SimTime& delay)
{
    if (simulationEngineInterfacePtr->TraceIsOn(TraceApplication)) {
        if (simulationEngineInterfacePtr->BinaryOutputIsOn()) {

            ApplicationReceiveTraceRecord traceData;

            traceData.packetSequenceNumber = sequenceNumber;
            traceData.sourceNodeId = thePacketId.GetSourceNodeId();
            traceData.sourceNodeSequenceNumber = thePacketId.GetSourceNodeSequenceNumber();
            traceData.delay = delay;
            traceData.receivedPackets = basicSafetyMessageInfo.numberPacketsReceived;
            traceData.packetLengthBytes = static_cast<uint16_t>(packetLengthBytes);

            assert(sizeof(traceData) == APPLICATION_RECEIVE_TRACE_RECORD_BYTES);

            simulationEngineInterfacePtr->OutputTraceInBinary(
                basicSafetyAppModelName,
                "",
                "BsmRecv",
                traceData);

        } else {
            ostringstream outStream;

            outStream << "Seq= " << sequenceNumber << " PktId= " << thePacketId
                      << " Delay= " << ConvertTimeToStringSecs(delay)
                      << " Pdr= " << basicSafetyMessageInfo.numberPacketsReceived << '/' << sequenceNumber
                      << " PacketBytes= " << packetLengthBytes;


            simulationEngineInterfacePtr->OutputTrace(
                basicSafetyAppModelName,
                "",
                "BsmRecv",
                outStream.str());
        }//if//
    }//if//

    basicSafetyMessageInfo.packetsReceivedStatPtr->IncrementCounter();
    basicSafetyMessageInfo.bytesReceivedStatPtr->IncrementCounter(packetLengthBytes);
    basicSafetyMessageInfo.endToEndDelayStatPtr->RecordStatValue(ConvertTimeToDoubleSecs(delay));
}//OutputTraceAndStatsForReceiveBasicSafetyMessage//

} //namespace Wave//

#endif

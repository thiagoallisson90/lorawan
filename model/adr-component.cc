/*
 * Copyright (c) 2018 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Matteo Perin <matteo.perin.2@studenti.unipd.it
 */

#include "adr-component.h"

#include <cmath>
#include <sstream>

namespace ns3
{
namespace lorawan
{

std::vector<std::string> split(const std::string& str, char delim) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string item;

    while (std::getline(ss, item, delim)) {
        tokens.push_back(item);
    }

    return tokens;
}

////////////////////////////////////////
// LinkAdrRequest commands management //
////////////////////////////////////////

NS_LOG_COMPONENT_DEFINE("AdrComponent");

NS_OBJECT_ENSURE_REGISTERED(AdrComponent);

TypeId
AdrComponent::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::AdrComponent")
            .SetGroupName("lorawan")
            .AddConstructor<AdrComponent>()
            .SetParent<NetworkControllerComponent>()
            .AddAttribute("MultipleGwCombiningMethod",
                          "Whether to average the received power of gateways or to use the maximum",
                          EnumValue(AdrComponent::AVERAGE),
                          MakeEnumAccessor<CombiningMethod>(&AdrComponent::tpAveraging),
                          MakeEnumChecker(AdrComponent::AVERAGE,
                                          "avg",
                                          AdrComponent::MAXIMUM,
                                          "max",
                                          AdrComponent::MINIMUM,
                                          "min"))
            .AddAttribute("MultiplePacketsCombiningMethod",
                          "Whether to average SNRs from multiple packets or to use the maximum",
                          EnumValue(AdrComponent::AVERAGE),
                          MakeEnumAccessor<CombiningMethod>(&AdrComponent::historyAveraging),
                          MakeEnumChecker(AdrComponent::AVERAGE,
                                          "avg",
                                          AdrComponent::MAXIMUM,
                                          "max",
                                          AdrComponent::MINIMUM,
                                          "min"))
            .AddAttribute("HistoryRange",
                          "Number of packets to use for averaging",
                          IntegerValue(4),
                          MakeIntegerAccessor(&AdrComponent::historyRange),
                          MakeIntegerChecker<int>(0, 100))
            .AddAttribute("ChangeTransmissionPower",
                          "Whether to toggle the transmission power or not",
                          BooleanValue(true),
                          MakeBooleanAccessor(&AdrComponent::m_toggleTxPower),
                          MakeBooleanChecker())
            .AddAttribute("Margin",
                "Dmargin value",
                DoubleValue(10),
                MakeDoubleAccessor(&AdrComponent::m_margin),
                MakeDoubleChecker<double>(0, 20));
    return tid;
}

AdrComponent::AdrComponent()
{
}

AdrComponent::~AdrComponent()
{
}

void
AdrComponent::OnReceivedPacket(Ptr<const Packet> packet,
                               Ptr<EndDeviceStatus> status,
                               Ptr<NetworkStatus> networkStatus)
{
    NS_LOG_FUNCTION(this->GetTypeId() << packet << networkStatus);

    // We will only act just before reply, when all Gateways will have received
    // the packet, since we need their respective received power.
}

void
AdrComponent::BeforeSendingReply(Ptr<EndDeviceStatus> status, Ptr<NetworkStatus> networkStatus)
{
    NS_LOG_FUNCTION(this << status << networkStatus);

    Ptr<Packet> myPacket = status->GetLastPacketReceivedFromDevice()->Copy();
    LorawanMacHeader mHdr;
    LoraFrameHeader fHdr;
    fHdr.SetAsUplink();
    myPacket->RemoveHeader(mHdr);
    myPacket->RemoveHeader(fHdr);

    // Execute the Adaptive Data Rate (ADR) algorithm only if the request bit is set
    if (fHdr.GetAdr())
    {
        if (int(status->GetReceivedPacketList().size()) < historyRange)
        {
            NS_LOG_ERROR("Not enough packets received by this device ("
                         << status->GetReceivedPacketList().size()
                         << ") for the algorithm to work (need " << historyRange << ")");
        }
        else
        {
            NS_LOG_DEBUG("New Adaptive Data Rate (ADR) request");

            // Get the spreading factor used by the device
            uint8_t spreadingFactor = status->GetFirstReceiveWindowSpreadingFactor();

            // Get the device transmission power (dBm)
            uint8_t transmissionPower = status->GetMac()->GetTransmissionPower();

            // New parameters for the end-device
            uint8_t newDataRate;
            uint8_t newTxPower;

            // Adaptive Data Rate (ADR) Algorithm
            AdrImplementation(&newDataRate, &newTxPower, status);

            // Change the power back to the default if we don't want to change it
            if (!m_toggleTxPower)
            {
                newTxPower = transmissionPower;
            }

            if (newDataRate != SfToDr(spreadingFactor) || newTxPower != transmissionPower)
            {
                // Create a list with mandatory channel indexes
                int channels[] = {0, 1, 2};
                std::list<int> enabledChannels(channels, channels + sizeof(channels) / sizeof(int));

                // Repetitions Setting
                const int rep = 1;

                NS_LOG_DEBUG("Sending LinkAdrReq with DR = " << (unsigned)newDataRate
                                                             << " and TP = " << (unsigned)newTxPower
                                                             << " dBm");

                status->m_reply.frameHeader.AddLinkAdrReq(newDataRate,
                                                          GetTxPowerIndex(newTxPower),
                                                          enabledChannels,
                                                          rep);
                status->m_reply.frameHeader.SetAsDownlink();
                status->m_reply.macHeader.SetMType(LorawanMacHeader::UNCONFIRMED_DATA_DOWN);

                status->m_reply.needsReply = true;
            }
            else
            {
                NS_LOG_DEBUG("Skipped request");
            }
        }
    }
    else
    {
        // Do nothing
    }
}

void
AdrComponent::OnFailedReply(Ptr<EndDeviceStatus> status, Ptr<NetworkStatus> networkStatus)
{
    NS_LOG_FUNCTION(this->GetTypeId() << networkStatus);
}

void
AdrComponent::AdrImplementation(uint8_t* newDataRate,
                                uint8_t* newTxPower,
                                Ptr<EndDeviceStatus> status)
{
    // Compute the maximum or median SNR, based on the boolean value historyAveraging
    double m_SNR = 0;
    switch (historyAveraging)
    {
    case AdrComponent::AVERAGE:
        m_SNR = GetAverageSNR(status->GetReceivedPacketList(), historyRange);
        break;
    case AdrComponent::MAXIMUM:
        m_SNR = GetMaxSNR(status->GetReceivedPacketList(), historyRange);
        break;
    case AdrComponent::MINIMUM:
        m_SNR = GetMinSNR(status->GetReceivedPacketList(), historyRange);
    }

    // std::cout << "HAvg = " << historyAveraging << ", margin = " << m_margin << std::endl;

    NS_LOG_DEBUG("m_SNR = " << m_SNR);

    // Get the spreading factor used by the device
    uint8_t spreadingFactor = status->GetFirstReceiveWindowSpreadingFactor();

    NS_LOG_DEBUG("SF = " << (unsigned)spreadingFactor);

    // Get the device data rate and use it to get the SNR demodulation threshold
    double req_SNR = threshold[SfToDr(spreadingFactor)];

    NS_LOG_DEBUG("Required SNR = " << req_SNR);

    // Get the device transmission power (dBm)
    double transmissionPower = status->GetMac()->GetTransmissionPower();

    NS_LOG_DEBUG("Transmission Power = " << transmissionPower);

    // Compute the SNR margin taking into consideration the SNR of
    // previously received packets
    double margin_SNR = m_SNR - req_SNR - m_margin;

    NS_LOG_DEBUG("Margin = " << margin_SNR);

    // Number of steps to decrement the spreading factor (thereby increasing the data rate)
    // and the TP.
    int steps = std::floor(margin_SNR / 3);

    NS_LOG_DEBUG("steps = " << steps);

    // If the number of steps is positive (margin_SNR is positive, so its
    // decimal value is high) increment the data rate, if there are some
    // leftover steps after reaching the maximum possible data rate
    //(corresponding to the minimum spreading factor) decrement the transmission power as
    // well for the number of steps left.
    // If, on the other hand, the number of steps is negative (margin_SNR is
    // negative, so its decimal value is low) increase the transmission power
    //(note that the spreading factor is not incremented as this particular algorithm
    // expects the node itself to raise its spreading factor whenever necessary).
    while (steps > 0 && spreadingFactor > min_spreadingFactor)
    {
        spreadingFactor--;
        steps--;
        NS_LOG_DEBUG("Decreased SF by 1");
    }
    while (steps > 0 && transmissionPower > min_transmissionPower)
    {
        transmissionPower -= 2;
        steps--;
        NS_LOG_DEBUG("Decreased Ptx by 2");
    }
    while (steps < 0 && transmissionPower < max_transmissionPower)
    {
        transmissionPower += 2;
        steps++;
        NS_LOG_DEBUG("Increased Ptx by 2");
    }

    *newDataRate = SfToDr(spreadingFactor);
    *newTxPower = transmissionPower;
}

uint8_t
AdrComponent::SfToDr(uint8_t sf)
{
    switch (sf)
    {
    case 12:
        return 0;
        break;
    case 11:
        return 1;
        break;
    case 10:
        return 2;
        break;
    case 9:
        return 3;
        break;
    case 8:
        return 4;
        break;
    default:
        return 5;
        break;
    }
}

double
AdrComponent::RxPowerToSNR(double transmissionPower) const
{
    // The following conversion ignores interfering packets
    return transmissionPower + 174 - 10 * log10(B) - NF;
}

// Get the maximum received power (it considers the values in dB!)
double
AdrComponent::GetMinTxFromGateways(EndDeviceStatus::GatewayList gwList)
{
    auto it = gwList.begin();
    double min = it->second.rxPower;

    for (; it != gwList.end(); it++)
    {
        if (it->second.rxPower < min)
        {
            min = it->second.rxPower;
        }
    }

    return min;
}

// Get the maximum received power (it considers the values in dB!)
double
AdrComponent::GetMaxTxFromGateways(EndDeviceStatus::GatewayList gwList)
{
    auto it = gwList.begin();
    double max = it->second.rxPower;

    for (; it != gwList.end(); it++)
    {
        if (it->second.rxPower > max)
        {
            max = it->second.rxPower;
        }
    }

    return max;
}

// Get the maximum received power
double
AdrComponent::GetAverageTxFromGateways(EndDeviceStatus::GatewayList gwList)
{
    double sum = 0;

    for (auto it = gwList.begin(); it != gwList.end(); it++)
    {
        NS_LOG_DEBUG("Gateway at " << it->first << " has TP " << it->second.rxPower);
        sum += it->second.rxPower;
    }

    double average = sum / gwList.size();

    NS_LOG_DEBUG("TP (average) = " << average);

    return average;
}

double
AdrComponent::GetReceivedPower(EndDeviceStatus::GatewayList gwList)
{
    switch (tpAveraging)
    {
    case AdrComponent::AVERAGE:
        return GetAverageTxFromGateways(gwList);
    case AdrComponent::MAXIMUM:
        return GetMaxTxFromGateways(gwList);
    case AdrComponent::MINIMUM:
        return GetMinTxFromGateways(gwList);
    default:
        return -1;
    }
}

// TODO Make this more elegant
double
AdrComponent::GetMinSNR(EndDeviceStatus::ReceivedPacketList packetList, int historyRange)
{
    double m_SNR;

    // Take elements from the list starting at the end
    auto it = packetList.rbegin();
    double min = RxPowerToSNR(GetReceivedPower(it->second.gwList));

    for (int i = 0; i < historyRange; i++, it++)
    {
        m_SNR = RxPowerToSNR(GetReceivedPower(it->second.gwList));

        NS_LOG_DEBUG("Received power: " << GetReceivedPower(it->second.gwList));
        NS_LOG_DEBUG("m_SNR = " << m_SNR);

        if (m_SNR < min)
        {
            min = m_SNR;
        }
    }

    NS_LOG_DEBUG("SNR (min) = " << min);

    return min;
}

double
AdrComponent::GetMaxSNR(EndDeviceStatus::ReceivedPacketList packetList, int historyRange)
{
    double m_SNR;

    // Take elements from the list starting at the end
    auto it = packetList.rbegin();
    double max = RxPowerToSNR(GetReceivedPower(it->second.gwList));

    for (int i = 0; i < historyRange; i++, it++)
    {
        m_SNR = RxPowerToSNR(GetReceivedPower(it->second.gwList));

        NS_LOG_DEBUG("Received power: " << GetReceivedPower(it->second.gwList));
        NS_LOG_DEBUG("m_SNR = " << m_SNR);

        if (m_SNR > max)
        {
            max = m_SNR;
        }
    }

    NS_LOG_DEBUG("SNR (max) = " << max);

    return max;
}

double
AdrComponent::GetAverageSNR(EndDeviceStatus::ReceivedPacketList packetList, int historyRange)
{
    double sum = 0;
    double m_SNR;

    // Take elements from the list starting at the end
    auto it = packetList.rbegin();
    for (int i = 0; i < historyRange; i++, it++)
    {
        m_SNR = RxPowerToSNR(GetReceivedPower(it->second.gwList));

        NS_LOG_DEBUG("Received power: " << GetReceivedPower(it->second.gwList));
        NS_LOG_DEBUG("m_SNR = " << m_SNR);

        sum += m_SNR;
    }

    double average = sum / historyRange;

    NS_LOG_DEBUG("SNR (average) = " << average);

    return average;
}

int
AdrComponent::GetTxPowerIndex(int txPower)
{
    if (txPower >= 16)
    {
        return 0;
    }
    else if (txPower >= 14)
    {
        return 1;
    }
    else if (txPower >= 12)
    {
        return 2;
    }
    else if (txPower >= 10)
    {
        return 3;
    }
    else if (txPower >= 8)
    {
        return 4;
    }
    else if (txPower >= 6)
    {
        return 5;
    }
    else if (txPower >= 4)
    {
        return 6;
    }
    else
    {
        return 7;
    }
}

// CAADR
NS_OBJECT_ENSURE_REGISTERED(CAADR);

TypeId
CAADR::GetTypeId()
{
  static TypeId tid =
        TypeId("ns3::CAADR")
            .SetGroupName("lorawan")
            .AddConstructor<CAADR>()
            .SetParent<AdrComponent>()
            .AddAttribute("Interval",
                          "Interval of message transmission in seconds",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&CAADR::m_interval),
                          MakeDoubleChecker<double>(1.0, 48 * 60 * 60))
            .AddAttribute("Prob",
                          "Success Probability",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&CAADR::m_prob),
                          MakeDoubleChecker<double>(0.01, 1.0))
            .AddAttribute("Run",
                          "Number of Running",
                          IntegerValue(1),
                          MakeIntegerAccessor(&CAADR::m_nRun),
                          MakeIntegerChecker<int>(1))
            .AddAttribute("ToAs",
                          "ToAs for SFs",
                          StringValue(""),
                          MakeStringAccessor(&CAADR::m_sToas),
                          MakeStringChecker());

  return tid;
}

CAADR::CAADR()
{
    // m_toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};
    m_nMax = {1, 1, 1, 1, 1, 1};
    m_n = {0, 0, 0, 0, 0, 0,};
}

CAADR::~CAADR()
{
    m_toas.clear();
    m_nMax.clear();
    m_n.clear();
}

void 
CAADR::SetToas(std::vector<double> toas)
{
    m_toas = toas;
}

double 
CAADR::Log(double base, double value)
{
    return std::log(value) / std::log(base);
}

int 
CAADR::NumMaxOfNodesPerSF(double toa, double succProb, int nFreq)
{
    double base = (1 - toa / m_interval);
    double numMax = 0.5 * Log(base, succProb);
    numMax = std::floor(numMax + 1) * nFreq;
    if (numMax < 0)
    {
        numMax = 0;
    }
    return numMax;
}

double 
CAADR::GetAveragePr(EndDeviceStatus::ReceivedPacketList packetList, int historyRange)
{
    double sum = 0;
    double m_power;

    // Take elements from the list starting at the end
    auto it = packetList.rbegin();
    for (int i = 0; i < historyRange; i++, it++)
    {
        m_power = GetReceivedPower(it->second.gwList);

        NS_LOG_DEBUG("Received power: " << GetReceivedPower(it->second.gwList));
        NS_LOG_DEBUG("m_SNR = " << m_power);

        sum += m_power;
    }

    double average = sum / historyRange;

    NS_LOG_DEBUG("Power (average) = " << average);

    return average;
}

uint8_t 
CAADR::SelectSF(double power)
{
    if (power >= -130.0)
    {
        return 7;
    }
    if (power >= -132.5)
    {
        return 8;
    }
    if (power >= -135.0)
    {
        return 9;
    }
    if (power >= -137.5)
    {
        return 10;
    }
    if (power >= -140.0)
    {
        return 11;
    }

    return 12;
}

void 
CAADR::AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status)
{
    if (m_toas.size() == 0 && m_sToas != "")
    {
        std::vector<std::string> toas = split(m_sToas, ',');
        for (auto toa: toas)
        {
            m_toas.push_back(std::stod(toa));
        }
    }
    else if (m_toas.size() == 0)
    {
        // ToAs for 51-Byte packet.
        m_toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};
    }

    // Get the spreading factor used by the device
    uint8_t spreadingFactor = status->GetFirstReceiveWindowSpreadingFactor();

    double decrementProb = 0.01;

    // Calcula potência média recebida
    double m_power = GetAveragePr(status->GetReceivedPacketList(), historyRange);

    // Seleciona o menor SF com PR >= PR_min
    uint8_t newSF = SelectSF(m_power); // SelectSF já cuida da checagem PR >= PR_min

    auto it1 = m_SfPerEd.find(status->GetMac()->GetDeviceAddress());
    if (it1 != m_SfPerEd.end())
    {
        if (spreadingFactor == newSF)
        {
            // Mesmo SF de antes, não faz nada
            *newDataRate = SfToDr(spreadingFactor);
            *newTxPower = status->GetMac()->GetTransmissionPower();
            return;
        }
    }

    // Verifica se SF ainda pode receber mais EDs (Se sim, n < n_max)
    if ((m_n[newSF - 7] >= m_nMax[newSF - 7]))
    {
        // Tenta aumentar o SF até encontrar um com espaço
        while (m_n[newSF - 7] >= m_nMax[newSF - 7] && newSF < 12)
        {
            newSF++;
        }
        if (newSF > 12)
        {
            newSF = 12;
        }

        // Se não há mais SFs disponíveis, reduz probabilidade e recalcula n_max
        if (m_n[newSF - 7] >= m_nMax[newSF - 7])
        {
            newSF = SelectSF(m_power);
            if (m_prob > decrementProb)
            {
                m_prob -= decrementProb;

                //std::cout << " Prob: " << m_prob << std::endl;

                for (size_t i = 0; i < m_toas.size(); ++i)
                {   
                    m_nMax[i] = NumMaxOfNodesPerSF(m_toas[i], m_prob);
                    //std::cout << "SF" << (i + 7) << " nMax: " << m_nMax[i] << "; ";
                }
                //std::cout << std::endl << std::endl;
            }
        }
    }

    // Atualiza mapa SF por dispositivo e vetor de contagem
    auto it = m_SfPerEd.find(status->GetMac()->GetDeviceAddress());
    if (it == m_SfPerEd.end())
    {
        // Primeiro acesso desse ED
        m_SfPerEd[status->GetMac()->GetDeviceAddress()] = newSF;
        m_n[newSF - 7]++;
    }
    else if (it->second != newSF)
    {
        // Troca de SF
        if (m_n[it->second - 7] > 0)
        {
            m_n[it->second - 7]--;
        }
        m_n[newSF - 7]++;
        it->second = newSF;
    }

    // Atualiza data rate e potência
    *newDataRate = SfToDr(newSF);
    //*newTxPower = 14;
    *newTxPower = status->GetMac()->GetTransmissionPower();
}

// GADR
NS_OBJECT_ENSURE_REGISTERED(GADR);

TypeId
GADR::GetTypeId()
{
  static TypeId tid =
        TypeId("ns3::GADR")
            .SetGroupName("lorawan")
            .AddConstructor<GADR>()
            .SetParent<AdrComponent>();
  
  return tid;
}

GADR::GADR()
{
    historyRange = 20;
}

GADR::~GADR()
{
}

double
GADR::CalcStd(double m_SNR, 
              EndDeviceStatus::ReceivedPacketList packetList, 
              int historyRange)
{
    double sumOfSquares = 0.0;

    // Take elements from the list starting at the end
    auto it = packetList.rbegin();
    for (int i = 0; i < historyRange; i++, it++)
    {
        double SNR = RxPowerToSNR(GetReceivedPower(it->second.gwList));
        sumOfSquares += std::pow(SNR - m_SNR, 2);
    }

    double std_SNR = std::sqrt(sumOfSquares / (historyRange - 1));

    return std_SNR;
}

void 
GADR::AdrImplementation(uint8_t* newDataRate,
                        uint8_t* newTxPower,
                        Ptr<EndDeviceStatus> status)
{
    double mean = GetAverageSNR(status->GetReceivedPacketList(), historyRange);
    double std_SNR = CalcStd(mean, status->GetReceivedPacketList(), historyRange);
    double m_SNR = 0;

    EndDeviceStatus::ReceivedPacketList packetList = status->GetReceivedPacketList();
    auto it = packetList.rbegin();
    int nMax = 0;
    for (int i = 0; i < historyRange; i++, it++)
    {
        double SNR = RxPowerToSNR(GetReceivedPower(it->second.gwList));
        
        if (SNR >= mean - std_SNR && SNR <= mean + std_SNR)
        {
            m_SNR += SNR;
            nMax++;
        }
    }

    m_SNR /= nMax;

    NS_LOG_DEBUG("m_SNR = " << m_SNR);

    // Get the spreading factor used by the device
    uint8_t spreadingFactor = status->GetFirstReceiveWindowSpreadingFactor();

    NS_LOG_DEBUG("SF = " << (unsigned)spreadingFactor);

    // Get the device data rate and use it to get the SNR demodulation threshold
    double req_SNR = threshold[SfToDr(spreadingFactor)];

    NS_LOG_DEBUG("Required SNR = " << req_SNR);

    // Get the device transmission power (dBm)
    double transmissionPower = status->GetMac()->GetTransmissionPower();

    NS_LOG_DEBUG("Transmission Power = " << transmissionPower);

    // Compute the SNR margin taking into consideration the SNR of
    // previously received packets
    double margin_SNR = m_SNR - req_SNR - m_margin;

    NS_LOG_DEBUG("Margin = " << margin_SNR);

    // Number of steps to decrement the spreading factor (thereby increasing the data rate)
    // and the TP.
    int steps = std::floor(margin_SNR / 3);

    NS_LOG_DEBUG("steps = " << steps);

    // If the number of steps is positive (margin_SNR is positive, so its
    // decimal value is high) increment the data rate, if there are some
    // leftover steps after reaching the maximum possible data rate
    //(corresponding to the minimum spreading factor) decrement the transmission power as
    // well for the number of steps left.
    // If, on the other hand, the number of steps is negative (margin_SNR is
    // negative, so its decimal value is low) increase the transmission power
    //(note that the spreading factor is not incremented as this particular algorithm
    // expects the node itself to raise its spreading factor whenever necessary).
    while (steps > 0 && spreadingFactor > min_spreadingFactor)
    {
        spreadingFactor--;
        steps--;
        NS_LOG_DEBUG("Decreased SF by 1");
    }
    while (steps > 0 && transmissionPower > min_transmissionPower)
    {
        transmissionPower -= 2;
        steps--;
        NS_LOG_DEBUG("Decreased Ptx by 2");
    }
    while (steps < 0 && transmissionPower < max_transmissionPower)
    {
        transmissionPower += 2;
        steps++;
        NS_LOG_DEBUG("Increased Ptx by 2");
    }

    *newDataRate = SfToDr(spreadingFactor);
    *newTxPower = transmissionPower;
}

// RADR
NS_OBJECT_ENSURE_REGISTERED(RADR);

TypeId
RADR::GetTypeId()
{
  static TypeId tid =
        TypeId("ns3::RADR")
            .SetGroupName("lorawan")
            .AddConstructor<RADR>()
            .SetParent<CAADR>();
  
  return tid;
}

RADR::RADR()
{
    // m_toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963}; // payload of 51B
}

RADR::~RADR()
{
    m_toas.clear();
    m_nMax.clear();

    for (auto it: m_gwInfos)
    {
        it.second.Clear();
    }
    m_gwInfos.clear();
}

EndDeviceStatus::PacketInfoPerGw
RADR::GetMaxSnrAndBestGw(EndDeviceStatus::GatewayList gwList)
{
    auto it = gwList.begin();
    double max = it->second.rxPower;
    EndDeviceStatus::PacketInfoPerGw bestInfo = it->second;

    for (; it != gwList.end(); it++)
    {
        if (it->second.rxPower > max)
        {
            max = it->second.rxPower;
            bestInfo = it->second;
        }
    }

    return bestInfo;
}   

void
RADR::AdrImplementation(uint8_t* newDataRate,
                        uint8_t* newTxPower,
                        Ptr<EndDeviceStatus> status)
{    
    // SF's Sensitivies
    std::vector<double> sensitivies = { -130.0, -132.5, -135.0, -137.5, -140.0, -142.5 };

    // Set ToAs
    if (m_toas.empty() && m_sToas != "")
    {
        std::vector<std::string> data = split(m_sToas, ',');
        for (auto d: data)
        {
            m_toas.push_back(std::stod(d));
        }
        data.clear();
    }
    else if (m_toas.empty())
    {
        // ToA for 51-Byte packet [Default Value]
        m_toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};
    }

    /*
     * Set SF and TP for ED:
     *  - Determine the maximum SNR received for the packet (max SNR), 
     *    and identify the GW that receives the packet with the max SNR (best GW)
     */
    EndDeviceStatus::ReceivedPacketList packetList = status->GetReceivedPacketList();
    // Get Last Packet Received
    auto it = packetList.rbegin();
    // Get max SNR and best GW
    EndDeviceStatus::PacketInfoPerGw bestInfo = GetMaxSnrAndBestGw(it->second.gwList);

    GatewayInfo gwInfo;
    auto itGw = m_gwInfos.find(bestInfo.gwAddress);
    if (itGw != m_gwInfos.end())
    {
        gwInfo = itGw->second;
    }    
    else
    {
        const Address& gwAddr = bestInfo.gwAddress;
        gwInfo = GatewayInfo(gwAddr);
        m_gwInfos.insert(std::pair<Address, GatewayInfo>(gwAddr, gwInfo));
    }
    
    // Determine max SNR and best GW
    double maxSnr = bestInfo.rxPower;
    uint8_t newSF = 12;
    for (size_t i = 0; i < sensitivies.size(); i++)
    {
        if (maxSnr > sensitivies[i])
        {
            newSF = (uint8_t) i + 7;
            break;
        }
    }

    // Get Last TP value
    uint8_t tp = status->GetMac()->GetTransmissionPower();

    // Update GatewayInfo
    // gwInfo.m_numEdsPerSf[newSF - 7]++;

    // Clear Data
    sensitivies.clear();

    *newDataRate = SfToDr(newSF);
    *newTxPower = tp;
}

NS_OBJECT_ENSURE_REGISTERED(MBADR);

TypeId
MBADR::GetTypeId()
{
  static TypeId tid =
        TypeId("ns3::MBADR")
            .SetGroupName("lorawan")
            .AddConstructor<MBADR>()
            .SetParent<AdrComponent>();
  
  return tid;
}

MBADR::MBADR()
{
}

MBADR::~MBADR()
{
}

/*
double CalcMedian(std::vector<double> v) 
{
    if (v.empty()) return NAN;
    
    std::sort(v.begin(), v.end());
    
    int n = v.size();
    if (n % 2 == 0)
    {
        return (v[n/2 - 1] + v[n/2]) / 2.0;
    }
    else
    {
        return v[n/2];
    }
}

double CalcFirstQuartile(std::vector<double> v)
{
    if (v.empty()) return NAN;

    std::sort(v.begin(), v.end());

    int n = v.size();
    int mid = n / 2;

    // Metade inferior (Tukey: exclui a mediana se n for ímpar)
    std::vector<double> lower(v.begin(), v.begin() + mid);

    int m = lower.size();
    if (m == 0) return NAN;

    if (m % 2 == 0)
    {
        return (lower[m/2 - 1] + lower[m/2]) / 2.0;
    }
    else
    {
        return lower[m/2];
    }
}

double CalcThirdQuartile(std::vector<double> v)
{
    if (v.empty()) return NAN;

    std::sort(v.begin(), v.end());

    int n = v.size();
    int mid = n / 2;

    // Metade superior (Tukey: exclui a mediana se n for ímpar)
    std::vector<double> upper;

    if (n % 2 == 0)
        upper = std::vector<double>(v.begin() + mid, v.end());
    else
        upper = std::vector<double>(v.begin() + mid + 1, v.end());

    int m = upper.size();
    if (m == 0) return NAN;

    if (m % 2 == 0)
    {
        return (upper[m/2 - 1] + upper[m/2]) / 2.0;
    }
    else
    {
        return upper[m/2];
    }
}

double CalcIQR(std::vector<double> v)
{
    if (v.empty()) return NAN;

    double Q1 = CalcFirstQuartile(v);
    double Q3 = CalcThirdQuartile(v);

    return Q3 - Q1;
}

std::vector<double> RemOutliers(std::vector<double> v)
{
    if (v.size() < 4) 
        return v; // Não faz sentido remover outliers com poucos dados

    double Q1 = CalcFirstQuartile(v);
    double Q3 = CalcThirdQuartile(v);
    double IQR = Q3 - Q1;
    

    double lowerBound = Q1 - 1.5 * IQR;
    double upperBound = Q3 + 1.5 * IQR;

    std::vector<double> filtered;

    for (double x : v)
    {
        if (x >= lowerBound && x <= upperBound)
        {
            filtered.push_back(x);
        }
    }

    return filtered;
}
*/

std::vector<double> 
MBADR::GetSnrList(EndDeviceStatus::ReceivedPacketList packetList, int historyRange)
{
    std::vector<double> snrList;
    // Take elements from the list starting at the end
    auto it = packetList.rbegin();

    for (int i = 0; i < historyRange; i++, it++)
    {
        //double snr = RxPowerToSNR(GetMaxTxFromGateways(it->second.gwList));
        double snr = RxPowerToSNR(GetReceivedPower(it->second.gwList));
        snrList.push_back(snr);
    }

    return snrList;
}

double
MBADR::CalcFirstQuartile(std::vector<double> v)
{
    if (v.empty()) return NAN;

    std::sort(v.begin(), v.end());

    int n = v.size();
    int mid = n / 2;

    // Metade inferior (Tukey: exclui a mediana se n for ímpar)
    std::vector<double> lower(v.begin(), v.begin() + mid);

    int m = lower.size();
    if (m == 0) return NAN;

    if (m % 2 == 0)
    {
        return (lower[m/2 - 1] + lower[m/2]) / 2.0;
    }
    else
    {
        return lower[m/2];
    }
}

double
MBADR::CalcThirdQuartile(std::vector<double> v)
{
    if (v.empty()) return NAN;

    std::sort(v.begin(), v.end());

    int n = v.size();
    int mid = n / 2;

    // Metade superior (Tukey: exclui a mediana se n for ímpar)
    std::vector<double> upper;

    if (n % 2 == 0)
    {
        upper = std::vector<double>(v.begin() + mid, v.end());
    }
    else
    {
        upper = std::vector<double>(v.begin() + mid + 1, v.end());
    }

    int m = upper.size();
    if (m == 0) return NAN;

    if (m % 2 == 0)
    {
        return (upper[m/2 - 1] + upper[m/2]) / 2.0;
    }
    else
    {
        return upper[m/2];
    }
}

double 
MBADR::CalcMedian(std::vector<double> v) 
{
    if (v.empty()) return NAN;
    
    std::sort(v.begin(), v.end());
    
    int n = v.size();
    if (n % 2 == 0)
    {
        return (v[n/2 - 1] + v[n/2]) / 2.0;
    }
    else
    {
        return v[n/2];
    }
}

std::vector<double> 
MBADR::RemOutliers(std::vector<double> v)
{
    if (v.size() < 4) 
    {
        return v; // Não faz sentido remover outliers com poucos dados
    }

    double Q1 = CalcFirstQuartile(v);
    double Q3 = CalcThirdQuartile(v);
    double IQR = Q3 - Q1;

    double lowerBound = Q1 - 1.5 * IQR;
    double upperBound = Q3 + 1.5 * IQR;

    std::vector<double> filtered;
    for (double x : v)
    {
        if (x >= lowerBound && x <= upperBound)
        {
            filtered.push_back(x);
        }
    }

    return filtered;
}

void
MBADR::AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status)
{    
    std::vector<double> snrList = GetSnrList(status->GetReceivedPacketList(), historyRange);
    std::vector<double> cleanedList = RemOutliers(snrList);

    /*if (cleanedList.empty()) 
    {
        snrList.clear();
        cleanedList.clear();

        *newDataRate = SfToDr(status->GetFirstReceiveWindowSpreadingFactor());
        *newTxPower = status->GetMac()->GetTransmissionPower();
        return;
    }*/

    double m_SNR = CalcMedian(cleanedList);

    NS_LOG_DEBUG("m_SNR = " << m_SNR);

    // Get the spreading factor used by the device
    uint8_t spreadingFactor = status->GetFirstReceiveWindowSpreadingFactor();

    NS_LOG_DEBUG("SF = " << (unsigned)spreadingFactor);

    // Get the device data rate and use it to get the SNR demodulation threshold
    double req_SNR = threshold[SfToDr(spreadingFactor)];

    NS_LOG_DEBUG("Required SNR = " << req_SNR);

    // Get the device transmission power (dBm)
    double transmissionPower = status->GetMac()->GetTransmissionPower();

    NS_LOG_DEBUG("Transmission Power = " << transmissionPower);

    // Compute the SNR margin taking into consideration the SNR of
    // previously received packets
    double margin_SNR = m_SNR - req_SNR - m_margin;

    NS_LOG_DEBUG("Margin = " << margin_SNR);

    // Number of steps to decrement the spreading factor (thereby increasing the data rate)
    // and the TP.
    int steps = std::floor(margin_SNR / 3);

    NS_LOG_DEBUG("steps = " << steps);

    // If the number of steps is positive (margin_SNR is positive, so its
    // decimal value is high) increment the data rate, if there are some
    // leftover steps after reaching the maximum possible data rate
    //(corresponding to the minimum spreading factor) decrement the transmission power as
    // well for the number of steps left.
    // If, on the other hand, the number of steps is negative (margin_SNR is
    // negative, so its decimal value is low) increase the transmission power
    //(note that the spreading factor is not incremented as this particular algorithm
    // expects the node itself to raise its spreading factor whenever necessary).
    while (steps > 0 && spreadingFactor > min_spreadingFactor)
    {
        spreadingFactor--;
        steps--;
        NS_LOG_DEBUG("Decreased SF by 1");
    }
    while (steps > 0 && transmissionPower > min_transmissionPower)
    {
        transmissionPower -= 2;
        steps--;
        NS_LOG_DEBUG("Decreased Ptx by 2");
    }
    while (steps < 0 && transmissionPower < max_transmissionPower)
    {
        transmissionPower += 2;
        steps++;
        NS_LOG_DEBUG("Increased Ptx by 2");
    }

    snrList.clear();
    cleanedList.clear();

    *newDataRate = SfToDr(spreadingFactor);
    *newTxPower = transmissionPower;
}

// KADR
NS_OBJECT_ENSURE_REGISTERED(KADR);

TypeId
KADR::GetTypeId()
{
  static TypeId tid =
        TypeId("ns3::KADR")
            .SetGroupName("lorawan")
            .AddConstructor<KADR>()
            .SetParent<AdrComponent>()
            .AddAttribute("Alpha",
                          "Interval of message transmission in seconds",
                          DoubleValue(5),
                          MakeDoubleAccessor(&KADR::m_alpha),
                          MakeDoubleChecker<double>(1, 10.0));
  
  return tid;
}

KADR::KADR()
{
}

KADR::~KADR()
{
}

std::vector<double> 
KADR::GetSnrList(EndDeviceStatus::ReceivedPacketList packetList, int historyRange)
{
    std::vector<double> snrList;
    // Take elements from the list starting at the end
    auto it = packetList.rbegin();

    for (int i = 0; i < historyRange; i++, it++)
    {
        double snr = RxPowerToSNR(GetMaxTxFromGateways(it->second.gwList));
        snrList.push_back(snr);
    }

    return snrList;
}

double 
KADR::GaussianVariogram(int h, double alpha) 
{
    if (h == 0)
    {
        return 0.0;
    }

    return 1.0 - std::exp(-(h * h) / (alpha * alpha));
}

double 
KADR::PerformKrigingInterpolation(const std::vector<double>& snrList) 
{
    int n = snrList.size();
    std::vector<std::vector<double>> matrixK(n + 1, std::vector<double>(n + 1, 0.0));
    std::vector<double> vectorM(n + 1, 1.0);
    vectorM[n] = 1.0; // Lagrange multiplier part

    // Fill variogram matrix K
    for (int i = 0; i < n; ++i) 
    {
        matrixK[i][n] = 1.0;
        matrixK[n][i] = 1.0;
        for (int j = 0; j < n; ++j) 
        {
            matrixK[i][j] = GaussianVariogram(std::abs(i - j), m_alpha);
        }
    }

    // Simplified: assume equal weights for demonstration (no real linear solver here)
    std::vector<double> weights(n, 1.0 / n);

    // Weighted average interpolation (Kriging estimate)
    double snrKriging = 0.0;
    for (int i = 0; i < n; ++i) 
    {
        snrKriging += weights[i] * snrList[i];
    }

    vectorM.clear();
    vectorM.shrink_to_fit();

    for (auto it: matrixK)
    {
        it.clear();
        it.shrink_to_fit();
    }

    matrixK.clear();
    matrixK.shrink_to_fit();

    weights.clear();
    weights.shrink_to_fit();

    return snrKriging;
}

void
KADR::AdrImplementation(uint8_t* newDataRate,
                        uint8_t* newTxPower,
                        Ptr<EndDeviceStatus> status)
{    
    std::vector<double> snrList = GetSnrList(status->GetReceivedPacketList(), historyRange);
    double m_SNR = PerformKrigingInterpolation(snrList);

    NS_LOG_DEBUG("m_SNR = " << m_SNR);

    // Get the spreading factor used by the device
    uint8_t spreadingFactor = status->GetFirstReceiveWindowSpreadingFactor();

    NS_LOG_DEBUG("SF = " << (unsigned)spreadingFactor);

    // Get the device data rate and use it to get the SNR demodulation threshold
    double req_SNR = threshold[SfToDr(spreadingFactor)];

    NS_LOG_DEBUG("Required SNR = " << req_SNR);

    // Get the device transmission power (dBm)
    double transmissionPower = status->GetMac()->GetTransmissionPower();

    NS_LOG_DEBUG("Transmission Power = " << transmissionPower);

    // Compute the SNR margin taking into consideration the SNR of
    // previously received packets
    double margin_SNR = m_SNR - req_SNR - m_margin;

    NS_LOG_DEBUG("Margin = " << margin_SNR);

    // Number of steps to decrement the spreading factor (thereby increasing the data rate)
    // and the TP.
    int steps = std::floor(margin_SNR / 3);

    NS_LOG_DEBUG("steps = " << steps);

    // If the number of steps is positive (margin_SNR is positive, so its
    // decimal value is high) increment the data rate, if there are some
    // leftover steps after reaching the maximum possible data rate
    //(corresponding to the minimum spreading factor) decrement the transmission power as
    // well for the number of steps left.
    // If, on the other hand, the number of steps is negative (margin_SNR is
    // negative, so its decimal value is low) increase the transmission power
    //(note that the spreading factor is not incremented as this particular algorithm
    // expects the node itself to raise its spreading factor whenever necessary).
    while (steps > 0 && spreadingFactor > min_spreadingFactor)
    {
        spreadingFactor--;
        steps--;
        NS_LOG_DEBUG("Decreased SF by 1");
    }
    while (steps > 0 && transmissionPower > min_transmissionPower)
    {
        transmissionPower -= 2;
        steps--;
        NS_LOG_DEBUG("Decreased Ptx by 2");
    }
    while (steps < 0 && transmissionPower < max_transmissionPower)
    {
        transmissionPower += 2;
        steps++;
        NS_LOG_DEBUG("Increased Ptx by 2");
    }

    snrList.clear();

    *newDataRate = SfToDr(spreadingFactor);
    *newTxPower = transmissionPower;
}

// SSFIR
NS_OBJECT_ENSURE_REGISTERED(SSFIR);

TypeId
SSFIR::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SSFIR")
            .SetGroupName("lorawan")
            .AddConstructor<SSFIR>()
            .SetParent<AdrComponent>();

    return tid;
}

SSFIR::SSFIR()
{
}

SSFIR::~SSFIR()
{
}

void
SSFIR::AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status)
{
    // Compute the average SNR
    double m_SNR = GetAverageSNR(status->GetReceivedPacketList(), historyRange);

    NS_LOG_DEBUG("m_SNR = " << m_SNR);

    // Get the spreading factor used by the device
    uint8_t spreadingFactor = status->GetFirstReceiveWindowSpreadingFactor();

    NS_LOG_DEBUG("SF = " << (unsigned)spreadingFactor);

    // Get the device data rate and use it to get the SNR demodulation threshold
    double req_SNR = threshold[SfToDr(spreadingFactor)];

    NS_LOG_DEBUG("Required SNR = " << req_SNR);

    // Get the device transmission power (dBm)
    double transmissionPower = status->GetMac()->GetTransmissionPower();

    NS_LOG_DEBUG("Transmission Power = " << transmissionPower);

    // Compute the SNR margin taking into consideration the SNR of
    // previously received packets
    double margin_SNR = m_SNR - req_SNR - m_margin;

    NS_LOG_DEBUG("Margin = " << margin_SNR);

    // Number of steps to decrement the spreading factor (thereby increasing the data rate)
    // and the TP.
    int steps = std::floor(margin_SNR / 3);

    NS_LOG_DEBUG("steps = " << steps);

    // If the number of steps is positive (margin_SNR is positive, so its
    // decimal value is high) increment the data rate, if there are some
    // leftover steps after reaching the maximum possible data rate
    //(corresponding to the minimum spreading factor) decrement the transmission power as
    // well for the number of steps left.
    // If, on the other hand, the number of steps is negative (margin_SNR is
    // negative, so its decimal value is low) increase the transmission power
    //(note that the spreading factor is not incremented as this particular algorithm
    // expects the node itself to raise its spreading factor whenever necessary).
    while (steps > 0 && spreadingFactor > min_spreadingFactor)
    {
        spreadingFactor--;
        steps--;
        NS_LOG_DEBUG("Decreased SF by 1");
    }
    while (steps > 0 && transmissionPower > min_transmissionPower)
    {
        transmissionPower -= 2;
        steps--;
        NS_LOG_DEBUG("Decreased Ptx by 2");
    }
    /*while (steps < 0 && transmissionPower < max_transmissionPower)
    {
        transmissionPower += 2;
        steps++;
        NS_LOG_DEBUG("Increased Ptx by 2");
    }*/

    uint8_t newSF = 11; // newSF = maxSF - 1 => maxSF = 12
    while (m_SNR > threshold[SfToDr(newSF)] && newSF >= min_spreadingFactor)
    {
        newSF--;
    }    
    newSF++;

    /*std::cout << "SF = " << unsigned(newSF) << ", TP = " << unsigned(transmissionPower) << " dBm"
              << ", m_SNR = " << m_SNR << "dB, HistoryRange = " << historyRange 
              << ", Threshold = " << threshold[SfToDr(newSF)] << " dBm" << std::endl;*/

    *newDataRate = SfToDr(newSF);
    *newTxPower = transmissionPower;
}

// SSFIR2
NS_OBJECT_ENSURE_REGISTERED(SSFIR2);

TypeId
SSFIR2::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SSFIR2")
            .SetGroupName("lorawan")
            .AddConstructor<SSFIR2>()
            .SetParent<AdrComponent>()
            .AddAttribute("RhoSF",
                          "Probability for Change SF",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&SSFIR2::m_rhoSF),
                          MakeDoubleChecker<double>(0.01, 0.95));

    return tid;
}

SSFIR2::SSFIR2()
{
}

SSFIR2::~SSFIR2()
{
}

void
SSFIR2::AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status)
{
    // Compute the average SNR
    double m_SNR = GetAverageSNR(status->GetReceivedPacketList(), historyRange);

    NS_LOG_DEBUG("m_SNR = " << m_SNR);

    // Get the spreading factor used by the device
    uint8_t spreadingFactor = status->GetFirstReceiveWindowSpreadingFactor();

    NS_LOG_DEBUG("SF = " << (unsigned)spreadingFactor);

    // Get the device data rate and use it to get the SNR demodulation threshold
    double req_SNR = threshold[SfToDr(spreadingFactor)];

    NS_LOG_DEBUG("Required SNR = " << req_SNR);

    // Get the device transmission power (dBm)
    double transmissionPower = status->GetMac()->GetTransmissionPower();

    NS_LOG_DEBUG("Transmission Power = " << transmissionPower);

    // Compute the SNR margin taking into consideration the SNR of
    // previously received packets
    double margin_SNR = m_SNR - req_SNR - m_margin;

    NS_LOG_DEBUG("Margin = " << margin_SNR);

    // Number of steps to decrement the spreading factor (thereby increasing the data rate)
    // and the TP.
    int steps = std::floor(margin_SNR / 3);

    NS_LOG_DEBUG("steps = " << steps);

    // If the number of steps is positive (margin_SNR is positive, so its
    // decimal value is high) increment the data rate, if there are some
    // leftover steps after reaching the maximum possible data rate
    //(corresponding to the minimum spreading factor) decrement the transmission power as
    // well for the number of steps left.
    // If, on the other hand, the number of steps is negative (margin_SNR is
    // negative, so its decimal value is low) increase the transmission power
    //(note that the spreading factor is not incremented as this particular algorithm
    // expects the node itself to raise its spreading factor whenever necessary).
    while (steps > 0 && spreadingFactor > min_spreadingFactor)
    {
        spreadingFactor--;
        steps--;
        NS_LOG_DEBUG("Decreased SF by 1");
    }
    while (steps > 0 && transmissionPower > min_transmissionPower)
    {
        transmissionPower -= 2;
        steps--;
        NS_LOG_DEBUG("Decreased Ptx by 2");
    }
    /*while (steps < 0 && transmissionPower < max_transmissionPower)
    {
        transmissionPower += 2;
        steps++;
        NS_LOG_DEBUG("Increased Ptx by 2");
    }*/

    Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable>(
                                        "Min",
                                        DoubleValue(0.0),
                                        "Max",
                                        DoubleValue(1.0)
    );

    double rho;
    uint8_t newSF = 12; // newSF = maxSF => maxSF = 12
    uint8_t sf = newSF - 1; // sf = newSF - 1 => 11
    while (m_SNR > threshold[SfToDr(sf)] && sf >= min_spreadingFactor)
    {
        rho = rv->GetValue();
        if (rho > m_rhoSF)
        {
            newSF = sf;
        }
        sf--;
    }

    /*std::cout << "SF = " << unsigned(newSF) << ", TP = " << unsigned(transmissionPower) << " dBm"
              << ", m_SNR = " << m_SNR << "dB, HistoryRange = " << historyRange << ", rhoSF = " << m_rhoSF
              << ", Threshold = " << threshold[SfToDr(newSF)] << " dBm" << std::endl;*/

    *newDataRate = SfToDr(newSF);
    *newTxPower = transmissionPower;
}

} // namespace lorawan
} // namespace ns3

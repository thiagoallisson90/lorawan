/*
 * Copyright (c) 2017 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#include "lorawan-mac-helper.h"

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/log.h"
#include "ns3/lora-net-device.h"
#include "ns3/random-variable-stream.h"
#include "ns3/poisson-sender.h"

#include <cmath>

namespace ns3
{
namespace lorawan
{

// Thiago Allisson: Add função ComparePerPr
bool comparePerPr(const EdAndPr& a, const EdAndPr& b) 
{
  return a.m_pr > b.m_pr;
}

// Thiago Allisson
double RxPowerToSNR(double transmissionPower, double bandwidth = 125e3, double NF = 6)
{
  return transmissionPower + 174 - 10 * log10(bandwidth) - NF;
}

// Thiago Allisson
double Log(double base, double value)
{
    return std::log(value) / std::log(base);
}

// Thiago Allisson
int NumMaxOfNodesPerSF(double toa, double t, double succProb, int nFreq = 1)
{
    double base = (1 - toa / t);
    double numMax = 0.5 * Log(base, succProb);
    return std::floor(numMax + 1) * nFreq;
}

// Thiago Allisson
double SuccProb(double toa, double t, double numOfNodes)
{
    return std::pow(1 - toa / t, 2 * (numOfNodes - 1));
}

// Thiago Allisson
struct GwInfoT
{
    std::vector<int> m_sf7;
    std::vector<int> m_sf8;
    std::vector<int> m_sf9;
    std::vector<int> m_sf10;
    std::vector<int> m_sf11;
    std::vector<int> m_sf12;

    int m_n;

    GwInfoT()
    {
        m_n = 0;
    }

    void Clear()
    {
        m_sf7.clear();
        m_sf8.clear();
        m_sf9.clear();
        m_sf10.clear();
        m_sf11.clear();
        m_sf12.clear();
    }

    void AddNode(int node, int sf)
    {
        switch (sf)
        {
        case 7:
            m_sf7.push_back(node);
            break;
        case 8:
            m_sf8.push_back(node);
            break;
        case 9:
            m_sf9.push_back(node);
            break;
        case 10:
            m_sf10.push_back(node);
            break;
        case 11:
            m_sf11.push_back(node);
            break;
        case 12:
            m_sf12.push_back(node);
            break;
        default:
            break;
        }

        m_n++;
    }
};

NS_LOG_COMPONENT_DEFINE("LorawanMacHelper");

LorawanMacHelper::LorawanMacHelper()
    : m_region(LorawanMacHelper::EU)
{
}

void
LorawanMacHelper::Set(std::string name, const AttributeValue& v)
{
    m_mac.Set(name, v);
}

void
LorawanMacHelper::SetDeviceType(enum DeviceType dt)
{
    NS_LOG_FUNCTION(this << dt);
    switch (dt)
    {
    case GW:
        m_mac.SetTypeId("ns3::GatewayLorawanMac");
        break;
    case ED_A:
        m_mac.SetTypeId("ns3::ClassAEndDeviceLorawanMac");
        break;
    }
    m_deviceType = dt;
}

void
LorawanMacHelper::SetAddressGenerator(Ptr<LoraDeviceAddressGenerator> addrGen)
{
    NS_LOG_FUNCTION(this);

    m_addrGen = addrGen;
}

void
LorawanMacHelper::SetRegion(enum LorawanMacHelper::Regions region)
{
    m_region = region;
}

Ptr<LorawanMac>
LorawanMacHelper::Create(Ptr<Node> node, Ptr<NetDevice> device) const
{
    Ptr<LorawanMac> mac = m_mac.Create<LorawanMac>();
    mac->SetDevice(device);

    // If we are operating on an end device, add an address to it
    if (m_deviceType == ED_A && m_addrGen)
    {
        DynamicCast<ClassAEndDeviceLorawanMac>(mac)->SetDeviceAddress(m_addrGen->NextAddress());
    }

    // Add a basic list of channels based on the region where the device is
    // operating
    if (m_deviceType == ED_A)
    {
        Ptr<ClassAEndDeviceLorawanMac> edMac = DynamicCast<ClassAEndDeviceLorawanMac>(mac);
        switch (m_region)
        {
        case LorawanMacHelper::EU: {
            ConfigureForEuRegion(edMac);
            break;
        }
        case LorawanMacHelper::SingleChannel: {
            ConfigureForSingleChannelRegion(edMac);
            break;
        }
        case LorawanMacHelper::ALOHA: {
            ConfigureForAlohaRegion(edMac);
            break;
        }
        default: {
            NS_LOG_ERROR("This region isn't supported yet!");
            break;
        }
        }
    }
    else
    {
        Ptr<GatewayLorawanMac> gwMac = DynamicCast<GatewayLorawanMac>(mac);
        switch (m_region)
        {
        case LorawanMacHelper::EU: {
            ConfigureForEuRegion(gwMac);
            break;
        }
        case LorawanMacHelper::SingleChannel: {
            ConfigureForSingleChannelRegion(gwMac);
            break;
        }
        case LorawanMacHelper::ALOHA: {
            ConfigureForAlohaRegion(gwMac);
            break;
        }
        default: {
            NS_LOG_ERROR("This region isn't supported yet!");
            break;
        }
        }
    }
    return mac;
}

void
LorawanMacHelper::ConfigureForAlohaRegion(Ptr<ClassAEndDeviceLorawanMac> edMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ApplyCommonAlohaConfigurations(edMac);

    /////////////////////////////////////////////////////
    // TxPower -> Transmission power in dBm conversion //
    /////////////////////////////////////////////////////
    edMac->SetTxDbmForTxPower(std::vector<double>{16, 14, 12, 10, 8, 6, 4, 2});

    ////////////////////////////////////////////////////////////
    // Matrix to know which data rate the gateway will respond with //
    ////////////////////////////////////////////////////////////
    LorawanMac::ReplyDataRateMatrix matrix = {{{{0, 0, 0, 0, 0, 0}},
                                               {{1, 0, 0, 0, 0, 0}},
                                               {{2, 1, 0, 0, 0, 0}},
                                               {{3, 2, 1, 0, 0, 0}},
                                               {{4, 3, 2, 1, 0, 0}},
                                               {{5, 4, 3, 2, 1, 0}},
                                               {{6, 5, 4, 3, 2, 1}},
                                               {{7, 6, 5, 4, 3, 2}}}};
    edMac->SetReplyDataRateMatrix(matrix);

    /////////////////////
    // Preamble length //
    /////////////////////
    edMac->SetNPreambleSymbols(8);

    //////////////////////////////////////
    // Second receive window parameters //
    //////////////////////////////////////
    edMac->SetSecondReceiveWindowDataRate(0);
    edMac->SetSecondReceiveWindowFrequency(869.525);
}

void
LorawanMacHelper::ConfigureForAlohaRegion(Ptr<GatewayLorawanMac> gwMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ///////////////////////////////
    // ReceivePath configuration //
    ///////////////////////////////
    Ptr<GatewayLoraPhy> gwPhy =
        DynamicCast<GatewayLoraPhy>(DynamicCast<LoraNetDevice>(gwMac->GetDevice())->GetPhy());

    ApplyCommonAlohaConfigurations(gwMac);

    if (gwPhy) // If cast is successful, there's a GatewayLoraPhy
    {
        NS_LOG_DEBUG("Resetting reception paths");
        gwPhy->ResetReceptionPaths();

        int receptionPaths = 0;
        int maxReceptionPaths = 1;
        while (receptionPaths < maxReceptionPaths)
        {
            DynamicCast<GatewayLoraPhy>(gwPhy)->AddReceptionPath();
            receptionPaths++;
        }
        gwPhy->AddFrequency(868.1);
    }
}

void
LorawanMacHelper::ApplyCommonAlohaConfigurations(Ptr<LorawanMac> lorawanMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    //////////////
    // SubBands //
    //////////////

    Ptr<LogicalLoraChannelHelper> channelHelper = CreateObject<LogicalLoraChannelHelper>();
    channelHelper->AddSubBand(868, 868.6, 1, 14);

    //////////////////////
    // Default channels //
    //////////////////////
    Ptr<LogicalLoraChannel> lc1 = CreateObject<LogicalLoraChannel>(868.1, 0, 5);
    channelHelper->AddChannel(lc1);

    lorawanMac->SetLogicalLoraChannelHelper(channelHelper);

    ///////////////////////////////////////////////////////////
    // Data rate -> Spreading factor, Data rate -> Bandwidth //
    // and Data rate -> MaxAppPayload conversions            //
    ///////////////////////////////////////////////////////////
    lorawanMac->SetSfForDataRate(std::vector<uint8_t>{12, 11, 10, 9, 8, 7, 7});
    lorawanMac->SetBandwidthForDataRate(
        std::vector<double>{125000, 125000, 125000, 125000, 125000, 125000, 250000});
    lorawanMac->SetMaxAppPayloadForDataRate(
        std::vector<uint32_t>{59, 59, 59, 123, 230, 230, 230, 230});
}

void
LorawanMacHelper::ConfigureForEuRegion(Ptr<ClassAEndDeviceLorawanMac> edMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ApplyCommonEuConfigurations(edMac);

    /////////////////////////////////////////////////////
    // TxPower -> Transmission power in dBm conversion //
    /////////////////////////////////////////////////////
    edMac->SetTxDbmForTxPower(std::vector<double>{16, 14, 12, 10, 8, 6, 4, 2});

    ////////////////////////////////////////////////////////////
    // Matrix to know which data rate the gateway will respond with //
    ////////////////////////////////////////////////////////////
    LorawanMac::ReplyDataRateMatrix matrix = {{{{0, 0, 0, 0, 0, 0}},
                                               {{1, 0, 0, 0, 0, 0}},
                                               {{2, 1, 0, 0, 0, 0}},
                                               {{3, 2, 1, 0, 0, 0}},
                                               {{4, 3, 2, 1, 0, 0}},
                                               {{5, 4, 3, 2, 1, 0}},
                                               {{6, 5, 4, 3, 2, 1}},
                                               {{7, 6, 5, 4, 3, 2}}}};
    edMac->SetReplyDataRateMatrix(matrix);

    /////////////////////
    // Preamble length //
    /////////////////////
    edMac->SetNPreambleSymbols(8);

    //////////////////////////////////////
    // Second receive window parameters //
    //////////////////////////////////////
    edMac->SetSecondReceiveWindowDataRate(0);
    edMac->SetSecondReceiveWindowFrequency(869.525);
}

void
LorawanMacHelper::ConfigureForEuRegion(Ptr<GatewayLorawanMac> gwMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ///////////////////////////////
    // ReceivePath configuration //
    ///////////////////////////////
    Ptr<GatewayLoraPhy> gwPhy =
        DynamicCast<GatewayLoraPhy>(DynamicCast<LoraNetDevice>(gwMac->GetDevice())->GetPhy());

    ApplyCommonEuConfigurations(gwMac);

    if (gwPhy) // If cast is successful, there's a GatewayLoraPhy
    {
        NS_LOG_DEBUG("Resetting reception paths");
        gwPhy->ResetReceptionPaths();

        std::vector<double> frequencies;
        frequencies.push_back(868.1);
        frequencies.push_back(868.3);
        frequencies.push_back(868.5);

        for (auto& f : frequencies)
        {
            gwPhy->AddFrequency(f);
        }

        int receptionPaths = 0;
        int maxReceptionPaths = 8;
        while (receptionPaths < maxReceptionPaths)
        {
            DynamicCast<GatewayLoraPhy>(gwPhy)->AddReceptionPath();
            receptionPaths++;
        }
    }
}

void
LorawanMacHelper::ApplyCommonEuConfigurations(Ptr<LorawanMac> lorawanMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    //////////////
    // SubBands //
    //////////////

    Ptr<LogicalLoraChannelHelper> channelHelper = CreateObject<LogicalLoraChannelHelper>();
    channelHelper->AddSubBand(868, 868.6, 0.01, 14);
    channelHelper->AddSubBand(868.7, 869.2, 0.001, 14);
    channelHelper->AddSubBand(869.4, 869.65, 0.1, 27);

    //////////////////////
    // Default channels //
    //////////////////////
    Ptr<LogicalLoraChannel> lc1 = CreateObject<LogicalLoraChannel>(868.1, 0, 5);
    Ptr<LogicalLoraChannel> lc2 = CreateObject<LogicalLoraChannel>(868.3, 0, 5);
    Ptr<LogicalLoraChannel> lc3 = CreateObject<LogicalLoraChannel>(868.5, 0, 5);
    channelHelper->AddChannel(lc1);
    channelHelper->AddChannel(lc2);
    channelHelper->AddChannel(lc3);

    lorawanMac->SetLogicalLoraChannelHelper(channelHelper);

    ///////////////////////////////////////////////////////////
    // Data rate -> Spreading factor, Data rate -> Bandwidth //
    // and Data rate -> MaxAppPayload conversions            //
    ///////////////////////////////////////////////////////////
    lorawanMac->SetSfForDataRate(std::vector<uint8_t>{12, 11, 10, 9, 8, 7, 7});
    lorawanMac->SetBandwidthForDataRate(
        std::vector<double>{125000, 125000, 125000, 125000, 125000, 125000, 250000});
    lorawanMac->SetMaxAppPayloadForDataRate(
        std::vector<uint32_t>{59, 59, 59, 123, 230, 230, 230, 230});
}

///////////////////////////////

void
LorawanMacHelper::ConfigureForSingleChannelRegion(Ptr<ClassAEndDeviceLorawanMac> edMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ApplyCommonSingleChannelConfigurations(edMac);

    /////////////////////////////////////////////////////
    // TxPower -> Transmission power in dBm conversion //
    /////////////////////////////////////////////////////
    edMac->SetTxDbmForTxPower(std::vector<double>{16, 14, 12, 10, 8, 6, 4, 2});

    ////////////////////////////////////////////////////////////
    // Matrix to know which DataRate the gateway will respond with //
    ////////////////////////////////////////////////////////////
    LorawanMac::ReplyDataRateMatrix matrix = {{{{0, 0, 0, 0, 0, 0}},
                                               {{1, 0, 0, 0, 0, 0}},
                                               {{2, 1, 0, 0, 0, 0}},
                                               {{3, 2, 1, 0, 0, 0}},
                                               {{4, 3, 2, 1, 0, 0}},
                                               {{5, 4, 3, 2, 1, 0}},
                                               {{6, 5, 4, 3, 2, 1}},
                                               {{7, 6, 5, 4, 3, 2}}}};
    edMac->SetReplyDataRateMatrix(matrix);

    /////////////////////
    // Preamble length //
    /////////////////////
    edMac->SetNPreambleSymbols(8);

    //////////////////////////////////////
    // Second receive window parameters //
    //////////////////////////////////////
    edMac->SetSecondReceiveWindowDataRate(0);
    edMac->SetSecondReceiveWindowFrequency(869.525);
}

void
LorawanMacHelper::ConfigureForSingleChannelRegion(Ptr<GatewayLorawanMac> gwMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ///////////////////////////////
    // ReceivePath configuration //
    ///////////////////////////////
    Ptr<GatewayLoraPhy> gwPhy =
        DynamicCast<GatewayLoraPhy>(DynamicCast<LoraNetDevice>(gwMac->GetDevice())->GetPhy());

    ApplyCommonEuConfigurations(gwMac);

    if (gwPhy) // If cast is successful, there's a GatewayLoraPhy
    {
        NS_LOG_DEBUG("Resetting reception paths");
        gwPhy->ResetReceptionPaths();

        std::vector<double> frequencies;
        frequencies.push_back(868.1);

        for (auto& f : frequencies)
        {
            gwPhy->AddFrequency(f);
        }

        int receptionPaths = 0;
        int maxReceptionPaths = 8;
        while (receptionPaths < maxReceptionPaths)
        {
            gwPhy->AddReceptionPath();
            receptionPaths++;
        }
    }
}

void
LorawanMacHelper::ApplyCommonSingleChannelConfigurations(Ptr<LorawanMac> lorawanMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    //////////////
    // SubBands //
    //////////////

    Ptr<LogicalLoraChannelHelper> channelHelper = CreateObject<LogicalLoraChannelHelper>();
    channelHelper->AddSubBand(868, 868.6, 0.01, 14);
    channelHelper->AddSubBand(868.7, 869.2, 0.001, 14);
    channelHelper->AddSubBand(869.4, 869.65, 0.1, 27);

    //////////////////////
    // Default channels //
    //////////////////////
    Ptr<LogicalLoraChannel> lc1 = CreateObject<LogicalLoraChannel>(868.1, 0, 5);
    channelHelper->AddChannel(lc1);

    lorawanMac->SetLogicalLoraChannelHelper(channelHelper);

    ///////////////////////////////////////////////////////////
    // Data rate -> Spreading factor, Data rate -> Bandwidth //
    // and Data rate -> MaxAppPayload conversions            //
    ///////////////////////////////////////////////////////////
    lorawanMac->SetSfForDataRate(std::vector<uint8_t>{12, 11, 10, 9, 8, 7, 7});
    lorawanMac->SetBandwidthForDataRate(
        std::vector<double>{125000, 125000, 125000, 125000, 125000, 125000, 250000});
    lorawanMac->SetMaxAppPayloadForDataRate(
        std::vector<uint32_t>{59, 59, 59, 123, 230, 230, 230, 230});
}

std::vector<int>
LorawanMacHelper::SetSpreadingFactorsUp(NodeContainer endDevices,
                                        NodeContainer gateways,
                                        Ptr<LoraChannel> channel)
{
    NS_LOG_FUNCTION_NOARGS();

    std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the end device sensitivity
        Ptr<EndDeviceLoraPhy> edPhy = DynamicCast<EndDeviceLoraPhy>(loraNetDevice->GetPhy());
        const double* edSensitivity = EndDeviceLoraPhy::sensitivity; // end-device-lora-phy.cc => loc 77

        if (rxPower > *edSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;
        }
        else if (rxPower > *(edSensitivity + 1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;
        }
        else if (rxPower > *(edSensitivity + 2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;
        }
        else if (rxPower > *(edSensitivity + 3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;
        }
        else if (rxPower > *(edSensitivity + 4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;
        }
        else if (rxPower > *(edSensitivity + 5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;
        }
        else // Device is out of range. Assign SF12.
        {
            // NS_LOG_DEBUG ("Device out of range");
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;
            // NS_LOG_DEBUG ("sfQuantity[6] = " << sfQuantity[6]);
        }

        /*

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy>
        (gatewayLoraNetDevice->GetPhy ()); const double *gwSensitivity = gatewayPhy->sensitivity;

        if(rxPower > *gwSensitivity)
          {
            mac->SetDataRate (5);
            sfQuantity[0] = sfQuantity[0] + 1;

          }
        else if (rxPower > *(gwSensitivity+1))
          {
            mac->SetDataRate (4);
            sfQuantity[1] = sfQuantity[1] + 1;

          }
        else if (rxPower > *(gwSensitivity+2))
          {
            mac->SetDataRate (3);
            sfQuantity[2] = sfQuantity[2] + 1;

          }
        else if (rxPower > *(gwSensitivity+3))
          {
            mac->SetDataRate (2);
            sfQuantity[3] = sfQuantity[3] + 1;
          }
        else if (rxPower > *(gwSensitivity+4))
          {
            mac->SetDataRate (1);
            sfQuantity[4] = sfQuantity[4] + 1;
          }
        else if (rxPower > *(gwSensitivity+5))
          {
            mac->SetDataRate (0);
            sfQuantity[5] = sfQuantity[5] + 1;

          }
        else // Device is out of range. Assign SF12.
          {
            mac->SetDataRate (0);
            sfQuantity[6] = sfQuantity[6] + 1;

          }
          */

    } // end loop on nodes

    return sfQuantity;

} //  end function

std::vector<int>
LorawanMacHelper::SetSpreadingFactorsUpBasedOnGWSens(NodeContainer endDevices,
                                                     NodeContainer gateways,
                                                     Ptr<LoraChannel> channel)
{
    NS_LOG_FUNCTION_NOARGS();

    std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy> (gatewayLoraNetDevice->GetPhy ()); 
        const double *gwSensitivity = gatewayPhy->sensitivity; // gateway-lora-phy.cc => loc 131

        if(rxPower > *gwSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;
        }
        else if (rxPower > *(gwSensitivity+1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;
        }
        else if (rxPower > *(gwSensitivity+2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;

        }
        else if (rxPower > *(gwSensitivity+3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;
        }
        else if (rxPower > *(gwSensitivity+4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;
        }
        else if (rxPower > *(gwSensitivity+5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;
        }
        else // Device is out of range. Assign SF12.
        {
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;
        }
    } // end loop on nodes

    return sfQuantity;

} //  end function


std::vector<int>
LorawanMacHelper::CSFA(NodeContainer endDevices,
                             NodeContainer gateways,
                             Ptr<LoraChannel> channel,
                             bool useGwSens)
{
    NS_LOG_FUNCTION_NOARGS();

    std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        // double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);
        double highestRxPower = position->GetDistanceFrom(bestGatewayPosition); // m

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            // double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm
            double currentRxPower = position->GetDistanceFrom(currPosition); // m

            if (currentRxPower < highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy> (gatewayLoraNetDevice->GetPhy ()); 
        const double *gwSensitivity = useGwSens ? gatewayPhy->sensitivity : EndDeviceLoraPhy::sensitivity;

        if(rxPower > *gwSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;
        }
        else if (rxPower > *(gwSensitivity+1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;
        }
        else if (rxPower > *(gwSensitivity+2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;

        }
        else if (rxPower > *(gwSensitivity+3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;
        }
        else if (rxPower > *(gwSensitivity+4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;
        }
        else if (rxPower > *(gwSensitivity+5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;
        }
        else // Device is out of range. Assign SF12.
        {
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;
        }
    } // end loop on nodes

    return sfQuantity;
}  

// ToDo
std::vector<int>
LorawanMacHelper::CeSFA(NodeContainer endDevices,
                              NodeContainer gateways,
                              Ptr<LoraChannel> channel,
                              bool useGwSens)
{
    NS_LOG_FUNCTION_NOARGS();

    std::vector<EdAndPr> vecSf8;
    std::vector<EdAndPr> vecSf9;
    std::vector<EdAndPr> vecSf10;
    std::vector<EdAndPr> vecSf11;
    std::vector<EdAndPr> vecSf12;

    std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy> (gatewayLoraNetDevice->GetPhy ()); 
        const double *gwSensitivity = useGwSens ? gatewayPhy->sensitivity : EndDeviceLoraPhy::sensitivity;

        if(rxPower > *gwSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;
        }
        else if (rxPower > *(gwSensitivity+1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;

            vecSf8.push_back(EdAndPr(object->GetId(), rxPower, (int) bestGateway->GetId()));
        }
        else if (rxPower > *(gwSensitivity+2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;

            vecSf9.push_back(EdAndPr(object->GetId(), rxPower, (int) bestGateway->GetId()));
        }
        else if (rxPower > *(gwSensitivity+3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;

            vecSf10.push_back(EdAndPr(object->GetId(), rxPower, (int) bestGateway->GetId()));
        }
        else if (rxPower > *(gwSensitivity+4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;

            vecSf11.push_back(EdAndPr(object->GetId(), rxPower, (int) bestGateway->GetId()));
        }
        else if (rxPower > *(gwSensitivity+5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;

            vecSf12.push_back(EdAndPr(object->GetId(), rxPower, (int) bestGateway->GetId()));
        }
        else // Device is out of range. Assign SF12.
        {
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;

            vecSf12.push_back(EdAndPr(object->GetId(), rxPower, (int) bestGateway->GetId()));
        }
    } // end loop on nodes

    // Ordenar de Forma Crescente SFs por Potência Recebida pelo GWs
    std::sort(vecSf8.begin(), vecSf8.end(), comparePerPr);
    std::sort(vecSf9.begin(), vecSf9.end(), comparePerPr);
    std::sort(vecSf10.begin(), vecSf10.end(), comparePerPr);
    std::sort(vecSf11.begin(), vecSf11.end(), comparePerPr);
    std::sort(vecSf12.begin(), vecSf12.end(), comparePerPr);

    for (int i = 0; i < vecSf8.size() * 0.03; i++)
    {
        EdAndPr data = vecSf8[i];

        Ptr<Node> node = endDevices.Get(data.m_ed);
        Ptr<LoraNetDevice> dev = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = dev->GetMac()->GetObject<EndDeviceLorawanMac>();
        mac->SetDataRate(5);

        sfQuantity[0]++;
        sfQuantity[1]--;
    }

    for (int i = 0; i < vecSf9.size() * 0.03; i++)
    {
        EdAndPr data = vecSf9[i];

        Ptr<Node> node = endDevices.Get(data.m_ed);
        Ptr<LoraNetDevice> dev = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = dev->GetMac()->GetObject<EndDeviceLorawanMac>();
        mac->SetDataRate(4);

        sfQuantity[1]++;
        sfQuantity[2]--;
    }

    for (int i = 0; i < vecSf10.size() * 0.05; i++)
    {
        EdAndPr data = vecSf10[i];

        Ptr<Node> node = endDevices.Get(data.m_ed);
        Ptr<LoraNetDevice> dev = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = dev->GetMac()->GetObject<EndDeviceLorawanMac>();
        mac->SetDataRate(3);

        sfQuantity[2]++;
        sfQuantity[3]--;
    }

    for (int i = 0; i < vecSf11.size() * 0.05; i++)
    {
        EdAndPr data = vecSf11[i];

        Ptr<Node> node = endDevices.Get(data.m_ed);
        Ptr<LoraNetDevice> dev = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = dev->GetMac()->GetObject<EndDeviceLorawanMac>();
        mac->SetDataRate(2);

        sfQuantity[3]++;
        sfQuantity[4]--;
    }

    for (int i = 0; i < vecSf12.size() * 0.05; i++)
    {
        EdAndPr data = vecSf12[i];

        Ptr<Node> node = endDevices.Get(data.m_ed);
        Ptr<LoraNetDevice> dev = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = dev->GetMac()->GetObject<EndDeviceLorawanMac>();
        mac->SetDataRate(1);

        sfQuantity[4]++;
        if (sfQuantity[5] > 0)
        {
            sfQuantity[5]--;
        }
        else 
        {
            sfQuantity[6]--;
        }
    }

    vecSf8.clear();
    vecSf9.clear();
    vecSf10.clear();
    vecSf11.clear();
    vecSf12.clear();

    return sfQuantity;
}

std::vector<int> 
LorawanMacHelper::RSFA(NodeContainer endDevices,
                      NodeContainer gateways,
                      Ptr<LoraChannel> channel,
                      bool useGwSens)
{
    NS_LOG_FUNCTION_NOARGS();

    std::map<int, GwInfoT> gwInfoMap;

    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);
        gwInfoMap.insert(std::make_pair((int) gw->GetId(), GwInfoT()));
    }

    std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy> (gatewayLoraNetDevice->GetPhy ()); 
        const double *gwSensitivity = gatewayPhy->sensitivity; // gateway-lora-phy.cc => loc 131

        if(rxPower > *gwSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 7);
        }
        else if (rxPower > *(gwSensitivity+1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 8);
        }
        else if (rxPower > *(gwSensitivity+2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 9);
        }
        else if (rxPower > *(gwSensitivity+3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 10);
        }
        else if (rxPower > *(gwSensitivity+4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 11);
        }
        else if (rxPower > *(gwSensitivity+5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
        else // Device is out of range. Assign SF12.
        {
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
    } // end loop on nodes

    std::vector<double> toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};

    int t = 600;
    double succ = 0.99;
    int nFreqs = 3;

    int nSF7 = NumMaxOfNodesPerSF(toas[0], t, succ, nFreqs);
    int nSF8 = NumMaxOfNodesPerSF(toas[1], t, succ, nFreqs);
    int nSF9 = NumMaxOfNodesPerSF(toas[2], t, succ, nFreqs);
    int nSF10 = NumMaxOfNodesPerSF(toas[3], t, succ, nFreqs);
    int nSF11 = NumMaxOfNodesPerSF(toas[4], t, succ, nFreqs);
    int nSF12 = NumMaxOfNodesPerSF(toas[5], t, succ, nFreqs);

    Ptr<UniformRandomVariable> uniformRV = CreateObject<UniformRandomVariable>();

    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);

        auto it = gwInfoMap.find(gw->GetId());
        
        // 7 => 8
        while ((int) it->second.m_sf8.size() < nSF8)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf8.size() < nSF8)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                
                it->second.m_sf8.push_back(it->second.m_sf7[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(4);

                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7)
            {
                break;
            }
        }

        // {7, 8} => 9
        while ((int) it->second.m_sf9.size() < nSF9)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf9.size() < nSF9)
            {                

                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                
                it->second.m_sf9.push_back(it->second.m_sf7[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(3);

                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf9.size() < nSF9)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                
                it->second.m_sf9.push_back(it->second.m_sf8[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(3);

                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8)
            {
                break;
            }
        }

        // {7, 8, 9} => 10
        while ((int) it->second.m_sf10.size() < nSF10)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                
                it->second.m_sf10.push_back(it->second.m_sf7[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(2);

                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                
                it->second.m_sf10.push_back(it->second.m_sf8[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(2);

                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                
                it->second.m_sf10.push_back(it->second.m_sf9[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(2);

                it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                    && (int) it->second.m_sf9.size() <= nSF9)
            {
                break;
            }
        }

        // {7, 8, 9, 10} => 11
        while ((int) it->second.m_sf11.size() < nSF11)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                
                it->second.m_sf11.push_back(it->second.m_sf7[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                
                it->second.m_sf11.push_back(it->second.m_sf8[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                
                it->second.m_sf11.push_back(it->second.m_sf9[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size() - 1);
                
                it->second.m_sf11.push_back(it->second.m_sf10[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10)
            {
                break;
            }
        }

        // {7, 8, 9, 10, 11} => 12
        while ((int) it->second.m_sf12.size() < nSF12)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                
                it->second.m_sf12.push_back(it->second.m_sf7[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                
                it->second.m_sf12.push_back(it->second.m_sf8[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                
                it->second.m_sf12.push_back(it->second.m_sf9[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size() - 1);
                
                it->second.m_sf12.push_back(it->second.m_sf10[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
            }

            if ((int) it->second.m_sf11.size() > nSF11 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf11.size() - 1);
                
                it->second.m_sf12.push_back(it->second.m_sf11[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf11[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                it->second.m_sf11.erase(it->second.m_sf11.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10
                && (int) it->second.m_sf11.size() <= nSF11)
            {
                break;
            }
        }
    }

    // Clear Data
    toas.clear();

    for (auto it: gwInfoMap)
    {
        it.second.Clear();
    }
    gwInfoMap.clear();

    std::cout << "R-SFA\n";

    // Returning
    return sfQuantity;
}  

/*
 * O mesmo que RSFA, mas com parametros para o cálculo 
 * do número maximo de nós por SF.
 */ 
std::vector<int> 
LorawanMacHelper::RSFA1(NodeContainer endDevices,
                        NodeContainer gateways,
                        Ptr<LoraChannel> channel,
                        std::vector<double> toas,
                        bool useGwSens,
                        double T,
                        double pSucc,
                        int nFreqs)
{
    NS_LOG_FUNCTION_NOARGS();

    std::map<int, GwInfoT> gwInfoMap;
    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);
        gwInfoMap.insert(std::make_pair((int) gw->GetId(), GwInfoT()));
    }

    std::vector<int> sfQuantity(6, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy> (gatewayLoraNetDevice->GetPhy ()); 
        const double *gwSensitivity = gatewayPhy->sensitivity; // gateway-lora-phy.cc => loc 131

        if(rxPower > *gwSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 7);
        }
        else if (rxPower > *(gwSensitivity+1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 8);
        }
        else if (rxPower > *(gwSensitivity+2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 9);
        }
        else if (rxPower > *(gwSensitivity+3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 10);
        }
        else if (rxPower > *(gwSensitivity+4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 11);
        }
        else if (rxPower > *(gwSensitivity+5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
        else // Device is out of range. Assign SF12.
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
    } // end loop on nodes

    /*std::cout << "SF Inicial\n";
    for (auto sf: sfQuantity)
    {
        std::cout << sf << " ";
    }
    std::cout << std::endl;
    for (int i = 0; i < 50; i++)
    {
        std::cout << "#";
    }
    std::cout << std::endl;
    sfQuantity.clear();*/

    /*std::vector<double> toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};
    int t = 600;
    double succ = 0.99;
    int nFreqs = 3;*/

    int nSF7 = NumMaxOfNodesPerSF(toas[0], T, pSucc, nFreqs);
    int nSF8 = NumMaxOfNodesPerSF(toas[1], T, pSucc, nFreqs);
    int nSF9 = NumMaxOfNodesPerSF(toas[2], T, pSucc, nFreqs);
    int nSF10 = NumMaxOfNodesPerSF(toas[3], T, pSucc, nFreqs);
    int nSF11 = NumMaxOfNodesPerSF(toas[4], T, pSucc, nFreqs);
    int nSF12 = NumMaxOfNodesPerSF(toas[5], T, pSucc, nFreqs);

    Ptr<UniformRandomVariable> uniformRV = CreateObject<UniformRandomVariable>();

    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);

        auto it = gwInfoMap.find(gw->GetId());
        
        // 7 => 8
        while ((int) it->second.m_sf8.size() < nSF8)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf8.size() < nSF8)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                
                it->second.m_sf8.push_back(it->second.m_sf7[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(4);

                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7)
            {
                break;
            }
        }

        // {7, 8} => 9
        while ((int) it->second.m_sf9.size() < nSF9)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf9.size() < nSF9)
            {                

                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                
                it->second.m_sf9.push_back(it->second.m_sf7[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(3);

                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf9.size() < nSF9)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                
                it->second.m_sf9.push_back(it->second.m_sf8[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(3);

                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8)
            {
                break;
            }
        }

        // {7, 8, 9} => 10
        while ((int) it->second.m_sf10.size() < nSF10)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                
                it->second.m_sf10.push_back(it->second.m_sf7[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(2);

                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                
                it->second.m_sf10.push_back(it->second.m_sf8[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(2);

                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                
                it->second.m_sf10.push_back(it->second.m_sf9[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(2);

                it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                    && (int) it->second.m_sf9.size() <= nSF9)
            {
                break;
            }
        }

        // {7, 8, 9, 10} => 11
        while ((int) it->second.m_sf11.size() < nSF11)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                
                it->second.m_sf11.push_back(it->second.m_sf7[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                
                it->second.m_sf11.push_back(it->second.m_sf8[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                
                it->second.m_sf11.push_back(it->second.m_sf9[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size() - 1);
                
                it->second.m_sf11.push_back(it->second.m_sf10[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10)
            {
                break;
            }
        }

        // {7, 8, 9, 10, 11} => 12
        while ((int) it->second.m_sf12.size() < nSF12)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                
                it->second.m_sf12.push_back(it->second.m_sf7[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                
                it->second.m_sf12.push_back(it->second.m_sf8[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                
                it->second.m_sf12.push_back(it->second.m_sf9[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size() - 1);
                
                it->second.m_sf12.push_back(it->second.m_sf10[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
            }

            if ((int) it->second.m_sf11.size() > nSF11 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf11.size() - 1);
                
                it->second.m_sf12.push_back(it->second.m_sf11[index]);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf11[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                it->second.m_sf11.erase(it->second.m_sf11.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10
                && (int) it->second.m_sf11.size() <= nSF11)
            {
                break;
            }
        }
    }

    std::vector<int> sfDist(6, 0);
    for (uint32_t i = 0; i < endDevices.GetN(); i++)
    {
        Ptr<Node> ed = endDevices.Get(i);
        Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();

        sfDist[5 - mac->GetDataRate()]++;
    }

    /*for (auto sf: sfDist)
    {
        std::cout << sf << " ";
    }
    std::cout << "\n";
    for (int i = 0 ; i < 50; i++)
    {
        std::cout << "#";
    }
    std::cout << std::endl;*/
    
    std::cout << "R-SFA\n";

    // Clear Data
    toas.clear();

    for (auto it: gwInfoMap)
    {
        it.second.Clear();
    }
    gwInfoMap.clear();

    sfQuantity.clear();

    // Returning
    // return sfQuantity;
    return sfDist;
}

std::vector<int> 
LorawanMacHelper::DRSFA(NodeContainer endDevices,
                        NodeContainer gateways,
                        Ptr<LoraChannel> channel,
                        std::vector<double> maxDelays,
                        int nRun,
                        bool useGwSens, 
                        double T,
                        double pSucc)
{
    NS_LOG_FUNCTION_NOARGS();

    std::map<int, GwInfoT> gwInfoMap;

    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);
        gwInfoMap.insert(std::make_pair((int) gw->GetId(), GwInfoT()));
    }

    std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy> (gatewayLoraNetDevice->GetPhy ()); 
        const double *gwSensitivity = gatewayPhy->sensitivity; // gateway-lora-phy.cc => loc 131

        if(rxPower > *gwSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 7);
        }
        else if (rxPower > *(gwSensitivity+1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 8);
        }
        else if (rxPower > *(gwSensitivity+2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 9);
        }
        else if (rxPower > *(gwSensitivity+3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 10);
        }
        else if (rxPower > *(gwSensitivity+4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 11);

            //std::cout << object->GetId() << ", " << maxDelays[object->GetId()] << ", " << nRun << " SF11\n";
        }
        else if (rxPower > *(gwSensitivity+5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
        else // Device is out of range. Assign SF12.
        {
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
    } // end loop on nodes

    std::vector<double> toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};

    // int t = 600;
    // double succ = 0.99;
    int nFreqs = 3;

    int nSF7 = NumMaxOfNodesPerSF(toas[0], T, pSucc, nFreqs);
    int nSF8 = NumMaxOfNodesPerSF(toas[1], T, pSucc, nFreqs);
    int nSF9 = NumMaxOfNodesPerSF(toas[2], T, pSucc, nFreqs);
    int nSF10 = NumMaxOfNodesPerSF(toas[3], T, pSucc, nFreqs);
    int nSF11 = NumMaxOfNodesPerSF(toas[4], T, pSucc, nFreqs);
    int nSF12 = NumMaxOfNodesPerSF(toas[5], T, pSucc, nFreqs);

    /*std::cout << nSF7 << ", " << nSF8 << ", " << nSF9 << ", " << nSF10 << ", " << nSF11 << ", " << nSF12 
            << std::endl;
    
    for (size_t i = 0; i < sfQuantity.size() - 2; i++)
    {
        std::cout << sfQuantity[i] << ", ";
    }
    std::cout << (sfQuantity[5] + sfQuantity[6]) << std::endl;*/

    Ptr<UniformRandomVariable> uniformRV = CreateObject<UniformRandomVariable>();

    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);

        auto it = gwInfoMap.find(gw->GetId());
        
        // 7 => 8
        while ((int) it->second.m_sf8.size() < nSF8)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf8.size() < nSF8)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[1] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[1])
                {                   
                    it->second.m_sf8.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(4);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7)
            {
                break;
            }
        }

        // {7, 8} => 9
        while ((int) it->second.m_sf9.size() < nSF9)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf9.size() < nSF9)
            {                
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[2] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[2])
                {
                    it->second.m_sf9.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(3);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf9.size() < nSF9)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[2] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }

                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[2])
                {
                    it->second.m_sf9.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(3);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8)
            {
                break;
            }
        }

        // {7, 8, 9} => 10
        while ((int) it->second.m_sf10.size() < nSF10)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[3] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[3])
                {
                    it->second.m_sf10.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(2);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[3] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }

                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[3])
                {                
                    it->second.m_sf10.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(2);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                int nodeId = it->second.m_sf9[index];
                while ((int) it->second.m_sf9.size() > nSF9 && toas[3] >= maxDelays[nodeId])
                {
                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);   
                }

                if ((int) it->second.m_sf9.size() > nSF9 && maxDelays[nodeId] > toas[3])
                {
                    it->second.m_sf10.push_back(it->second.m_sf9[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(2);

                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                    && (int) it->second.m_sf9.size() <= nSF9)
            {
                break;
            }
        }

        // {7, 8, 9, 10} => 11
        while ((int) it->second.m_sf11.size() < nSF11)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[4])
                {                
                    it->second.m_sf11.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }
                
                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[4])
                {
                    it->second.m_sf11.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                int nodeId = it->second.m_sf9[index];
                while ((int) it->second.m_sf9.size() > nSF9 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);   
                }
                
                if ((int) it->second.m_sf9.size() > nSF9 && maxDelays[nodeId] > toas[4])
                {
                    it->second.m_sf11.push_back(it->second.m_sf9[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
                }
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size() - 1);
                int nodeId = it->second.m_sf10[index];
                while ((int) it->second.m_sf10.size() > nSF10 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);   
                }
                
                if ((int) it->second.m_sf10.size() > nSF10 && maxDelays[nodeId] > toas[4])
                {
                    it->second.m_sf11.push_back(it->second.m_sf10[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10)
            {
                break;
            }
        }

        // {7, 8, 9, 10, 11} => 12
        while ((int) it->second.m_sf12.size() < nSF12)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }
                
                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }
                
                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                int nodeId = it->second.m_sf9[index];
                while ((int) it->second.m_sf9.size() > nSF9 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);   
                }
                
                if ((int) it->second.m_sf9.size() > nSF9 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf9[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
                }
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size() - 1);
                int nodeId = it->second.m_sf10[index];
                while ((int) it->second.m_sf10.size() > nSF10 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);   
                }
                
                if ((int) it->second.m_sf10.size() > nSF10 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf10[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
                }
            }

            if ((int) it->second.m_sf11.size() > nSF11 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf11.size() - 1);
                int nodeId = it->second.m_sf11[index];
                while ((int) it->second.m_sf11.size() > nSF11 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf11.erase(it->second.m_sf11.begin() + index);   
                }
                
                if ((int) it->second.m_sf11.size() > nSF11 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf11[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf11[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    it->second.m_sf11.erase(it->second.m_sf11.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10
                && (int) it->second.m_sf11.size() <= nSF11)
            {
                break;
            }
        }
    }

    // Clear Data
    toas.clear();

    for (auto it: gwInfoMap)
    {
        it.second.Clear();
    }
    gwInfoMap.clear();

    std::cout << "DR-SFA\n";

    // Returning
    return sfQuantity;
}  

std::vector<int> 
LorawanMacHelper::SFTPA(NodeContainer endDevices,
                        NodeContainer gateways,
                        Ptr<LoraChannel> channel,
                        std::vector<double> maxDelays,
                        bool useGwSens, 
                        double T,
                        double pSucc)
{
    NS_LOG_FUNCTION_NOARGS();

    std::map<int, GwInfoT> gwInfoMap;

    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);
        gwInfoMap.insert(std::make_pair((int) gw->GetId(), GwInfoT()));
    }

    std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy> (gatewayLoraNetDevice->GetPhy ()); 
        const double *gwSensitivity = gatewayPhy->sensitivity; // gateway-lora-phy.cc => loc 131

        if(rxPower > *gwSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 7);
        }
        else if (rxPower > *(gwSensitivity+1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 8);
        }
        else if (rxPower > *(gwSensitivity+2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 9);
        }
        else if (rxPower > *(gwSensitivity+3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 10);
        }
        else if (rxPower > *(gwSensitivity+4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 11);

            //std::cout << object->GetId() << ", " << maxDelays[object->GetId()] << ", " << nRun << " SF11\n";
        }
        else if (rxPower > *(gwSensitivity+5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
        else // Device is out of range. Assign SF12.
        {
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
    } // end loop on nodes

    std::vector<double> toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};

    // int t = 600;
    // double succ = 0.99;
    int nFreqs = 3;

    int nSF7 = NumMaxOfNodesPerSF(toas[0], T, pSucc, nFreqs);
    int nSF8 = NumMaxOfNodesPerSF(toas[1], T, pSucc, nFreqs);
    int nSF9 = NumMaxOfNodesPerSF(toas[2], T, pSucc, nFreqs);
    int nSF10 = NumMaxOfNodesPerSF(toas[3], T, pSucc, nFreqs);
    int nSF11 = NumMaxOfNodesPerSF(toas[4], T, pSucc, nFreqs);
    int nSF12 = NumMaxOfNodesPerSF(toas[5], T, pSucc, nFreqs);

    /*std::cout << nSF7 << ", " << nSF8 << ", " << nSF9 << ", " << nSF10 << ", " << nSF11 << ", " << nSF12 
            << std::endl;
    
    for (size_t i = 0; i < sfQuantity.size() - 2; i++)
    {
        std::cout << sfQuantity[i] << ", ";
    }
    std::cout << (sfQuantity[5] + sfQuantity[6]) << std::endl;*/

    Ptr<UniformRandomVariable> uniformRV = CreateObject<UniformRandomVariable>();
    std::vector<double> sensValues = {-130.0, -132.5, -135.0, -137.5, -140.0, -142.5};
    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);

        auto it = gwInfoMap.find(gw->GetId());
        
        // 7 => 8
        while ((int) it->second.m_sf8.size() < nSF8)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf8.size() < nSF8)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[1] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[1])
                {                   
                    it->second.m_sf8.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(4);
                    
                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[0] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7)
            {
                break;
            }
        }

        // {7, 8} => 9
        while ((int) it->second.m_sf9.size() < nSF9)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf9.size() < nSF9)
            {                
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[2] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[2])
                {
                    it->second.m_sf9.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(3);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[1] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf9.size() < nSF9)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[2] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }

                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[2])
                {
                    it->second.m_sf9.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(3);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[1] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8)
            {
                break;
            }
        }

        // {7, 8, 9} => 10
        while ((int) it->second.m_sf10.size() < nSF10)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[3] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[3])
                {
                    it->second.m_sf10.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(2);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[2] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[3] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }

                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[3])
                {                
                    it->second.m_sf10.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(2);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[2] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                int nodeId = it->second.m_sf9[index];
                while ((int) it->second.m_sf9.size() > nSF9 && toas[3] >= maxDelays[nodeId])
                {
                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);   
                }

                if ((int) it->second.m_sf9.size() > nSF9 && maxDelays[nodeId] > toas[3])
                {
                    it->second.m_sf10.push_back(it->second.m_sf9[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(2);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[2] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                    && (int) it->second.m_sf9.size() <= nSF9)
            {
                break;
            }
        }

        // {7, 8, 9, 10} => 11
        while ((int) it->second.m_sf11.size() < nSF11)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[4])
                {                
                    it->second.m_sf11.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[3] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }
                
                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[4])
                {
                    it->second.m_sf11.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[3] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                int nodeId = it->second.m_sf9[index];
                while ((int) it->second.m_sf9.size() > nSF9 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);   
                }
                
                if ((int) it->second.m_sf9.size() > nSF9 && maxDelays[nodeId] > toas[4])
                {
                    it->second.m_sf11.push_back(it->second.m_sf9[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[3] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
                }
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size() - 1);
                int nodeId = it->second.m_sf10[index];
                while ((int) it->second.m_sf10.size() > nSF10 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);   
                }
                
                if ((int) it->second.m_sf10.size() > nSF10 && maxDelays[nodeId] > toas[4])
                {
                    it->second.m_sf11.push_back(it->second.m_sf10[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[3] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10)
            {
                break;
            }
        }

        // {7, 8, 9, 10, 11} => 12
        while ((int) it->second.m_sf12.size() < nSF12)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }
                
                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[4] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }
                
                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[4] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                int nodeId = it->second.m_sf9[index];
                while ((int) it->second.m_sf9.size() > nSF9 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);   
                }
                
                if ((int) it->second.m_sf9.size() > nSF9 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf9[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[4] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
                }
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size() - 1);
                int nodeId = it->second.m_sf10[index];
                while ((int) it->second.m_sf10.size() > nSF10 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);   
                }
                
                if ((int) it->second.m_sf10.size() > nSF10 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf10[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[4] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
                }
            }

            if ((int) it->second.m_sf11.size() > nSF11 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf11.size() - 1);
                int nodeId = it->second.m_sf11[index];
                while ((int) it->second.m_sf11.size() > nSF11 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf11.erase(it->second.m_sf11.begin() + index);   
                }
                
                if ((int) it->second.m_sf11.size() > nSF11 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf11[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf11[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[4] && newTP >= 2)
                    {
                        newTP -= 2;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 2;
                    mac->SetTxPower(newTP);

                    it->second.m_sf11.erase(it->second.m_sf11.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10
                && (int) it->second.m_sf11.size() <= nSF11)
            {
                break;
            }
        }
    }

    // Clear Data
    toas.clear();

    for (auto it: gwInfoMap)
    {
        it.second.Clear();
    }
    gwInfoMap.clear();

    std::cout << "SFTPA\n";

    // Returning
    return sfQuantity;
}

std::vector<int> 
LorawanMacHelper::ITPA(NodeContainer endDevices,
                      NodeContainer gateways,
                      Ptr<LoraChannel> channel,
                      bool useGwSens)
{
    NS_LOG_FUNCTION_NOARGS();

    std::vector<EdAndPr> vecSF7;
    std::vector<EdAndPr> vecSF8;
    std::vector<EdAndPr> vecSF9;
    std::vector<EdAndPr> vecSF10;
    std::vector<EdAndPr> vecSF11;
    std::vector<EdAndPr> vecSF12;

    std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy> (gatewayLoraNetDevice->GetPhy ()); 
        const double *gwSensitivity = gatewayPhy->sensitivity; // gateway-lora-phy.cc => loc 131

        if(rxPower > *gwSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;

            vecSF7.push_back(EdAndPr(object->GetId(), rxPower, bestGateway->GetId()));
        }
        else if (rxPower > *(gwSensitivity+1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;

            vecSF8.push_back(EdAndPr(object->GetId(), rxPower, bestGateway->GetId()));
        }
        else if (rxPower > *(gwSensitivity+2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;

            vecSF9.push_back(EdAndPr(object->GetId(), rxPower, bestGateway->GetId()));
        }
        else if (rxPower > *(gwSensitivity+3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;

            vecSF10.push_back(EdAndPr(object->GetId(), rxPower, bestGateway->GetId()));
        }
        else if (rxPower > *(gwSensitivity+4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;

            vecSF11.push_back(EdAndPr(object->GetId(), rxPower, bestGateway->GetId()));
        }
        else if (rxPower > *(gwSensitivity+5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;

            vecSF12.push_back(EdAndPr(object->GetId(), rxPower, bestGateway->GetId()));
        }
        else // Device is out of range. Assign SF12.
        {
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;

            vecSF12.push_back(EdAndPr(object->GetId(), rxPower, bestGateway->GetId()));
        }
    } // end loop on nodes


    std::vector<double> toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};
    Ptr<UniformRandomVariable> uniformRV = CreateObject<UniformRandomVariable>();

    int numMaxSF7 = NumMaxOfNodesPerSF(toas[0], 10 * 60, 0.99, 3);
    int numMaxSF8 = NumMaxOfNodesPerSF(toas[1], 10 * 60, 0.99, 3);
    int numMaxSF9 = NumMaxOfNodesPerSF(toas[2], 10 * 60, 0.99, 3);
    int numMaxSF10 = NumMaxOfNodesPerSF(toas[3], 10 * 60, 0.99, 3);
    int numMaxSF11 = NumMaxOfNodesPerSF(toas[4], 10 * 60, 0.99, 3);
    int numMaxSF12 = NumMaxOfNodesPerSF(toas[5], 10 * 60, 0.99, 3);

    while ((int) vecSF8.size() < numMaxSF8)
    {
        if ((int) vecSF7.size() <= numMaxSF7)
        {
            break;
        }

        uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF7.size() - 1);
        EdAndPr data = vecSF7[i];

        Ptr<Node> node = endDevices.Get(data.m_ed);
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<ClassAEndDeviceLorawanMac> mac = 
            loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
        mac->SetDataRate(4);

        vecSF8.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
        vecSF7.erase(vecSF7.begin() + i);
    }

    while ((int) vecSF9.size() < numMaxSF9)
    {
        bool toBeContinue = false;

        if ((int) vecSF7.size() > numMaxSF7)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF7.size() - 1);
            EdAndPr data = vecSF7[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(3);

            vecSF9.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF7.erase(vecSF7.begin() + i);

            toBeContinue = true;
        }

        if ((int) vecSF8.size() > numMaxSF8)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF8.size() - 1);
            EdAndPr data = vecSF8[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(3);

            vecSF9.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF8.erase(vecSF8.begin() + i);

            toBeContinue = true;
        }

        if (!toBeContinue)
        {
            break;
        }
    }

    while ((int) vecSF10.size() < numMaxSF10)
    {
        bool toBeContinue = false;

        if ((int) vecSF7.size() > numMaxSF7)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF7.size() - 1);
            EdAndPr data = vecSF7[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(2);

            vecSF10.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF7.erase(vecSF7.begin() + i);

            toBeContinue = true;
        }

        if ((int) vecSF8.size() > numMaxSF8)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF8.size() - 1);
            EdAndPr data = vecSF8[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(2);

            vecSF10.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF8.erase(vecSF8.begin() + i);

            toBeContinue = true;
        }

        if ((int) vecSF9.size() > numMaxSF9)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF9.size() - 1);
            EdAndPr data = vecSF9[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(2);

            vecSF10.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF9.erase(vecSF9.begin() + i);

            toBeContinue = true;
        }

        if (!toBeContinue)
        {
            break;
        }
    }

    while ((int) vecSF11.size() < numMaxSF11)
    {
        bool toBeContinue = false;

        if ((int) vecSF7.size() > numMaxSF7)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF7.size() - 1);
            EdAndPr data = vecSF7[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(1);

            vecSF11.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF7.erase(vecSF7.begin() + i);

            toBeContinue = true;
        }

        if ((int) vecSF8.size() > numMaxSF8)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF8.size() - 1);
            EdAndPr data = vecSF8[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(1);

            vecSF11.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF8.erase(vecSF8.begin() + i);

            toBeContinue = true;
        }

        if ((int) vecSF9.size() > numMaxSF9)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF9.size() - 1);
            EdAndPr data = vecSF9[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(1);

            vecSF11.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF9.erase(vecSF9.begin() + i);

            toBeContinue = true;
        }

        if ((int) vecSF10.size() > numMaxSF10)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF10.size() - 1);
            EdAndPr data = vecSF10[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(1);

            vecSF11.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF10.erase(vecSF10.begin() + i);

            toBeContinue = true;
        }

        if (!toBeContinue)
        {
            break;
        }
    }

    while ((int) vecSF12.size() < numMaxSF12)
    {
        bool toBeContinue = false;

        if ((int) vecSF7.size() > numMaxSF7)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF7.size() - 1);
            EdAndPr data = vecSF7[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(0);

            vecSF12.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF7.erase(vecSF7.begin() + i);

            toBeContinue = true;
        }

        if ((int) vecSF8.size() > numMaxSF8)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF8.size() - 1);
            EdAndPr data = vecSF8[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(0);

            vecSF12.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF8.erase(vecSF8.begin() + i);

            toBeContinue = true;
        }

        if ((int) vecSF9.size() > numMaxSF9)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF9.size() - 1);
            EdAndPr data = vecSF9[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(0);

            vecSF12.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF9.erase(vecSF9.begin() + i);

            toBeContinue = true;
        }

        if ((int) vecSF10.size() > numMaxSF10)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF10.size() - 1);
            EdAndPr data = vecSF10[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(0);

            vecSF12.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF10.erase(vecSF10.begin() + i);

            toBeContinue = true;
        }

        if ((int) vecSF11.size() > numMaxSF11)
        {
            uint32_t i = uniformRV->GetInteger(0, (uint32_t) vecSF11.size() - 1);
            EdAndPr data = vecSF11[i];

            Ptr<Node> node = endDevices.Get(data.m_ed);
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<ClassAEndDeviceLorawanMac> mac = 
                loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
            mac->SetDataRate(0);

            vecSF12.push_back(EdAndPr(data.m_ed, data.m_gw, data.m_pr));
            vecSF11.erase(vecSF11.begin() + i);

            toBeContinue = true;
        }

        if (!toBeContinue)
        {
            break;
        }
    }

    toas.clear();
    vecSF7.clear();
    vecSF8.clear();
    vecSF9.clear();
    vecSF10.clear();
    vecSF11.clear();
    vecSF12.clear();

    return sfQuantity;
}

std::vector<int> 
LorawanMacHelper::SFTPA1(NodeContainer endDevices,
                         NodeContainer gateways,
                         Ptr<LoraChannel> channel,
                         std::vector<double> toas,
                         int nFreqs,
                         bool useGwSens, 
                         double T,
                         double pSucc)
{
    NS_LOG_FUNCTION_NOARGS();

    std::map<int, GwInfoT> gwInfoMap;

    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);
        gwInfoMap.insert(std::make_pair((int) gw->GetId(), GwInfoT()));
    }

    // std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy> (gatewayLoraNetDevice->GetPhy ()); 
        const double *gwSensitivity = gatewayPhy->sensitivity; // gateway-lora-phy.cc => loc 131

        if(rxPower > *gwSensitivity)
        {
            mac->SetDataRate(5);
            // sfQuantity[0] = sfQuantity[0] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 7);
        }
        else if (rxPower > *(gwSensitivity+1))
        {
            mac->SetDataRate(4);
            // sfQuantity[1] = sfQuantity[1] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 8);
        }
        else if (rxPower > *(gwSensitivity+2))
        {
            mac->SetDataRate(3);
            // sfQuantity[2] = sfQuantity[2] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 9);
        }
        else if (rxPower > *(gwSensitivity+3))
        {
            mac->SetDataRate(2);
            // sfQuantity[3] = sfQuantity[3] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 10);
        }
        else if (rxPower > *(gwSensitivity+4))
        {
            mac->SetDataRate(1);
            // sfQuantity[4] = sfQuantity[4] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 11);

            //std::cout << object->GetId() << ", " << maxDelays[object->GetId()] << ", " << nRun << " SF11\n";
        }
        else if (rxPower > *(gwSensitivity+5))
        {
            mac->SetDataRate(0);
            // sfQuantity[5] = sfQuantity[5] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
        else // Device is out of range. Assign SF12.
        {
            mac->SetDataRate(0);
            // sfQuantity[6] = sfQuantity[6] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
    } // end loop on nodes

    // std::vector<double> toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};
    // int t = 600;
    // double succ = 0.99;
    // int nFreqs = 3;

    int nSF7 = NumMaxOfNodesPerSF(toas[0], T, pSucc, nFreqs);
    int nSF8 = NumMaxOfNodesPerSF(toas[1], T, pSucc, nFreqs);
    int nSF9 = NumMaxOfNodesPerSF(toas[2], T, pSucc, nFreqs);
    int nSF10 = NumMaxOfNodesPerSF(toas[3], T, pSucc, nFreqs);
    int nSF11 = NumMaxOfNodesPerSF(toas[4], T, pSucc, nFreqs);
    int nSF12 = NumMaxOfNodesPerSF(toas[5], T, pSucc, nFreqs);

    Ptr<UniformRandomVariable> uniformRV = CreateObject<UniformRandomVariable>();
    std::vector<double> sensValues = {-130.0, -132.5, -135.0, -137.5, -140.0, -142.5};
    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);

        auto it = gwInfoMap.find(gw->GetId());
        
        // 7 => 8
        while ((int) it->second.m_sf8.size() < nSF8)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf8.size() < nSF8)
            {                   
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(4);
                
                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[0] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf8.push_back(it->second.m_sf7[index]);
                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7)
            {
                break;
            }
        }

        // {7, 8} => 9
        while ((int) it->second.m_sf9.size() < nSF9)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf9.size() < nSF9)
            {                
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(3);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[1] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf9.push_back(it->second.m_sf7[index]);
                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf9.size() < nSF9)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(3);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[1] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf9.push_back(it->second.m_sf8[index]);
                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8)
            {
                break;
            }
        }

        // {7, 8, 9} => 10
        while ((int) it->second.m_sf10.size() < nSF10)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(2);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[2] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf10.push_back(it->second.m_sf7[index]);
                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(2);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[2] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf10.push_back(it->second.m_sf8[index]);
                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);

                Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(2);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[2] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf10.push_back(it->second.m_sf9[index]);
                it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                    && (int) it->second.m_sf9.size() <= nSF9)
            {
                break;
            }
        }

        // {7, 8, 9, 10} => 11
        while ((int) it->second.m_sf11.size() < nSF11)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[3] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf11.push_back(it->second.m_sf7[index]);
                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf11.size() < nSF11)
            {    
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[3] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf11.push_back(it->second.m_sf8[index]);
                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[3] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf11.push_back(it->second.m_sf9[index]);
                it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf11.size() < nSF11)
            {                
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(1);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[3] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf11.push_back(it->second.m_sf10[index]);
                it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10)
            {
                break;
            }
        }

        // {7, 8, 9, 10, 11} => 12
        while ((int) it->second.m_sf12.size() < nSF12)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf12.size() < nSF12)
            { 
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[4] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf12.push_back(it->second.m_sf7[index]);
                it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf12.size() < nSF12)
            {    
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[4] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);
                
                it->second.m_sf12.push_back(it->second.m_sf8[index]);
                it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf12.size() < nSF12)
            {                   
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[4] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);
                
                it->second.m_sf12.push_back(it->second.m_sf9[index]);
                it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf12.size() < nSF12)
            {                
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[4] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf12.push_back(it->second.m_sf10[index]);
                it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
            }

            if ((int) it->second.m_sf11.size() > nSF11 && (int) it->second.m_sf12.size() < nSF12)
            {                  
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf11.size());

                Ptr<Node> ed = endDevices.Get(it->second.m_sf11[index]);
                Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                mac->SetDataRate(0);

                double newTP = 14;
                double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                while(rssi > sensValues[4] && newTP >= 2)
                {
                    newTP -= 2;
                    rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                gw->GetObject<MobilityModel>());
                }
                newTP += 2;
                mac->SetTxPower(newTP);

                it->second.m_sf12.push_back(it->second.m_sf11[index]);
                it->second.m_sf11.erase(it->second.m_sf11.begin() + index);
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10
                && (int) it->second.m_sf11.size() <= nSF11)
            {
                break;
            }
        }
    }

    std::vector<int> sfQuantity(6, 0);
    for (uint32_t i = 0; i < endDevices.GetN(); i++)
    {
        Ptr<Node> ed = endDevices.Get(i);
        Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();

        sfQuantity[5 - mac->GetDataRate()]++;
    }

    // Clear Data
    toas.clear();

    for (auto it: gwInfoMap)
    {
        it.second.Clear();
    }
    gwInfoMap.clear();

    std::cout << "SFTPA\n";

    // Returning
    return sfQuantity;
}

// Thiago Allisson
double CalcSuccRate(const std::vector<double>& toas,
                    double t,
                    const std::vector<int>& Ns)
{
    double rate = 0.0;

    for (size_t i = 0; i < toas.size(); ++i)
    {
        if (Ns[i] > 0)
        {
            rate += Ns[i] * (3600.0 / t) * SuccProb(toas[i], t, 1.0 * Ns[i] / 3.0);
        }
    }

    return rate;
}


// Thiago Allisson
std::vector<int> CopyVector(std::vector<int> from)
{
    std::vector<int> to;
    for (auto v: from)
    {
        to.push_back(v);
    }

    return to;
}

// Thiago Allisson
void PrintVector(const std::vector<int>& vec)
{
    std::cout << "[";
    for (auto v: vec)
    {
        std::cout << v << " ";
    }
    std::cout << "]\n";
}                               

std::vector<int> 
LorawanMacHelper::SFTPAp(NodeContainer endDevices,
                         NodeContainer gateways,
                         Ptr<LoraChannel> channel,
                         std::vector<double> maxDelays,
                         bool useGwSens, 
                         double T,
                         double pSucc)
{
    NS_LOG_FUNCTION_NOARGS();

    std::map<int, GwInfoT> gwInfoMap;

    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);
        gwInfoMap.insert(std::make_pair((int) gw->GetId(), GwInfoT()));
    }

    std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = currPosition;
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = DynamicCast<LoraNetDevice>(gatewayNetDevice);
        Ptr<GatewayLoraPhy> gatewayPhy = DynamicCast<GatewayLoraPhy> (gatewayLoraNetDevice->GetPhy ()); 
        const double *gwSensitivity = gatewayPhy->sensitivity; // gateway-lora-phy.cc => loc 131

        if(rxPower > *gwSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 7);
        }
        else if (rxPower > *(gwSensitivity+1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 8);
        }
        else if (rxPower > *(gwSensitivity+2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 9);
        }
        else if (rxPower > *(gwSensitivity+3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 10);
        }
        else if (rxPower > *(gwSensitivity+4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 11);

            //std::cout << object->GetId() << ", " << maxDelays[object->GetId()] << ", " << nRun << " SF11\n";
        }
        else if (rxPower > *(gwSensitivity+5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
        else // Device is out of range. Assign SF12.
        {
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;

            auto it = gwInfoMap.find((int) bestGateway->GetId());
            it->second.AddNode((int) object->GetId(), 12);
        }
    } // end loop on nodes

    std::vector<double> toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};

    // int t = 600;
    // double succ = 0.99;
    int nFreqs = 3;

    int nSF7 = NumMaxOfNodesPerSF(toas[0], T, pSucc, nFreqs);
    int nSF8 = NumMaxOfNodesPerSF(toas[1], T, pSucc, nFreqs);
    int nSF9 = NumMaxOfNodesPerSF(toas[2], T, pSucc, nFreqs);
    int nSF10 = NumMaxOfNodesPerSF(toas[3], T, pSucc, nFreqs);
    int nSF11 = NumMaxOfNodesPerSF(toas[4], T, pSucc, nFreqs);
    int nSF12 = NumMaxOfNodesPerSF(toas[5], T, pSucc, nFreqs);

    /*std::cout << nSF7 << ", " << nSF8 << ", " << nSF9 << ", " << nSF10 << ", " << nSF11 << ", " << nSF12 
            << std::endl;
    
    for (size_t i = 0; i < sfQuantity.size() - 2; i++)
    {
        std::cout << sfQuantity[i] << ", ";
    }
    std::cout << (sfQuantity[5] + sfQuantity[6]) << std::endl;*/

    Ptr<UniformRandomVariable> uniformRV = CreateObject<UniformRandomVariable>();
    std::vector<double> sensValues = {-130.0, -132.5, -135.0, -137.5, -140.0, -142.5};
    for (uint32_t i = 0; i < gateways.GetN(); i++)
    {
        Ptr<Node> gw = gateways.Get(i);

        auto it = gwInfoMap.find(gw->GetId());
        
        // 7 => 8
        while ((int) it->second.m_sf8.size() < nSF8)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf8.size() < nSF8)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[1] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[1])
                {                   
                    it->second.m_sf8.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(4);
                    
                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[0] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7)
            {
                break;
            }
        }

        // {7, 8} => 9
        while ((int) it->second.m_sf9.size() < nSF9)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf9.size() < nSF9)
            {                
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[2] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[2])
                {
                    it->second.m_sf9.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(3);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[1] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf9.size() < nSF9)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[2] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }

                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[2])
                {
                    it->second.m_sf9.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(3);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[1] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8)
            {
                break;
            }
        }

        // {7, 8, 9} => 10
        while ((int) it->second.m_sf10.size() < nSF10)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[3] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[3])
                {
                    it->second.m_sf10.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(2);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[2] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[3] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }

                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[3])
                {                
                    it->second.m_sf10.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(2);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[2] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf10.size() < nSF10)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                int nodeId = it->second.m_sf9[index];
                while ((int) it->second.m_sf9.size() > nSF9 && toas[3] >= maxDelays[nodeId])
                {
                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);   
                }

                if ((int) it->second.m_sf9.size() > nSF9 && maxDelays[nodeId] > toas[3])
                {
                    it->second.m_sf10.push_back(it->second.m_sf9[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(2);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[2] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                    && (int) it->second.m_sf9.size() <= nSF9)
            {
                break;
            }
        }

        // {7, 8, 9, 10} => 11
        while ((int) it->second.m_sf11.size() < nSF11)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }

                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[4])
                {                
                    it->second.m_sf11.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[3] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }
                
                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[4])
                {
                    it->second.m_sf11.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[3] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                int nodeId = it->second.m_sf9[index];
                while ((int) it->second.m_sf9.size() > nSF9 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);   
                }
                
                if ((int) it->second.m_sf9.size() > nSF9 && maxDelays[nodeId] > toas[4])
                {
                    it->second.m_sf11.push_back(it->second.m_sf9[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[3] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
                }
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf11.size() < nSF11)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size() - 1);
                int nodeId = it->second.m_sf10[index];
                while ((int) it->second.m_sf10.size() > nSF10 && toas[4] >= maxDelays[nodeId])
                {
                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);   
                }
                
                if ((int) it->second.m_sf10.size() > nSF10 && maxDelays[nodeId] > toas[4])
                {
                    it->second.m_sf11.push_back(it->second.m_sf10[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(1);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[3] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10)
            {
                break;
            }
        }

        // {7, 8, 9, 10, 11} => 12
        while ((int) it->second.m_sf12.size() < nSF12)
        {
            if ((int) it->second.m_sf7.size() > nSF7 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf7.size() - 1);
                int nodeId = it->second.m_sf7[index];
                while ((int) it->second.m_sf7.size() > nSF7 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);   
                }
                
                if ((int) it->second.m_sf7.size() > nSF7 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf7[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf7[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[4] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf7.erase(it->second.m_sf7.begin() + index);
                }
            }

            if ((int) it->second.m_sf8.size() > nSF8 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf8.size() - 1);
                int nodeId = it->second.m_sf8[index];
                while ((int) it->second.m_sf8.size() > nSF8 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);   
                }
                
                if ((int) it->second.m_sf8.size() > nSF8 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf8[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf8[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[4] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf8.erase(it->second.m_sf8.begin() + index);
                }
            }

            if ((int) it->second.m_sf9.size() > nSF9 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf9.size() - 1);
                int nodeId = it->second.m_sf9[index];
                while ((int) it->second.m_sf9.size() > nSF9 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);   
                }
                
                if ((int) it->second.m_sf9.size() > nSF9 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf9[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf9[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[4] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf9.erase(it->second.m_sf9.begin() + index);
                }
            }

            if ((int) it->second.m_sf10.size() > nSF10 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf10.size() - 1);
                int nodeId = it->second.m_sf10[index];
                while ((int) it->second.m_sf10.size() > nSF10 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);   
                }
                
                if ((int) it->second.m_sf10.size() > nSF10 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf10[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf10[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[4] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf10.erase(it->second.m_sf10.begin() + index);
                }
            }

            if ((int) it->second.m_sf11.size() > nSF11 && (int) it->second.m_sf12.size() < nSF12)
            {
                int index = (int) uniformRV->GetInteger(0, it->second.m_sf11.size() - 1);
                int nodeId = it->second.m_sf11[index];
                while ((int) it->second.m_sf11.size() > nSF11 && toas[5] >= maxDelays[nodeId])
                {
                    it->second.m_sf11.erase(it->second.m_sf11.begin() + index);   
                }
                
                if ((int) it->second.m_sf11.size() > nSF11 && maxDelays[nodeId] > toas[5])
                {
                    it->second.m_sf12.push_back(it->second.m_sf11[index]);

                    Ptr<Node> ed = endDevices.Get(it->second.m_sf11[index]);
                    Ptr<LoraNetDevice> dev = ed->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<ClassAEndDeviceLorawanMac> mac = dev->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    mac->SetDataRate(0);

                    double newTP = 14;
                    double rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                      gw->GetObject<MobilityModel>());
                    while(rssi > sensValues[4] && newTP >= 1)
                    {
                        newTP -= 1;
                        rssi = channel->GetRxPower(newTP, ed->GetObject<MobilityModel>(), 
                                                    gw->GetObject<MobilityModel>());
                    }
                    newTP += 1;
                    if (newTP == 1)
                    {
                        newTP = 2;
                    }
                    mac->SetTxPower(newTP);

                    it->second.m_sf11.erase(it->second.m_sf11.begin() + index);
                }
            }

            if ((int) it->second.m_sf7.size() <= nSF7 && (int) it->second.m_sf8.size() <= nSF8 
                && (int) it->second.m_sf9.size() <= nSF9 && (int) it->second.m_sf10.size() <= nSF10
                && (int) it->second.m_sf11.size() <= nSF11)
            {
                break;
            }
        }
    }

    // Clear Data
    toas.clear();

    for (auto it: gwInfoMap)
    {
        it.second.Clear();
    }
    gwInfoMap.clear();

    std::cout << "SFTPA+\n";

    // Returning
    return sfQuantity;
}

std::vector<int>
LorawanMacHelper::SetSpreadingFactorsGivenDistribution(NodeContainer endDevices,
                                                       NodeContainer gateways,
                                                       std::vector<double> distribution)
{
    NS_LOG_FUNCTION_NOARGS();
    NS_ASSERT(distribution.size() == 6);

    std::vector<int> sfQuantity(7, 0);
    Ptr<UniformRandomVariable> uniformRV = CreateObject<UniformRandomVariable>();
    std::vector<double> cumdistr(6);
    cumdistr[0] = distribution[0];
    for (int i = 1; i < 6; ++i)
    {
        cumdistr[i] = distribution[i] + cumdistr[i - 1];
    }

    NS_LOG_DEBUG("Distribution: " << distribution[0] << " " << distribution[1] << " "
                                  << distribution[2] << " " << distribution[3] << " "
                                  << distribution[4] << " " << distribution[5]);
    NS_LOG_DEBUG("Cumulative distribution: " << cumdistr[0] << " " << cumdistr[1] << " "
                                             << cumdistr[2] << " " << cumdistr[3] << " "
                                             << cumdistr[4] << " " << cumdistr[5]);

    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
        NS_ASSERT(mac);

        double prob = uniformRV->GetValue(0, 1);

        // NS_LOG_DEBUG ("Probability: " << prob);
        if (prob < cumdistr[0])
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;
        }
        else if (prob > cumdistr[0] && prob < cumdistr[1])
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;
        }
        else if (prob > cumdistr[1] && prob < cumdistr[2])
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;
        }
        else if (prob > cumdistr[2] && prob < cumdistr[3])
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;
        }
        else if (prob > cumdistr[3] && prob < cumdistr[4])
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;
        }
        else
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;
        }

    } // end loop on nodes

    return sfQuantity;

} //  end function
} // namespace lorawan
} // namespace ns3

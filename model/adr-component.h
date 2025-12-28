/*
 * Copyright (c) 2017 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Matteo Perin <matteo.perin.2@studenti.unipd.2>
 */

#ifndef ADR_COMPONENT_H
#define ADR_COMPONENT_H

#include "network-controller-components.h"
#include "network-status.h"

#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/packet.h"

#include <algorithm>
#include <vector>

namespace ns3
{
namespace lorawan
{

/**
 * \ingroup lorawan
 *
 * LinkAdrRequest commands management
 */
class AdrComponent : public NetworkControllerComponent
{
  public:
    /**
     * Available policies for combining radio metrics in packet history.
     */
    enum CombiningMethod
    {
        AVERAGE,
        MAXIMUM,
        MINIMUM,
    };
    
    /**
     *  Register this type.
     *  \return The object TypeId.
     */
    static TypeId GetTypeId();

    AdrComponent();           //!< Default constructor
    ~AdrComponent() override; //!< Destructor

    void OnReceivedPacket(Ptr<const Packet> packet,
                          Ptr<EndDeviceStatus> status,
                          Ptr<NetworkStatus> networkStatus) override;

    void BeforeSendingReply(Ptr<EndDeviceStatus> status, Ptr<NetworkStatus> networkStatus) override;

    void OnFailedReply(Ptr<EndDeviceStatus> status, Ptr<NetworkStatus> networkStatus) override;

  protected:
    /**
     * Implementation of the default Adaptive Data Rate (ADR) procedure.
     *
     * ADR is meant to optimize radio modulation parameters of end devices to improve energy
     * consuption and radio resource utilization. For more details see
     * https://doi.org/10.1109/NOMS.2018.8406255 .
     *
     * \param newDataRate [out] new data rate value selected for the end device.
     * \param newTxPower [out] new tx power value selected for the end device.
     * \param status State representation of the current end device.
     */
    virtual void AdrImplementation(uint8_t* newDataRate, uint8_t* newTxPower, Ptr<EndDeviceStatus> status);

    /**
     * Convert spreading factor values [7:12] to respective data rate values [0:5].
     *
     * \param sf The spreading factor value.
     * \return Value of the data rate as uint8_t.
     */
    uint8_t SfToDr(uint8_t sf);

    /**
     * Convert reception power values [dBm] to Signal to Noise Ratio (SNR) values [dB].
     *
     * The conversion comes from the formula \f$P_{rx}=-174+10\log_{10}(B)+SNR+NF\f$ where
     * \f$P_{rx}\f$ is the received transmission power, \f$B\f$ is the transmission bandwidth and
     * \f$NF\f$ is the noise figure of the receiver. The constant \f$-174\f$ is the thermal noise
     * [dBm] in 1 Hz of bandwidth and is influenced the temperature of the receiver, assumed
     * constant in this model. For more details see the SX1301 chip datasheet.
     *
     * \param transmissionPower Value of received transmission power.
     * \return SNR value as double.
     */
    double RxPowerToSNR(double transmissionPower) const;

    /**
     * Get the min RSSI (dBm) among gateways receiving the same transmission.
     *
     * \param gwList List of gateways paired with reception information.
     * \return Min RSSI of transmission as double.
     */
    double GetMinTxFromGateways(EndDeviceStatus::GatewayList gwList);
    /**
     * Get the max RSSI (dBm) among gateways receiving the same transmission.
     *
     * \param gwList List of gateways paired with packet reception information.
     * \return Max RSSI of transmission as double.
     */
    double GetMaxTxFromGateways(EndDeviceStatus::GatewayList gwList);
    /**
     * Get the average RSSI (dBm) of gateways receiving the same transmission.
     *
     * \param gwList List of gateways paired with packet reception information.
     * \return Average RSSI of transmission as double.
     */
    double GetAverageTxFromGateways(EndDeviceStatus::GatewayList gwList);
    /**
     * Get RSSI metric for a transmission according to chosen gateway aggregation policy.
     *
     * \param gwList List of gateways paired with packet reception information.
     * \return RSSI of tranmsmission as double.
     */
    double GetReceivedPower(EndDeviceStatus::GatewayList gwList);

    /**
     * Get the min Signal to Noise Ratio (SNR) of the receive packet history.
     *
     * \param packetList History of received packets with reception information.
     * \param historyRange Number of packets to consider going back in time.
     * \return Min SNR among packets as double.
     */
    double GetMinSNR(EndDeviceStatus::ReceivedPacketList packetList, int historyRange);
    /**
     * Get the max Signal to Noise Ratio (SNR) of the receive packet history.
     *
     * \param packetList History of received packets with reception information.
     * \param historyRange Number of packets to consider going back in time.
     * \return Max SNR among packets as double.
     */
    double GetMaxSNR(EndDeviceStatus::ReceivedPacketList packetList, int historyRange);
    /**
     * Get the average Signal to Noise Ratio (SNR) of the received packet history.
     *
     * \param packetList History of received packets with reception information.
     * \param historyRange Number of packets to consider going back in time.
     * \return Average SNR of packets as double.
     */
    double GetAverageSNR(EndDeviceStatus::ReceivedPacketList packetList, int historyRange);

    /**
     * Get the LoRaWAN protocol TXPower configuration index from the Equivalent Isotropically
     * Radiated Power (EIRP) in dBm.
     *
     * \param txPower Transission EIRP configuration.
     * \return TXPower index as int.
     */
    int GetTxPowerIndex(int txPower);

    enum CombiningMethod tpAveraging;      //!< TX power from gateways policy
    int historyRange;                      //!< Number of previous packets to consider
    enum CombiningMethod historyAveraging; //!< Received SNR history policy

    const int min_spreadingFactor = 7;    //!< Spreading factor lower limit
    const int min_transmissionPower = 2;  //!< Minimum transmission power (dBm) (Europe)
    const int max_transmissionPower = 14; //!< Maximum transmission power (dBm) (Europe)
    // const int offset = 10;                //!< Device specific SNR margin (dB)
    const int B = 125000; //!< Bandwidth (Hz)
    const int NF = 6;     //!< Noise Figure (dB)
    double threshold[6] = {
        -20.0,
        -17.5,
        -15.0,
        -12.5,
        -10.0,
        -7.5}; //!< Vector containing the required SNR for the 6 allowed spreading factor
               //!< levels ranging from 7 to 12 (the SNR values are in dB).

    bool m_toggleTxPower; //!< Whether to control transmission power of end devices or not

    double m_margin;
};

class CAADR : public AdrComponent
{
public:
  static TypeId GetTypeId();

  CAADR();           //!< Default constructor
  ~CAADR();          //!< Destructor

  void SetToas(std::vector<double> toas);

protected:
  void AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status) override;

  double Log(double base, double value);

  int NumMaxOfNodesPerSF(double toa, double succProb, int nFreq = 1);

  double GetAveragePr(EndDeviceStatus::ReceivedPacketList packetList, int historyRange);

  uint8_t SelectSF(double power);
                         
  double m_interval;
  double m_prob;
  
  std::vector<double> m_toas;
  std::vector<int> m_nMax;
  std::vector<int> m_n;
  // std::vector<double> m_lastProb;

  std::string m_sToas;
  int m_nRun;

  std::map<LoraDeviceAddress, uint8_t> m_SfPerEd;
};

class GADR : public AdrComponent
{
public:
  /**
   *  Register this type.
   *  \return The object TypeId.
   */
  static TypeId GetTypeId();

  GADR();           //!< Default constructor
  ~GADR();          //!< Destructor

protected:
  double CalcStd(double m_SNR,
                 EndDeviceStatus::ReceivedPacketList packetList, 
                 int historyRange);

  void AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status) override;
};

struct GatewayInfo
{
  Address m_gwAddr;
  std::vector<int> m_numEdsPerSf;

  GatewayInfo(Address gwAddr):
    m_gwAddr(gwAddr)
  {
    m_numEdsPerSf = {0, 0, 0, 0, 0, 0};
  }

  GatewayInfo()
  {
  }

  ~GatewayInfo()
  {
    m_numEdsPerSf.clear();
  }

  void Clear()
  {
    m_numEdsPerSf.clear();
  }
};

class RADR : public CAADR
{
public:
  static TypeId GetTypeId();

  RADR();           //!< Default constructor
  ~RADR();          //!< Destructor

protected:
  EndDeviceStatus::PacketInfoPerGw GetMaxSnrAndBestGw(EndDeviceStatus::GatewayList gwList);                         

  void AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status) override;

  std::map<Address, GatewayInfo> m_gwInfos;
};

class MBADR : public AdrComponent
{
public:
  /**
   *  Register this type.
   *  \return The object TypeId.
   */
  static TypeId GetTypeId();

  MBADR();           //!< Default constructor
  ~MBADR();          //!< Destructor

protected:  
  // Get SNR list
  std::vector<double> GetSnrList(EndDeviceStatus::ReceivedPacketList packetList, int historyRange);

  // Calculates 1st quartile (25th percentile)
  double CalcFirstQuartile(std::vector<double> v);

  // Calculates 3rd quartile (75th percentile)
  double CalcThirdQuartile(std::vector<double> v);

  std::vector<double> RemOutliers(std::vector<double> v);

  // Calculates the median (50th percentile)
  double CalcMedian(std::vector<double> v);

  void AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status) override;
};

class KADR : public AdrComponent
{
public:
  static TypeId GetTypeId();

  KADR();           //!< Default constructor
  ~KADR();          //!< Destructor

protected:
  // Get SNR List
  std::vector<double> GetSnrList(EndDeviceStatus::ReceivedPacketList packetList, int historyRange);

  // Gaussian variogram function
  double GaussianVariogram(int h, double alpha);

  // Ordinary Kriging interpolation function
  double PerformKrigingInterpolation(const std::vector<double>& snrList);

  void AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status) override;

  double m_alpha;                        
};

/* SSFIR ADR implementation class.
 * Based on the paper: Collision Avoidance Adaptive 
 * Data Rate Algorithm for LoRaWAN.
 * Published in MDPI Future Internet, 2024.
 * Link: https://doi.org/10.3390/fi16100380
 */
class SSFIR : public AdrComponent
{
public:
  static TypeId GetTypeId();

  SSFIR();           //!< Default constructor
  ~SSFIR();          //!< Destructor

protected:
  void AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status) override;       
};

class SSFIR2 : public AdrComponent
{
public:
  static TypeId GetTypeId();

  SSFIR2();           //!< Default constructor
  ~SSFIR2();          //!< Destructor

protected:
  void AdrImplementation(uint8_t* newDataRate,
                         uint8_t* newTxPower,
                         Ptr<EndDeviceStatus> status) override;       
  
  double m_rhoSF;
};

} // namespace lorawan
} // namespace ns3

#endif

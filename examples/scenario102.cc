/*
 * This program creates a simple network which uses an ADR algorithm to set up
 * the Spreading Factors of the devices in the Network.
 */

#include "ns3/point-to-point-module.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/lora-channel.h"
#include "ns3/mobility-helper.h"
#include "ns3/lora-phy-helper.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/lora-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/periodic-sender.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/lora-device-address-generator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/config.h"
#include "ns3/rectangle.h"
#include "ns3/hex-grid-position-allocator.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/file-helper.h"
#include "ns3/names.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/adr-fuzzy.h"
#include "ns3/adr-component.h"
#include <vector>

#include <iostream>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("AdrExample");

// Trace sources that are called when a node changes its DR or TX power
void OnDataRateChange (uint8_t oldDr, uint8_t newDr)
{
  NS_LOG_DEBUG ("DR" << unsigned(oldDr) << " -> DR" << unsigned(newDr));
}
void OnTxPowerChange (double oldTxPower, double newTxPower)
{
  NS_LOG_DEBUG (oldTxPower << " dBm -> " << newTxPower << " dBm");
}

int CalcSfToDr (int sf)
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

Ptr<ListPositionAllocator> randomPositions ()
{
  Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator> ();
  pos->Add(Vector(437.08324291459655,203.26283031253786,0));
  pos->Add(Vector(117.88699807713125,134.53643311650342,0));
  pos->Add(Vector(243.40990228664202,373.6846629768591,0));
  pos->Add(Vector(19.86867667033789,371.6416786759741,0));
  pos->Add(Vector(536.2526071370105,413.43647621380904,0));
  pos->Add(Vector(302.1257827811296,105.26038449643403,0));
  pos->Add(Vector(293.4579873177428,306.0901045219799,0));
  pos->Add(Vector(202.7875877545438,520.3664700925509,0));
  pos->Add(Vector(307.44769090939315,590.2627071940459,0));
  pos->Add(Vector(271.936405166673,445.2339782289079,0));
  pos->Add(Vector(453.12738522115745,317.5582557703469,0));
  pos->Add(Vector(25.62985898144854,433.1891177349234,0));
  pos->Add(Vector(449.3815121569145,330.2546659653834,0));
  pos->Add(Vector(589.5613561108006,136.98671911433536,0));
  pos->Add(Vector(470.9539075066428,558.7159981935129,0));
  pos->Add(Vector(501.60635002693846,588.5310056149974,0));
  pos->Add(Vector(539.4048489066115,356.4738038762621,0));
  pos->Add(Vector(299.7476097165442,499.8174674479202,0));
  pos->Add(Vector(408.028867545056,7.389706243761918,0));
  pos->Add(Vector(171.7422697778155,565.1194412405368,0));
  pos->Add(Vector(400.8978454435047,163.3352876874,0));
  pos->Add(Vector(302.72302671627307,253.17024446469685,0));
  pos->Add(Vector(159.0227811350954,109.3598743944276,0));
  pos->Add(Vector(93.73011193429596,93.8941130280763,0));
  pos->Add(Vector(269.31245296642516,245.69010214317217,0));
  pos->Add(Vector(128.12808425068388,303.31964818627324,0));
  pos->Add(Vector(7.446727666329944,485.9160684222071,0));
  pos->Add(Vector(282.7761147241817,387.9041626820154,0));
  pos->Add(Vector(338.0137683521931,369.1643399395951,0));
  pos->Add(Vector(322.6422310500615,515.2620175027089,0));
  pos->Add(Vector(528.1625632772493,249.46862587418403,0));
  pos->Add(Vector(413.54089750848317,356.45517668946775,0));
  pos->Add(Vector(453.61997925317826,48.9472156193774,0));
  pos->Add(Vector(501.89524272209354,229.32194343940034,0));
  pos->Add(Vector(447.7845748113837,524.9589376063933,0));
  pos->Add(Vector(162.09457070099023,312.1458311206635,0));
  pos->Add(Vector(99.38481706414295,218.98448198194038,0));
  pos->Add(Vector(293.7307021582113,186.45688866122788,0));
  pos->Add(Vector(589.4434493317196,351.7778786144445,0));
  pos->Add(Vector(498.25948137666495,249.91153784780943,0));
  pos->Add(Vector(482.7268854947305,99.59693317627605,0));
  pos->Add(Vector(162.19332301896958,571.5538185436361,0));
  pos->Add(Vector(21.36120500314145,529.8574116736811,0));
  pos->Add(Vector(91.07930596916762,264.82877175977916,0));
  pos->Add(Vector(430.96888524016225,141.0422403547809,0));
  pos->Add(Vector(176.50295095962275,319.74017002634304,0));
  pos->Add(Vector(528.5526629801202,158.24804721297326,0));
  pos->Add(Vector(150.90428506786358,49.172917114307154,0));
  pos->Add(Vector(499.82907290079686,492.8108328416159,0));
  pos->Add(Vector(239.36871923133663,124.3238200994849,0));
  pos->Add(Vector(445.49378024194095,343.42848672038883,0));
  pos->Add(Vector(543.543733525969,91.80421013822952,0));
  pos->Add(Vector(78.6346254036054,170.5176443529252,0));
  pos->Add(Vector(5.2857929088145905,235.42174186066163,0));
  pos->Add(Vector(351.860749004639,395.48162775776393,0));
  pos->Add(Vector(0.031024778519239682,358.3301907471232,0));
  pos->Add(Vector(588.8205879537229,404.52835106094074,0));
  pos->Add(Vector(485.7540514715455,95.84813002895882,0));
  pos->Add(Vector(533.2824435699868,331.6909161587488,0));
  pos->Add(Vector(77.25724062889827,353.9542616309185,0));
  pos->Add(Vector(455.85532637490047,84.97444107708891,0));
  pos->Add(Vector(75.92555426911547,520.5045996271722,0));
  pos->Add(Vector(13.610710785749891,599.7786950310646,0));
  pos->Add(Vector(510.42563186862134,279.78205696586525,0));
  pos->Add(Vector(164.78953346842573,24.721927143256718,0));
  pos->Add(Vector(581.7455258693224,473.5625188586293,0));
  pos->Add(Vector(261.02036504841635,32.9641555126736,0));
  pos->Add(Vector(200.4974514350315,384.58443607378825,0));
  pos->Add(Vector(184.98452108979887,105.41690662685915,0));
  pos->Add(Vector(584.7632394780817,593.1391862662293,0));
  pos->Add(Vector(195.40456928866828,73.09495932058721,0));
  pos->Add(Vector(569.2879091119244,571.5638585544249,0));
  pos->Add(Vector(401.4669246144557,538.1714362815846,0));
  pos->Add(Vector(235.47803925511872,525.1743320682697,0));
  pos->Add(Vector(406.6312985258454,447.73085453478876,0));
  pos->Add(Vector(259.7409840225299,425.15563602897794,0));
  pos->Add(Vector(488.3665127170699,6.1141675601458445,0));
  pos->Add(Vector(338.78317007170074,21.30418381102974,0));
  pos->Add(Vector(242.5544828536299,73.74717664200494,0));
  pos->Add(Vector(539.3611624305578,409.39229875563274,0));
  pos->Add(Vector(127.10552127929968,273.81786908982883,0));
  pos->Add(Vector(309.82523472682186,148.33596107278032,0));
  pos->Add(Vector(479.16961064769583,369.44595016461915,0));
  pos->Add(Vector(357.2514697396946,84.91499412519836,0));
  pos->Add(Vector(595.6425345318024,333.271756617697,0));
  pos->Add(Vector(229.335739061023,263.4902688895889,0));
  pos->Add(Vector(360.36120716778515,480.32270026439693,0));
  pos->Add(Vector(581.0839590187129,175.5482257434118,0));
  pos->Add(Vector(243.0853615542857,208.29788897800543,0));
  pos->Add(Vector(372.38769521188493,547.974684436542,0));
  pos->Add(Vector(387.5187325399249,217.55336453671885,0));
  pos->Add(Vector(117.89325978378477,308.98476877250516,0));
  pos->Add(Vector(599.2540432862507,51.165811235621206,0));
  pos->Add(Vector(15.760852224428223,392.1937487493254,0));
  pos->Add(Vector(209.21081708793307,567.1988673198431,0));
  pos->Add(Vector(43.6944910553003,462.3208600640037,0));
  pos->Add(Vector(357.663245941626,335.5991479552107,0));
  pos->Add(Vector(332.6657053004174,476.599131397524,0));
  pos->Add(Vector(329.82263925324446,252.18352922573285,0));
  pos->Add(Vector(268.21280175225183,122.75351056324403,0));
  pos->Add(Vector(91.0423247980042,505.9058131476474,0));
  pos->Add(Vector(598.0410694796097,300.34511624366297,0));
  pos->Add(Vector(210.99956988115366,244.99906507133412,0));
  pos->Add(Vector(28.564496712285713,540.9981758847289,0));
  pos->Add(Vector(492.56446076899795,29.672624747129372,0));
  pos->Add(Vector(79.26413105130091,96.15095396084352,0));
  pos->Add(Vector(354.3539720797086,426.7295845242458,0));
  pos->Add(Vector(506.71127699413984,221.05176003308551,0));
  pos->Add(Vector(233.49938633073427,221.45486019124712,0));
  pos->Add(Vector(248.95709101143984,145.91063244644727,0));
  pos->Add(Vector(271.0300208532199,521.5604547496254,0));
  pos->Add(Vector(386.2329706240921,587.4718408092409,0));
  pos->Add(Vector(528.0255051537885,242.35772920006414,0));
  pos->Add(Vector(432.30566948842227,439.8743990580534,0));
  pos->Add(Vector(324.8980475860229,418.8032992894233,0));
  pos->Add(Vector(257.3728397473213,416.84178782728776,0));
  pos->Add(Vector(115.25464448542648,338.59681276516005,0));
  pos->Add(Vector(488.81443484497566,263.4472809841664,0));
  pos->Add(Vector(39.98588961210194,102.64220889325972,0));
  pos->Add(Vector(468.84439214275557,379.7963853626735,0));
  pos->Add(Vector(327.1786120613192,12.149282496458147,0));
  pos->Add(Vector(302.7617888831692,144.4045201228872,0));
  pos->Add(Vector(60.51866191076005,457.8727944734795,0));
  pos->Add(Vector(522.3765697669514,384.2977669823193,0));
  pos->Add(Vector(349.52221598036607,382.4214028227831,0));
  pos->Add(Vector(37.14675142431911,234.21040659797373,0));
  pos->Add(Vector(270.55900666623825,37.2916960308338,0));
  pos->Add(Vector(61.8286237037782,182.07066404768165,0));
  pos->Add(Vector(154.39378387698432,95.28699906815001,0));
  pos->Add(Vector(348.75728820522335,416.501259427013,0));
  pos->Add(Vector(294.3221295724376,174.7551443370261,0));
  pos->Add(Vector(70.57636111908843,74.74609280542728,0));
  pos->Add(Vector(387.0115930271485,232.43716344250996,0));
  pos->Add(Vector(61.77728226244186,450.67392392704363,0));
  pos->Add(Vector(370.12893262594054,330.38329400123496,0));
  pos->Add(Vector(412.26509062199096,410.5357229215279,0));

  return pos;
}

int main (int argc, char *argv[])
{
  bool verbose = false;
  bool adrEnabled = true;
  bool initializeSF = true;
  int nDevices = 136;
  double sideLength = 600;
  std::string adrType = "ns3::AdrComponent";
  int intervalTx = 15;
  int nRun = 1;

  CommandLine cmd;

  cmd.AddValue ("verbose", "Whether to print output or not", verbose);
  cmd.AddValue ("adrType", "ADR Class [ns3::AdrComponent, ns3::AdrFuzzy]", adrType);
  
  if (adrType == "ns3::AdrComponent")
    {
      cmd.AddValue ("MultipleGwCombiningMethod",
                    "ns3::AdrComponent::MultipleGwCombiningMethod");
      cmd.AddValue ("MultiplePacketsCombiningMethod",
                    "ns3::AdrComponent::MultiplePacketsCombiningMethod");
      cmd.AddValue ("ChangeTransmissionPower",
                    "ns3::AdrComponent::ChangeTransmissionPower");
      cmd.AddValue ("HistoryRange", "ns3::AdrComponent::HistoryRange");
    }
  else if (adrType == "ns3::AdrFuzzy")
    {
      cmd.AddValue ("MultipleGwCombiningMethod",
                "ns3::AdrFuzzy::MultipleGwCombiningMethod");
      cmd.AddValue ("MultiplePacketsCombiningMethod",
                    "ns3::AdrFuzzy::MultiplePacketsCombiningMethod");
      cmd.AddValue ("ChangeTransmissionPower",
                    "ns3::AdrFuzzy::ChangeTransmissionPower");
      cmd.AddValue ("HistoryRange", "ns3::AdrComponent::HistoryRange");
    }
  
  cmd.AddValue ("MType", "ns3::EndDeviceLorawanMac::MType");
  cmd.AddValue ("EDDRAdaptation", "ns3::EndDeviceLorawanMac::EnableEDDataRateAdaptation");
  cmd.AddValue ("AdrEnabled", "Whether to enable ADR", adrEnabled);
  cmd.AddValue ("nDevices", "Number of devices to simulate", nDevices);
  cmd.AddValue ("sideLength",
                "Length of the side of the rectangle nodes will be placed in",
                sideLength);
  cmd.AddValue ("initializeSF",
                "Whether to initialize the SFs",
                initializeSF);
  cmd.AddValue ("MaxTransmissions",
                "ns3::EndDeviceLorawanMac::MaxTransmissions");
  cmd.AddValue ("intervalTx",
                "Time in seconds to ED transmit new message",
                intervalTx); // 15, 30 or 60 minutes
  cmd.AddValue ("nRun", "Number of Run for SetRun", nRun);

  cmd.Parse (argc, argv);

  //RngSeedManager::SetSeed (30);
  RngSeedManager::SetRun (nRun);

  // Set the EDs to require Data Rate control from the NS
  Config::SetDefault ("ns3::EndDeviceLorawanMac::DRControl", BooleanValue (true));

  // Create a simple wireless channel
  ///////////////////////////////////

  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.57);
  loss->SetReference (40, 127.41); // ToDo Thiago
  loss->SetAttribute("Exponent", DoubleValue (2.0));

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  // Create the correlated shadowing component
  Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
      CreateObject<CorrelatedShadowingPropagationLossModel> ();
  // Aggregate shadowing to the logdistance loss
  loss->SetNext (shadowing);

  // Add the effect to the channel propagation loss
  Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();
  shadowing->SetNext (buildingLoss);

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  // Helpers
  ////////// 
  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking ();

  ////////////////
  // Create GWs //
  ////////////////
  NodeContainer gateways;
  gateways.Create (1);

  MobilityHelper mobilityGw;  
  Ptr<ListPositionAllocator> positionAllocGw = CreateObject<ListPositionAllocator> ();
  positionAllocGw->Add (Vector (sideLength / 2, sideLength / 2, 0));
  mobilityGw.SetPositionAllocator (positionAllocGw);
  mobilityGw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityGw.Install (gateways);

  // Create the LoraNetDevices of the gateways
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  // Create EDs
  /////////////
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // Install mobility model on fixed nodes
  MobilityHelper mobilityEd;
  /*mobilityEd.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                  "X", PointerValue (CreateObjectWithAttributes<UniformRandomVariable>
                                                      ("Min", DoubleValue(0),
                                                      "Max", DoubleValue(sideLength))),
                                  "Y", PointerValue (CreateObjectWithAttributes<UniformRandomVariable>
                                                      ("Min", DoubleValue(0),
                                                      "Max", DoubleValue(sideLength))));*/
  Ptr<ListPositionAllocator> posAllocEds = randomPositions ();
  mobilityEd.SetPositionAllocator (posAllocEds);
  mobilityEd.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityEd.Install (endDevices);

  // Create a LoraDeviceAddressGenerator
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject<LoraDeviceAddressGenerator> (nwkId,nwkAddr);

  // Create the LoraNetDevices of the end devices
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  macHelper.SetAddressGenerator (addrGen);
  macHelper.SetRegion (LorawanMacHelper::EU);
  NetDeviceContainer endDevicesNetDevices = 
    helper.Install (phyHelper, macHelper, endDevices);

  // Install applications in EDs
  int appPeriodSeconds = intervalTx * 60;      // One packet every 15, 30 and 60 minutes
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriodSeconds));
  ApplicationContainer appContainer = appHelper.Install (endDevices);

  // Do not set spreading factors up: we will wait for the NS to do this
  if (initializeSF)
    {
      std::vector<int> sfList = {12, 12,  7, 12,  8, 11, 11, 11,  8, 12, 11,  7,  8,  8, 10, 11,  9, 10,
  9,  8, 12,  9, 11,  8,  9,  9, 11,  9,  7, 11,  7, 10, 12, 11,  9,  9,
 12, 11,  9,  9, 10,  8,  8, 12, 12,  7, 10, 10, 10, 10,  8,  7,  8,  8,
 11, 12, 11, 10, 12, 11,  9,  8,  7, 10,  7,  9, 11, 10, 10,  9, 11, 12,
 10, 11,  8, 11, 12, 11,  8, 12, 10, 11, 10,  7, 10,  7,  9, 12,  7, 12,
 12,  7, 10, 12, 10, 10,  7,  7,  9,  9, 12, 12, 10,  9, 12, 11, 11, 10,
 12, 10,  8, 12,  9, 11,  7,  8,  9, 10,  9, 11, 12,  9,  9, 10,  8, 11,
 12,  9,  9, 12, 12,  7, 10,  8, 10, 10};
      // macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
      // Ptr<UniformRandomVariable> sfRandom = CreateObject<UniformRandomVariable>();
      for (uint32_t i = 0; i < endDevices.GetN (); i++)
        {
          Ptr<Node> node = endDevices.Get (i);
          int sfNode = sfList[i];
          Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
          Ptr<ClassAEndDeviceLorawanMac> mac =
            loraNetDevice->GetMac ()->GetObject<ClassAEndDeviceLorawanMac> ();
          mac->SetDataRate (CalcSfToDr (sfNode));
        }
    }
  
  /************************
   * Install Energy Model *
   ************************/
  BasicEnergySourceHelper basicSourceHelper;
  LoraRadioEnergyModelHelper radioEnergyHelper;

  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (10000)); // Energy in J
  basicSourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (3.3));

  radioEnergyHelper.Set ("StandbyCurrentA", DoubleValue (0.0014));
  radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.028));
  radioEnergyHelper.Set ("SleepCurrentA", DoubleValue (0.0000015));
  radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.0112));

  radioEnergyHelper.SetTxCurrentModel ("ns3::LinearLoraTxCurrentModel");

  // install source on EDs' nodes
  EnergySourceContainer sources = basicSourceHelper.Install (endDevices);
  Names::Add ("/Names/EnergySource", sources.Get (0));

  // install device model
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install
      (endDevicesNetDevices, sources);

  /**************
   * Get output *
   **************/
  FileHelper fileHelper;
  
  std::string batteryFile;
  if (adrType == "ns3::AdrComponent")
    {
      batteryFile = "battery-level-component-";
    }
  else if (adrType == "ns3::AdrFuzzy")
    {
      batteryFile = "battery-level-fuzzy-";
    }
  batteryFile += std::to_string (nRun);

  fileHelper.ConfigureFile (batteryFile, FileAggregator::COMMA_SEPARATED);
  fileHelper.WriteProbe ("ns3::DoubleProbe", "/Names/EnergySource/RemainingEnergy", "Output");

  ////////////
  // Create NS
  ////////////

  NodeContainer networkServers;
  networkServers.Create (1);

  // Install the NetworkServer application on the network server
  NetworkServerHelper networkServerHelper;
  networkServerHelper.SetGateways (gateways);
  networkServerHelper.SetEndDevices (endDevices);
  networkServerHelper.EnableAdr (adrEnabled);
  networkServerHelper.SetAdr (adrType);
  networkServerHelper.SetRun (nRun);
  networkServerHelper.Install (networkServers);
  
  // Install the Forwarder application on the gateways
  ForwarderHelper forwarderHelper;
  forwarderHelper.Install (gateways);

  /**********************
   *  Handle buildings  *
   **********************/
  double radius = 100;
  double xLength = 20;
  double deltaX = 32;
  double yLength = 40;
  double deltaY = 17;
  int gridWidth = 2 * radius / (xLength + deltaX);
  int gridHeight = 2 * radius / (yLength + deltaY);

  Ptr<GridBuildingAllocator> gridBuildingAllocator;
  gridBuildingAllocator = CreateObject<GridBuildingAllocator> ();
  gridBuildingAllocator->SetAttribute ("GridWidth", UintegerValue (gridWidth));
  gridBuildingAllocator->SetAttribute ("LengthX", DoubleValue (xLength));
  gridBuildingAllocator->SetAttribute ("LengthY", DoubleValue (yLength));
  gridBuildingAllocator->SetAttribute ("DeltaX", DoubleValue (deltaX));
  gridBuildingAllocator->SetAttribute ("DeltaY", DoubleValue (deltaY));
  gridBuildingAllocator->SetAttribute ("Height", DoubleValue (6));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsX", UintegerValue (2));
  gridBuildingAllocator->SetBuildingAttribute ("NRoomsY", UintegerValue (4));
  gridBuildingAllocator->SetBuildingAttribute ("NFloors", UintegerValue (2));
  gridBuildingAllocator->SetAttribute (
      "MinX", DoubleValue (-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
  gridBuildingAllocator->SetAttribute (
      "MinY", DoubleValue (-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
  BuildingContainer bContainer = gridBuildingAllocator->Create (gridWidth * gridHeight);

  // std::cout << "Qtd de Buildings = " << (bContainer.GetN ()) << std::endl;

  BuildingsHelper::Install (endDevices);
  BuildingsHelper::Install (gateways);

  // Connect our traces
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/TxPower",
                                 MakeCallback (&OnTxPowerChange));
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/DataRate",
                                 MakeCallback (&OnDataRateChange));

  // Activate printing of ED MAC parameters
  std::string nodeFile, phyFile, globalFile;
  if (adrType == "ns3::AdrComponent")
    {
      nodeFile = "nodeData102ADR-";
      phyFile = "phyPerformance102ADR-";
      globalFile = "globalPerformance102ADR-";
    }
  else if (adrType == "ns3::AdrFuzzy")
    {
      nodeFile = "nodeData102FADR-";
      phyFile = "phyPerformance102FADR-";
      globalFile = "globalPerformance102FADR-";
    }
  nodeFile += std::to_string (nRun) + ".csv";
  phyFile += std::to_string (nRun) + ".csv";
  globalFile += std::to_string (nRun) + ".csv";

  Time stateSamplePeriod = Seconds (intervalTx * 60);
  helper.EnablePeriodicDeviceStatusPrinting (endDevices, gateways, nodeFile, stateSamplePeriod);
  helper.EnablePeriodicPhyPerformancePrinting (gateways, phyFile, stateSamplePeriod);
  helper.EnablePeriodicGlobalPerformancePrinting (globalFile, stateSamplePeriod);

  LoraPacketTracker& tracker = helper.GetPacketTracker ();

  // Start simulation
  // Time simulationTime = Seconds (12 * 24 * 60 * 60);
  Time simulationTime = Seconds (2 * 24 * 60 * 60);
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();

  std::cout << tracker.CountMacPacketsGlobally(Seconds (intervalTx * 60 * (intervalTx - 2)),
                                               Seconds (intervalTx * 60 * (intervalTx - 1))) << std::endl;

  return 0;
}

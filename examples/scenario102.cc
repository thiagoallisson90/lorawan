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

int main (int argc, char *argv[])
{
  bool verbose = false;
  bool adrEnabled = true;
  bool initializeSF = true;
  int nDevices = 102;
  double sideLength = 500;
  std::string adrType = "ns3::AdrComponent";
  int nPeriods = 20;
  int intervalTx = 30;
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

  RngSeedManager::SetSeed (time (NULL));
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
  // End Device mobility
  MobilityHelper mobilityEd, mobilityGw;
  mobilityEd.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                  "X", PointerValue (CreateObjectWithAttributes<UniformRandomVariable>
                                                      ("Min", DoubleValue(0),
                                                      "Max", DoubleValue(sideLength))),
                                  "Y", PointerValue (CreateObjectWithAttributes<UniformRandomVariable>
                                                      ("Min", DoubleValue(0),
                                                      "Max", DoubleValue(sideLength))));

  Ptr<ListPositionAllocator> positionAllocGw = CreateObject<ListPositionAllocator> ();
  positionAllocGw->Add (Vector (sideLength / 2, sideLength / 2, 0));
  mobilityGw.SetPositionAllocator (positionAllocGw);
  mobilityGw.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

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
  mobilityEd.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  for (int i = 0; i < nDevices; ++i)
  {
    mobilityEd.Install (endDevices.Get (i));
  }

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
      // macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
      Ptr<UniformRandomVariable> sfRandom = CreateObject<UniformRandomVariable>();
      for (uint32_t i = 0; i < endDevices.GetN (); i++)
      {
        Ptr<Node> node = endDevices.Get (i);
        int sfNode = sfRandom->GetInteger (7, 12);
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

  std::string animFile = "anim" + std::to_string (nRun) + ".xml";

  // Start simulation
  Time simulationTime = Seconds (12 * 24 * 60 * 60);
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();

  std::cout << tracker.CountMacPacketsGlobally(Seconds (intervalTx * 60 * (nPeriods - 2)),
                                               Seconds (intervalTx * 60 * (nPeriods - 1))) << std::endl;

  return 0;
}

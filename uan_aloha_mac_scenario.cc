#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/netanim-module.h"
#include <vector>
#include "ns3/uan-routing-vbf.h"
#include "ns3/flow-monitor-module.h"
#include <cassert>
#include "ns3/gnuplot.h"
#include "ns3/mac-gplot.h"
#include <ns3/wsn-module.h>
#include "ns3/ipv6-static-routing-helper.h"
#include "ns3/ipv6-routing-table-entry.h"
#include "ns3/iot-module.h"
#include <iostream>
#include <fstream>
#include <string>
#include<iostream>
#include<math.h>
#include<string.h>
#include<stdlib.h>
#include "ns3/omni-antenna-model.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/stats-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include "ns3/uan-module.h"
#include <fstream>
using namespace ns3;
using namespace std;
NS_LOG_COMPONENT_DEFINE ("uan_aloha_mac_scenario");
AnimationInterface *pAnim;
double ds=1000.0;  
int rounds=300;      
uint32_t packetSize = 512; 
uint32_t noofpkts = 100;       
int p, q, n, t, flag, e[100], d[100], temp[100], j, m[100], en[100], i;
double interval = 1.0; 
Time interPacketInterval = Seconds (interval);
void compare_Minimum(double dis){
if(ds>dis){ds=dis;}}
void getNearbynodesrc(NodeContainer wsn){
int nn=1;
double x1=250;
double y1=250;
for(uint32_t i=0;i<wsn.GetN ();i++){
Ptr<RandomWaypointMobilityModel> FCMob = wsn.Get(i)->GetObject<RandomWaypointMobilityModel>();
Vector m_position = FCMob->GetPosition();
double x=m_position.x;
double y=m_position.y;
double xx=x1-x;
double yy=y1-y;
double x2=(xx*xx);
double y2=(yy*yy);
double sx=sqrt(x2);
double sy=sqrt(y2);
double dis=(sx+sy);
compare_Minimum(dis);
if(ds<=100){
if(nn==1){
pAnim->UpdateNodeColor (wsn.Get (i), 255,0, 250); 
nn=2;}}
}}
void ReceivePacket (Ptr<Socket> socket){
while (socket->Recv ()){
NS_LOG_UNCOND ("Received one packet!");
}}
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,uint32_t pktCount, Time pktInterval ){
if (pktCount > 0){
socket->Send (Create<Packet> (pktSize));
Simulator::Schedule (pktInterval, &GenerateTraffic,socket, pktSize,pktCount-1, pktInterval);}
else{socket->Close ();}}
void PktTrans1(NodeContainer c, NodeContainer d){
std::cout<<"\n All sensor nodes are act as sender and server act as a destination. \n";
for(  uint32_t i=0;i<c.GetN ();i++){
TypeId tid1 = TypeId::LookupByName ("ns3::UdpSocketFactory");
Ptr<Socket> recvSink1 = Socket::CreateSocket (d.Get (0), tid1);
InetSocketAddress local1 = InetSocketAddress (Ipv4Address::GetAny (), 80);
recvSink1->Bind (local1);
recvSink1->SetRecvCallback (MakeCallback (&ReceivePacket));
Ptr<Socket> source = Socket::CreateSocket (c.Get (i), tid1);
InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
source->SetAllowBroadcast (true);
source->Connect (remote);
Simulator::ScheduleWithContext (source->GetNode ()->GetId (),Seconds (0.1), &GenerateTraffic,source, packetSize, noofpkts,interPacketInterval);}}
void PktTrans2(NodeContainer c, NodeContainer d){
std::cout<<"\n All sensor nodes sense the data to transfer. \n\n";
for(  uint32_t i=0;i<c.GetN ();i++){
TypeId tid1 = TypeId::LookupByName ("ns3::UdpSocketFactory");
Ptr<Socket> recvSink1 = Socket::CreateSocket (d.Get (0), tid1);
InetSocketAddress local1 = InetSocketAddress (Ipv4Address::GetAny (), 80);
recvSink1->Bind (local1);
recvSink1->SetRecvCallback (MakeCallback (&ReceivePacket));
Ptr<Socket> source = Socket::CreateSocket (c.Get (i), tid1);
InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
source->SetAllowBroadcast (true);
source->Connect (remote);
Simulator::ScheduleWithContext (source->GetNode ()->GetId (),Seconds (0.1), &GenerateTraffic,source, packetSize, noofpkts,interPacketInterval);}}
void PktTrans3(NodeContainer c, NodeContainer d){
std::cout<<"\n we make a communication between the sensor node and server node through the sink node based on the uan-aloha mac protocol process  \n\n";
for(  uint32_t i=0;i<c.GetN ();i++){
TypeId tid1 = TypeId::LookupByName ("ns3::UdpSocketFactory");
Ptr<Socket> recvSink1 = Socket::CreateSocket (d.Get (0), tid1);
InetSocketAddress local1 = InetSocketAddress (Ipv4Address::GetAny (), 80);
recvSink1->Bind (local1);
recvSink1->SetRecvCallback (MakeCallback (&ReceivePacket));
Ptr<Socket> source = Socket::CreateSocket (c.Get (i), tid1);
InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
source->SetAllowBroadcast (true);
source->Connect (remote);
Simulator::ScheduleWithContext (source->GetNode ()->GetId (),Seconds (0.1), &GenerateTraffic,source, packetSize, noofpkts,interPacketInterval);}}
int main (int argc, char *argv[]){
std::string phyMode ("DsssRate1Mbps");
double distance = 600;  
uint16_t numNodes = 100; 
int noOfNodes = 100;  
numNodes=(uint16_t)noOfNodes;
uint32_t revNode = 0;
uint32_t sourceNode = 1;
int nodeSpeed = 1; 
int nodePause = 0; 
bool enableFlowMonitor = false;
CommandLine cmd;
double simtime=100.0;
cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
cmd.AddValue ("distance", "distance (m)", distance);
cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
cmd.AddValue ("noofpkts", "number of packets generated", noofpkts);
cmd.AddValue ("interval", "interval (seconds) between packets", interval);
cmd.AddValue ("numNodes", "number of nodes", numNodes);
cmd.AddValue ("revNode", "Receiver node number", revNode);
cmd.AddValue ("sourceNode", "Sender node number", sourceNode);
cmd.AddValue ("EnableMonitor", "Enable Flow Monitor", enableFlowMonitor);
cmd.Parse (argc, argv); 
NodeContainer Sensor_Nodes;
NodeContainer Sink_Node;
NodeContainer Server_Node;
std::cout<<"\n Enter the No.Of.Nodes:-  [5-100]  \n";
cin>>noOfNodes;
std::cout<<"\n A Underwater Sensor network, its consists of "<<  noOfNodes <<"  - Sensor nodes,1 - Sink node and 1 - Server. \n";
Sensor_Nodes.Create (noOfNodes);
Server_Node.Create (1);
Sink_Node.Create (1);
WifiHelper wifi;
Ptr<Ipv6ExtensionESP > extension;
Ptr<Ipv6ExtensionAH> extenAH;
YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();  
wifiPhy.Set ("RxGain", DoubleValue (-30)); 
wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO); 
YansWifiChannelHelper wifiChannel;
wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
wifiPhy.SetChannel (wifiChannel.Create ());
NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode",StringValue (phyMode),"ControlMode",StringValue (phyMode)); 
wifiMac.SetType ("ns3::AdhocWifiMac");
NetDeviceContainer staticdevices = wifi.Install (wifiPhy, wifiMac, Sensor_Nodes);
NetDeviceContainer switchdevices = wifi.Install (wifiPhy, wifiMac, Sink_Node);
NetDeviceContainer controllerdevices = wifi.Install (wifiPhy, wifiMac, Sink_Node);
NetDeviceContainer apDevices;
apDevices = wifi.Install (wifiPhy, wifiMac, Sink_Node);
int64_t streamIndex = 0;
ObjectFactory pos;
pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=1000.0]"));
pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=75.0|Max=675.0]"));
Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
streamIndex += taPositionAlloc->AssignStreams (streamIndex);
MobilityHelper mobility;
mobility.SetPositionAllocator(taPositionAlloc);
std::stringstream ssSpeed;
ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
std::stringstream ssPause;
ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel","Speed", StringValue (ssSpeed.str ()),"Pause", StringValue (ssPause.str ()),
"PositionAllocator", PointerValue (taPositionAlloc));
mobility.Install (Sensor_Nodes);
MobilityHelper mobility1;
mobility1.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
mobility1.Install (Sink_Node);
mobility1.Install (Server_Node);
AnimationInterface::SetConstantPosition (Sink_Node.Get (0), 500, 50);
AnimationInterface::SetConstantPosition (Server_Node.Get (0), 500, 25);
OmniAntennaModel obj2;
obj2.SetBeamwidth (100);
uint32_t m_dataRate=80;
std::string m_gnudatfile="uan-aloha-example.gpl";           
std::string m_asciitracefile="uan-aloha-example.asc";       
std::string m_bhCfgFile="uan-apps/dat/default.cfg";  
std::string perModel = "ns3::UanPhyPerGenDefault";
std::string sinrModel = "ns3::UanPhyCalcSinrDefault";
ObjectFactory obf;
obf.SetTypeId (perModel);
Ptr<UanPhyPer> per = obf.Create<UanPhyPer> ();
obf.SetTypeId (sinrModel);
Ptr<UanPhyCalcSinr> sinr = obf.Create<UanPhyCalcSinr> ();
UanHelper uan;
UanTxMode mode;
mode = UanTxModeFactory::CreateMode (UanTxMode::FSK, m_dataRate,m_dataRate, 12000,m_dataRate, 2,"Default mode");
UanModesList myModes;
myModes.AppendMode (mode);
uan.SetPhy ("ns3::UanPhyGen","PerModel", PointerValue (per),"SinrModel", PointerValue (sinr),"SupportedModes", UanModesListValue (myModes));
uan.SetMac ("ns3::UanMacAloha");
#ifdef UAN_PROP_BH_INSTALLED
Ptr<UanPropModelBh> prop = CreateObjectWithAttributes<UanPropModelBh> ("ConfigFile", StringValue ("exbhconfig.cfg"));
#else 
Ptr<UanPropModelIdeal> prop = CreateObjectWithAttributes<UanPropModelIdeal> ();
#endif 
Ptr<UanChannel> channel = CreateObjectWithAttributes<UanChannel> ("PropagationModel", PointerValue (prop));
NetDeviceContainer devices = uan.Install (Sensor_Nodes, channel);
Ipv4StaticRoutingHelper staticRouting;
Ipv4ListRoutingHelper list;
list.Add (staticRouting, 0);
InternetStackHelper internet;
internet.SetRoutingHelper (list); 
internet.Install (Sensor_Nodes);
internet.Install (Sink_Node);
internet.Install (Server_Node);
InternetStackHelper internetv6;
internetv6.SetIpv4StackInstall (false);
Ipv4AddressHelper ipv4;
NS_LOG_INFO ("Assign IP Addresses.");
ipv4.SetBase ("10.1.1.0", "255.255.255.0");
Ipv4InterfaceContainer i = ipv4.Assign (staticdevices);
Ipv4InterfaceContainer iii = ipv4.Assign (switchdevices);
Ipv4InterfaceContainer iv = ipv4.Assign (controllerdevices);
TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
Ptr<Socket> recvSink = Socket::CreateSocket (Sensor_Nodes.Get (revNode), tid);
InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
Simulator::Schedule (Seconds (6.3), &PktTrans2, Sink_Node,Server_Node);
Simulator::Schedule (Seconds (9.3), &PktTrans3, Sink_Node,Server_Node);
recvSink->Bind (local);
recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
Ptr<Socket> source = Socket::CreateSocket (Sensor_Nodes.Get (sourceNode), tid);
InetSocketAddress remote = InetSocketAddress (i.GetAddress (revNode, 0), 80);
source->Connect (remote);
Simulator::Schedule (Seconds (1.3), &GenerateTraffic, source, packetSize, noofpkts, interPacketInterval);
Simulator::Schedule (Seconds (3.3), &PktTrans1, Sensor_Nodes,Sink_Node);
Simulator::Stop (Seconds (simtime));
macgplot mg;
mg.Packet_Delivery_Ratio(noOfNodes,"ALOHA");
mg.Energy_Consumption(noOfNodes,"ALOHA");
mg.Delay(noOfNodes,"ALOHA");
mg.Throughput(noOfNodes,"ALOHA");
pAnim= new AnimationInterface ("uan_aloha_mac_scenario.xml");
pAnim->SetBackgroundImage ("/home/research/ns-allinone-3.26/netanim-3.107/img1/bg.png", -825, -225, 9.500, 9.00, 1.0);
uint32_t Serverimg =pAnim->AddResource("/home/research/ns-allinone-3.26/netanim-3.107/img1/Server.png");
uint32_t Sinkimg =pAnim->AddResource("/home/research/ns-allinone-3.26/netanim-3.107/img1/Sink.png");
uint32_t Sensorimg =pAnim->AddResource("/home/research/ns-allinone-3.26/netanim-3.107/img1/Sensor.png");
for(  uint32_t i=0;i<Sensor_Nodes.GetN ();i++){
pAnim->UpdateNodeDescription (Sensor_Nodes.Get (i), "Sensor"); 
Ptr<Node> wid= Sensor_Nodes.Get (i);
uint32_t nodeId = wid->GetId ();
pAnim->UpdateNodeImage (nodeId, Sensorimg);
pAnim->UpdateNodeColor(Sensor_Nodes.Get(i), 255, 255, 0); 
pAnim->UpdateNodeSize (nodeId, 40.0,40.0);}
for(  uint32_t i=0;i<Sink_Node.GetN ();i++){
pAnim->UpdateNodeDescription (Sink_Node.Get (i), "Sink"); 
Ptr<Node> wid= Sink_Node.Get (i);
uint32_t nodeId = wid->GetId ();
pAnim->UpdateNodeImage (nodeId, Sinkimg);
pAnim->UpdateNodeColor(Sink_Node.Get(i), 0, 255, 0); 
pAnim->UpdateNodeSize (nodeId, 40.0,40.0);}
for(  uint32_t i=0;i<Server_Node.GetN ();i++){
pAnim->UpdateNodeDescription (Server_Node.Get (i), "Server"); 
Ptr<Node> wid= Server_Node.Get (i);
uint32_t nodeId = wid->GetId ();
pAnim->UpdateNodeImage (nodeId, Serverimg);
pAnim->UpdateNodeColor(Server_Node.Get(i), 0, 255, 0); 
pAnim->UpdateNodeSize (nodeId, 50.0,50.0);}
FlowMonitorHelper flowmon;
Ptr<FlowMonitor> monitor = flowmon.InstallAll();
Simulator::Run ();
monitor->CheckForLostPackets ();
uint32_t LostPacketsum = 0;
uint32_t rxPacketsum = 0;
uint32_t DropPacketsum = 0;
double DelaySum = 0.035; 
Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i){
rxPacketsum += (i->second.txBytes/(numNodes*10));
LostPacketsum += i->second.lostPackets;
DropPacketsum += i->second.packetsDropped.size();
DelaySum += i->second.delaySum.GetSeconds();}
Simulator::Destroy ();
system("gnuplot 'Packet_Delivery_Ratio.plt'");
system("gnuplot 'Energy_Consumption.plt'");
system("gnuplot 'Delay.plt'");
system("gnuplot 'Throughput.plt'");
return 0;}

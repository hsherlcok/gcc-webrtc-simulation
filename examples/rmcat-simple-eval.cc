/******************************************************************************
 * Copyright 2016-2017 cisco Systems, Inc.                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 * You may obtain a copy of the License at                                    *
 *                                                                            *
 *     http://www.apache.org/licenses/LICENSE-2.0                             *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 ******************************************************************************/

/**
 * @file
 * Simple example demonstrating the usage of the rmcat ns3 module, using:
 *  - NADA as controller for rmcat flows
 *  - Statistics-based traffic source as codec
 *  - [Optionally] TCP flows
 *  - [Optionally] UDP flows
 *
 * @version 0.1.1
 * @author Jiantao Fu
 * @author Sergio Mena
 * @author Xiaoqing Zhu
 */

#include "ns3/gcc-controller.h"
#include "ns3/nada-controller.h"
#include "ns3/gcc-sender.h"
#include "ns3/gcc-receiver.h"
#include "ns3/rmcat-constants.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/data-rate.h"
#include "ns3/bulk-send-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/traffic-control-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/core-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include <string>

// Maybe Ignore it 
const uint32_t GCC_DEFAULT_RMIN  =  150000;  // in bps: 150Kbps
const uint32_t GCC_DEFAULT_RMAX  = 1500000;  // in bps: 1.5Mbps

// TODO SHOULD MODIFY THIS BUT DON'T KNOW EXACT INITIAL VALUE.
const uint32_t GCC_DEFAULT_RINIT =  150000;  // in bps: 150Kbps (r_init)

const uint32_t TOPO_DEFAULT_BW     = 1000000;    // in bps: 1Mbps
const uint32_t TOPO_DEFAULT_PDELAY =      50;    // in ms:   50ms
const uint32_t TOPO_DEFAULT_QDELAY =     300;    // in ms:  300ms

using namespace ns3;

double
GetInterval(double lambda) {
  double X = rand()/(double)RAND_MAX;
 
  return -1 / lambda * log(X);
}

static NodeContainer BuildExampleTopo (uint64_t bps,
                                       uint32_t msDelay,
                                       uint32_t msQdelay)
{
    NodeContainer nodes;
    nodes.Create (2);

    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute ("DataRate", DataRateValue  (DataRate (bps)));
    pointToPoint.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (msDelay-20))); //send-router or recevier-router : 10ms
    auto bufSize = std::max<uint32_t> (DEFAULT_PACKET_SIZE, bps * msQdelay / 8000);
    pointToPoint.SetQueue ("ns3::DropTailQueue",
                           "Mode", StringValue ("QUEUE_MODE_BYTES"),
                           "MaxBytes", UintegerValue (bufSize));
    NetDeviceContainer devices = pointToPoint.Install (nodes);

    InternetStackHelper stack;
    stack.Install (nodes);
    Ipv4AddressHelper address;
    address.SetBase ("10.1.1.0", "255.255.255.0");
    address.Assign (devices);

    // Uncomment to capture simulated traffic
    // pointToPoint.EnablePcapAll ("rmcat-example");

    // disable tc for now, some bug in ns3 causes extra delay
    TrafficControlHelper tch;
    tch.Uninstall (devices);

    return nodes;
}

static void InstallLargeTCP (Ptr<Node> sender,
                        Ptr<Node> receiver,
                        uint16_t port,
                        float startTime,
                        float stopTime)
{
    // configure TCP source/sender/client
    auto serverAddr = receiver->GetObject<Ipv4> ()->GetAddress (1,0).GetLocal ();
    BulkSendHelper source{"ns3::TcpSocketFactory",
                           InetSocketAddress{serverAddr, port}};
    // Set the amount of data to send in bytes. Zero is unlimited.
    source.SetAttribute ("MaxBytes", UintegerValue (0));
    source.SetAttribute ("SendSize", UintegerValue (DEFAULT_PACKET_SIZE));

    auto clientApps = source.Install (sender);
    clientApps.Start (Seconds (startTime));
    clientApps.Stop (Seconds (stopTime));

    // configure TCP sink/receiver/server
    PacketSinkHelper sink{"ns3::TcpSocketFactory",
                           InetSocketAddress{Ipv4Address::GetAny (), port}};
    auto serverApps = sink.Install (receiver);
    serverApps.Start (Seconds (startTime));
    serverApps.Stop (Seconds (stopTime));

}

/*
static void InstallShortTCP (NodeContainer nodes, Ptr<Node> sender,
                        Ptr<Node> receiver,
                        uint16_t port, const float endTime)
{
    // configure TCP source/sender/client
    auto serverAddr = receiver->GetObject<Ipv4> ()->GetAddress (1,0).GetLocal ();
    BulkSendHelper source{"ns3::TcpSocketFactory",
                           InetSocketAddress{serverAddr, port}};
    
    source.SetAttribute ("MaxBytes", UintegerValue (30000)); //30Kbytes
    source.SetAttribute ("SendSize", UintegerValue (DEFAULT_PACKET_SIZE));

    NodeContainer srNodes[30];
    
    for (int i = 0; i < 30; i++) {
      srNodes[i] = ConnectSRNodes(nodes, linkBw, msDelay, msQDelay);
    }

    double startTime = 10;
    double stopTime = endTime;

    while(startTime < stopTime)
    {
      for(int i = 0;i<30;i++)
      {
        auto clientApps = source.Install (srNodes[i].Get(0));
        clientApps.Start (Seconds (startTime));
        clientApps.Stop (Seconds (stopTime));
        
        // configure TCP sink/receiver/server
        PacketSinkHelper sink{"ns3::TcpSocketFactory",
          InetSocketAddress{Ipv4Address::GetAny (), port}};
        auto serverApps = sink.Install (srNodes[i].Get(1));
        serverApps.Start (Seconds (startTime));
        serverApps.Stop (Seconds (stopTime));
      }

      startTime = startTime + GetInterval(10); //exponential avg : 10
    }
}
*/
static Time GetIntervalFromBitrate (uint64_t bitrate, uint32_t packetSize)
{
    if (bitrate == 0u) {
        return Time::Max ();
    }
    const auto secs = static_cast<double> (packetSize + IPV4_UDP_OVERHEAD) /
                            (static_cast<double> (bitrate) / 8. );
    return Seconds (secs);
}

static void InstallUDP (Ptr<Node> sender,
                        Ptr<Node> receiver,
                        uint16_t serverPort,
                        uint64_t bitrate,
                        uint32_t packetSize,
                        uint32_t startTime,
                        uint32_t stopTime)
{
    // configure UDP source/sender/client
    auto serverAddr = receiver->GetObject<Ipv4> ()->GetAddress (1,0).GetLocal ();
    const auto interPacketInterval = GetIntervalFromBitrate (bitrate, packetSize);
    uint32_t maxPacketCount = 0XFFFFFFFF;
    UdpClientHelper client{serverAddr, serverPort};
    client.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
    client.SetAttribute ("Interval", TimeValue (interPacketInterval));
    client.SetAttribute ("PacketSize", UintegerValue (packetSize));

    auto clientApps = client.Install (sender);
    clientApps.Start (Seconds (startTime));
    clientApps.Stop (Seconds (stopTime));

    // configure TCP sink/receiver/server
    UdpServerHelper server{serverPort};
    auto serverApps = server.Install (receiver);
    serverApps.Start (Seconds (startTime));
    serverApps.Stop (Seconds (stopTime));
}

static void InstallApps (bool gcc,
                         Ptr<Node> sender,
                         Ptr<Node> receiver,
                         uint16_t port,
                         float initBw,
                         float minBw,
                         float maxBw,
                         float startTime,
                         float stopTime)
{
    Ptr<GccSender> sendApp = CreateObject<GccSender> ();
    Ptr<GccReceiver> recvApp = CreateObject<GccReceiver> ();
    sender->AddApplication (sendApp);
    receiver->AddApplication (recvApp);

    if (gcc) {
        sendApp->SetController (std::make_shared<rmcat::GccController> ());
    }
    Ptr<Ipv4> ipv4 = receiver->GetObject<Ipv4> ();
    Ipv4Address receiverIp = ipv4->GetAddress (1, 0).GetLocal ();
    sendApp->Setup (receiverIp, port); // initBw, minBw, maxBw);

    const auto fps = 30.;		// Set Video Fps.
    auto innerCodec = new syncodecs::StatisticsCodec{fps};
    auto codec = new syncodecs::ShapedPacketizer{innerCodec, DEFAULT_PACKET_SIZE};
    sendApp->SetCodec (std::shared_ptr<syncodecs::Codec>{codec});

    recvApp->Setup (port);

    sendApp->SetStartTime (Seconds (startTime));
    sendApp->SetStopTime (Seconds (stopTime));

    recvApp->SetStartTime (Seconds (startTime));
    recvApp->SetStopTime (Seconds (stopTime));
}

static NodeContainer ConnectSRNodes (NodeContainer routers, uint64_t bps, uint32_t msDelay, uint32_t msQdelay, uint32_t count)
{
    NodeContainer nodes;
    nodes.Create (2);
  
    NodeContainer n0r0 = NodeContainer (nodes.Get(0), routers.Get (0));
    NodeContainer r1n1 = NodeContainer (routers.Get(1), nodes.Get (1));
  
    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute ("DataRate", DataRateValue  (DataRate (bps*2))); //non-bottleneck link : bps*2
    pointToPoint.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10)));
    auto bufSize = std::max<uint32_t> (DEFAULT_PACKET_SIZE, bps * 2 *  msQdelay / 8000);
    pointToPoint.SetQueue ("ns3::DropTailQueue",
                           "Mode", StringValue ("QUEUE_MODE_BYTES"),
                           "MaxBytes", UintegerValue (bufSize));

    NetDeviceContainer d_n0r0 = pointToPoint.Install (n0r0);
    NetDeviceContainer d_r1n1 = pointToPoint.Install (r1n1);


    std::stringstream s1, s2;
    s1 <<"10.0."<< 2*count<<".0";
    s2 <<"10.0."<< 2*count+1<<".0";

    InternetStackHelper stack;
    stack.Install (nodes);
    Ipv4AddressHelper address;
    address.SetBase (Ipv4Address(s1.str().c_str()), "255.255.255.0");
    address.Assign (d_n0r0);
    address.SetBase (Ipv4Address(s2.str().c_str()), "255.255.255.0");
    address.Assign (d_r1n1);

    // Uncomment to capture simulated traffic
    // pointToPoint.EnablePcapAll ("rmcat-example");

    // disable tc for now, some bug in ns3 causes extra delay
    TrafficControlHelper tch;
    tch.Uninstall (d_n0r0);
    tch.Uninstall (d_r1n1);

    return nodes;
}

int main (int argc, char *argv[])
{
    // Number of Flows 
    int nWebRTC = 1;
    int nTcp = 0;
    bool shortTcp = false;
    int nUdp = 0;

    bool log = false;
    bool gcc = true;

    std::string strArg  = "strArg default";

    CommandLine cmd;
    cmd.AddValue ("webrtc", "Number of WebRTC (GCC) flows", nWebRTC);
    cmd.AddValue ("tcp", "Number of TCP flows", nTcp);
    cmd.AddValue ("shortTcp", "Generate Short TCP flow?", shortTcp);
    cmd.AddValue ("udp",  "Number of UDP flows", nUdp);
    cmd.AddValue ("log", "Turn on logs", log);
    cmd.AddValue ("gcc", "true: use GCC, false: use dummy", gcc);   // Default is declared in rmcat-sender.cc
    cmd.Parse (argc, argv);

    if (log) {
        LogComponentEnable ("GccSender", LOG_INFO);
        LogComponentEnable ("GccReceiver", LOG_INFO);
//        LogComponentEnable ("RmcatReceiver", LOG_INFO);
//        LogComponentEnable ("Packet", LOG_FUNCTION);
    }

    // configure default TCP parameters
    Config::SetDefault ("ns3::TcpSocket::DelAckCount", UintegerValue (0));
    Config::SetDefault ("ns3::TcpL4Protocol::SocketType", StringValue ("ns3::TcpNewReno"));    // Tcp Type
    Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1000));

    const uint64_t linkBw   = TOPO_DEFAULT_BW;
    const uint32_t msDelay  = TOPO_DEFAULT_PDELAY;
    const uint32_t msQDelay = TOPO_DEFAULT_QDELAY;

    const float minBw =  GCC_DEFAULT_RMIN;
    const float maxBw =  GCC_DEFAULT_RMAX;
    const float initBw = GCC_DEFAULT_RINIT;

    const float endTime = 300.;
    uint32_t count=0;

    NodeContainer nodes = BuildExampleTopo (linkBw, msDelay, msQDelay);

    int port = 8000;
    for (int i = 0; i < nWebRTC; i++) {
        auto start = 10. * i;
        auto end = std::max (start + 1., endTime - start);

        count++;
        NodeContainer srNodes = ConnectSRNodes(nodes, linkBw, msDelay, msQDelay,count);
        InstallApps (gcc, srNodes.Get (0), srNodes.Get (1), port++,
                     initBw, minBw, maxBw, start, end);
    }


    //if(shortTCP)
  //  {
  //    InstallShortTCP (nodes, srNodes.Get (0), srNodes.Get (1), port++, endTime);
  //  }
 //   else
 //   {
      for (int i = 0; i < nTcp; i++) {
        auto start = 17. * i;
        auto end = std::max (start + 1., endTime - start);
        count++;
        NodeContainer srNodes = ConnectSRNodes(nodes, linkBw, msDelay, msQDelay,count);
        InstallLargeTCP (srNodes.Get (0), srNodes.Get (1), port++, start, end);
      }
   // }

    // UDP parameters
    const uint64_t bandwidth = GCC_DEFAULT_RMAX / 4;
    const uint32_t pktSize = DEFAULT_PACKET_SIZE;

    for (int i = 0; i < nUdp; i++) {
        auto start = 23. * i;
        auto end = std::max (start + 1., endTime - start);
        count++;
        NodeContainer srNodes = ConnectSRNodes(nodes, linkBw, msDelay, msQDelay,count);
        InstallUDP (srNodes.Get (0), srNodes.Get (1), port++,
                    bandwidth, pktSize, start, end);
    }


  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

    std::cout << "Running Simulation..." << std::endl;
    Simulator::Stop (Seconds (endTime));
    Simulator::Run ();
    Simulator::Destroy ();
    std::cout << "Done" << std::endl;

    return 0;
}

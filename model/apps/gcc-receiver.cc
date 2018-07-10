/******************************************************************************
 * Copyright 2016-2017 Cisco Systems, Inc.                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 *                                                                            *
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
 * Receiver application implementation for rmcat ns3 module.
 *
 * @version 0.1.1
 * @author Jiantao Fu
 * @author Sergio Mena
 * @author Xiaoqing Zhu
 */

#include "gcc-receiver.h"
#include "rmcat-constants.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("GccReceiver");

#define LOGTIMER 10
#define RTTLOG 1
namespace ns3 {
GccReceiver::GccReceiver ()
: m_running{false}
, m_waiting{false}
, m_ssrc{0}
, m_remoteSsrc{0}
, m_srcIp{}
, m_srcPort{}
, m_socket{NULL}
, m_header{}
, m_sendEvent{}
, m_periodUs{RMCAT_FEEDBACK_PERIOD_US}
, m_movertt{0}
{}

GccReceiver::~GccReceiver () {}

void GccReceiver::Setup (uint16_t port)
{
    m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
    auto local = InetSocketAddress{Ipv4Address::GetAny (), port};
    auto ret = m_socket->Bind (local);
    NS_ASSERT (ret == 0);
    m_socket->SetRecvCallback (MakeCallback (&GccReceiver::RecvPacket, this));

    m_running = false;
    m_waiting = true;
}

void GccReceiver::StartApplication ()
{
    m_running = true;
    m_ssrc = rand ();
    m_header.SetSendSsrc (m_ssrc);
    Time tFirst {MicroSeconds (m_periodUs)};

    m_timer = ns3::Seconds(0);
    m_numPackets = 0;
   
    m_movertt = 0;
    m_rttT = ns3::Seconds(0);
}

void GccReceiver::StopApplication ()
{
    m_running = false;
    m_waiting = true;
    m_header.Clear ();
    Simulator::Cancel (m_sendEvent);
}

void GccReceiver::RecvPacket (Ptr<Socket> socket)
{
    if (!m_running) {
        return;
    }

    Address remoteAddr{};
    auto packet = m_socket->RecvFrom (remoteAddr);
   
    // std::cout<<"shit : " << packet->GetSize()<<std::endl;
    m_numPackets += packet->GetSize();

    if(m_timer + ns3::Seconds(LOGTIMER) < ns3::Simulator::Now())
    { 
       std::cout<<ns3::Simulator::Now()<<" Node ID : "<<GetNode()->GetId()<<" ptr : "<<this<<" Recv Throuhput per 10s : "<<(double)m_numPackets*8/((ns3::Simulator::Now()-m_timer).ToDouble(ns3::Time::S)*1000*1000)<<std::endl;
       m_timer = ns3::Simulator::Now();
       m_numPackets = 0;
    }

    NS_ASSERT (packet);
    RtpHeader header{};
    NS_LOG_INFO ("GccReceiver::RecvPacket, " << packet->ToString ());
    packet->RemoveHeader (header);
    auto srcIp = InetSocketAddress::ConvertFrom (remoteAddr).GetIpv4 ();
    const auto srcPort = InetSocketAddress::ConvertFrom (remoteAddr).GetPort ();
    if (m_waiting) {
        m_waiting = false;
        m_remoteSsrc = header.GetSsrc ();
        m_srcIp = srcIp;
        m_srcPort = srcPort;
    } else {
        // Only one flow supported
        NS_ASSERT (m_remoteSsrc == header.GetSsrc ());
        NS_ASSERT (m_srcIp == srcIp);
        NS_ASSERT (m_srcPort == srcPort);
    }
    uint64_t txTimestampUs = header.GetTimestamp();
    uint64_t recvTimestampUs = Simulator::Now ().GetMicroSeconds ();
    m_movertt = m_movertt * .8 + (recvTimestampUs - txTimestampUs) * .2;
    
    if(m_rttT + ns3::Seconds(RTTLOG) < ns3::Simulator::Now()) {
        std::cout << "movertt : " << m_movertt << std::endl;
        m_rttT = ns3::Simulator::Now();
    }
    
    AddFeedback (header.GetSequence (), recvTimestampUs);
    m_sendEvent = Simulator::ScheduleNow(&GccReceiver::SendFeedback, this, false);
}

void GccReceiver::AddFeedback (uint16_t sequence,
                                 uint64_t recvTimestampUs)
{
    auto res = m_header.AddFeedback (m_remoteSsrc, sequence, recvTimestampUs);
    // std::cout << "AddFeedback:: " << sequence << "\n";
    if (res == CCFeedbackHeader::CCFB_TOO_LONG) {
        SendFeedback (false);
        res = m_header.AddFeedback (m_remoteSsrc, sequence, recvTimestampUs);
    }
    NS_ASSERT (res == CCFeedbackHeader::CCFB_NONE);
}

void GccReceiver::SendFeedback (bool reschedule)
{
    if (m_running && !m_header.Empty ()) {
        //TODO (authors): If packet empty, easiest is to send it as is. Propose to authors
        auto packet = Create<Packet> ();
        packet->AddHeader (m_header);
        NS_LOG_INFO ("GccReceiver::SendFeedback, " << packet->ToString ());
        m_socket->SendTo (packet, 0, InetSocketAddress{m_srcIp, m_srcPort});

        m_header.Clear ();
        m_header.SetSendSsrc (m_ssrc);
    }

    if (reschedule) {
        Time tNext {MicroSeconds (m_periodUs)};
        m_sendEvent = Simulator::Schedule (tNext, &GccReceiver::SendFeedback, this, true);
    }
}

}


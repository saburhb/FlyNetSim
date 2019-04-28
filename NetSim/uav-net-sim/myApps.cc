#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "myApps.h"

pthread_mutex_t tNext_mutex;
long last_schedule_time = 0;
long g_cng_start = 0;
long g_cng_end = 0;
long g_cng_total_bytes = 0;
int cong_new_rate = 1;

using namespace ns3;

MyApp::MyApp ()
  : m_socket (0),
    m_peer (),
    m_packetSize (0),
    m_nPackets (0),
    m_dataRate (0),
    m_sendEvent (),
    m_running (false),
    m_packetsSent (0),
    m_appId(0),
    m_appType(0)   //Command: 0 , Telemetry: 1
{
}

MyApp::~MyApp()
{
  m_socket = 0;
  m_elapse = 0;
}

void MyApp::PrintParameters ()
{
   std::cout << "Socket Value : " << m_socket << std::endl;
   std::cout << "Peer Address : " << m_peer << std::endl;
   std::cout << "Packet Size : " << m_packetSize << std::endl;
   std::cout << "Number of Packets : " << m_nPackets << std::endl;
   std::cout << "Data Rate : " << m_dataRate << std::endl;
   std::cout << "Application ID : " << m_appId << std::endl;
   std::cout << "Application Type : " << m_appType << std::endl;

}


void MyApp::Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate, uint32_t appId, uint32_t appType)
{
  m_socket = socket;
  m_peer = address;
  m_packetSize = packetSize;
  m_nPackets = nPackets;
  m_dataRate = dataRate;
  m_appId = appId;
  m_appType = appType;
}

void MyApp::StartApplication (void)
{
  m_running = true;
  m_packetsSent = 0;
  m_socket->Bind ();
  m_socket->Connect (m_peer);
  std::cout << " Socket : " << m_socket  << std::endl;
  std::cout << " Peer Address : " << m_peer << std::endl;
  m_elapse = (double)(ns3::Simulator::Now().GetSeconds());

}

void MyApp::StopApplication (void)
{
  m_running = false;

  if (m_sendEvent.IsRunning ())
    {
      Simulator::Cancel (m_sendEvent);
    }

  if (m_socket)
    {
      m_socket->Close ();
    }
}


void MyApp::SendMsg (Ptr<Socket> send_socket, char *msg)
{
  int len = strlen(msg);
  pthread_mutex_lock(&tNext_mutex);
  //Ptr<Packet> packet = Create<Packet> ((uint8_t*)msg, (len+50));
  Ptr<Packet> packet = Create<Packet> ((uint8_t*)msg, (1448));
  std::cout << " SendMsg: Socket : " << m_socket  << " PKT SIZE: " << len << std::endl;
  send_socket->Send (packet);
  free(msg);
  pthread_mutex_unlock(&tNext_mutex);

}

void MyApp::SendPacket (void)
{
  m_packetsSent++;
  Ptr<Packet> packet = Create<Packet> (m_packetSize);
  g_cng_end =  ns3::Simulator::Now().GetMilliSeconds();
  g_cng_total_bytes += m_packetSize;
  long diff_cng = g_cng_end - g_cng_start;

  if(diff_cng > 5000)  //print the contending traffic rate every 5 sec
  {
    cong_new_rate = (int)(m_rate);
    std::cout << "############### CONTENDING TRAFFIC RATE : " <<  m_rate  << "  Mbps ##################" << std::endl;  
    g_cng_total_bytes = 0;
    g_cng_start = g_cng_end;
  }

  m_socket->Send (packet);

  ScheduleTx ();

}


void MyApp::ScheduleTx (void)
{
      double next_sched = (((double)(m_packetSize) * 8) / (double)(m_rate*1000000));
      Time tNext (Seconds (next_sched));

      m_sendEvent = Simulator::Schedule (tNext, &MyApp::SendPacket, this);
      last_schedule_time = tNext.GetSeconds();
}


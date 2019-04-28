#ifndef MYAPPS_H_
#define MYAPPS_H_
#include <stdio.h>
#include <stdlib.h>
#include "ns3/applications-module.h"

extern pthread_mutex_t tNext_mutex;
extern pthread_mutex_t mob_mutex;
extern long last_schedule_time;
extern int cong_new_rate;

using namespace ns3;

class MyApp : public Application
{
public:

  MyApp ();
  virtual ~MyApp();

  void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate, uint32_t appId, uint32_t appType);
  void ChangeRate(DataRate newrate);
  void SendMsg (Ptr<Socket>, char *);
  void PrintParameters ();
  virtual void StartApplication (void);
  virtual void StopApplication (void);
  void SendPacket (void);
  void ScheduleTx (void);

  Ptr<Socket>     m_socket;
  Address         m_peer;
  int             m_rate;
  double          m_elapse;
  int             m_congId;

private:

  uint32_t        m_packetSize;
  uint32_t        m_nPackets;
  DataRate        m_dataRate;
  EventId         m_sendEvent;
  bool            m_running;
  uint32_t        m_packetsSent;
  uint32_t        m_appId;   //Odd: LTE App , Even: Wifi App
  uint32_t        m_appType;   //Command: 0 , Telemetry: 1
};

#endif

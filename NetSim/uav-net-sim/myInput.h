#ifndef MYINPUT_H_
#define MYINPUT_H_
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

using namespace std;


class MyInput
{
public:

  MyInput (); 
  virtual ~MyInput();

  void loadInput (void);

  int m_num_uav;
  int m_network;  //0: WiFi, 1: LTE
  int m_num_traffic;
  float m_traf_rate;
  int m_traf_size;

  vector<float> m_x_values;
  vector<float> m_y_values;
  vector<float> m_z_values;


private:

};

#endif


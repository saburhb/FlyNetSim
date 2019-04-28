#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <libxml/parser.h>
#include <libxml/xmlIO.h>
#include <libxml/xinclude.h>
#include <libxml/tree.h>

#include "myInput.h"

#define DOCNAME "config.xml"

using namespace std;


MyInput::MyInput ()
{
    m_num_uav = 0;
    m_network = 0;
    m_num_traffic = 0;
    m_traf_rate = 0.0;
    m_traf_size = 0;
}

MyInput::~MyInput()
{

}

void MyInput::loadInput (void)
{
  /****************** XML PARSING ******************/
  /*************************************************/
  xmlDocPtr doc;
  xmlNodePtr cur;
  xmlChar *key;

  doc = xmlParseFile(DOCNAME);
  if(doc == NULL){
    fprintf(stderr, "Document not parsed successfully \n");
  }

  cur = xmlDocGetRootElement(doc);
  if(cur == NULL){
    fprintf(stderr, "Empty document \n");
    xmlFree(doc);
  }

  printf("******* Parsing XML file ********\n");

  cur = cur->xmlChildrenNode;
  while (cur != NULL)
  {
    if(!xmlStrcmp(cur->name, (const xmlChar *)"instance"))  // get number of UAVs
    {   
      key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1); 
      printf("instance = %s \n", key);
      char *num_ins = (char*)key;
      m_num_uav = atoi(num_ins);
      xmlFree(key);
    }   
    if(!xmlStrcmp(cur->name, (const xmlChar *)"network")) // get the network type
    {   
      key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1); 
      char *netype = (char*)key;
      m_network = atoi(netype);
      xmlFree(key);
    }  

    if(!xmlStrcmp(cur->name, (const xmlChar *)"traffic")) //get the congestion details
    {
      key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
      printf("Traffic ... \n");
      xmlNodePtr tr_cur = cur->xmlChildrenNode;
      while (tr_cur != NULL)
      {
        if(!xmlStrcmp(tr_cur->name, (const xmlChar *)"count")) // number of congestion nodes
        {
          key = xmlNodeListGetString(doc, tr_cur->xmlChildrenNode, 1);
          printf("count = %s \n", key);
          char *num_trf = (char*)key;
          m_num_traffic = atoi(num_trf);
          xmlFree(key);
        }
        if(!xmlStrcmp(tr_cur->name, (const xmlChar *)"rate")) // datarate of each congestion nodes
        {
          key = xmlNodeListGetString(doc, tr_cur->xmlChildrenNode, 1);
          printf("rate = %s \n", key);
          char *rate_trf = (char*)key;
          m_traf_rate = atof(rate_trf);
          xmlFree(key);
        }
        if(!xmlStrcmp(tr_cur->name, (const xmlChar *)"size")) // size of congestion packets
        {
          key = xmlNodeListGetString(doc, tr_cur->xmlChildrenNode, 1);
          printf("size = %s \n", key);
          char *size_trf = (char*)key;
          m_traf_size = atoi(size_trf);
          xmlFree(key);
        }
        tr_cur = tr_cur->next;
      } 
    } //finished traffic details

    if(!xmlStrcmp(cur->name, (const xmlChar *)"vehicle")) // get the vehicle position details
    {
      // Create position allocator for the vehicles

      key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
      printf("Vehicle ... \n");
      xmlNodePtr vh_cur = cur->xmlChildrenNode;
      while (vh_cur != NULL)
     {
        if(!xmlStrcmp(vh_cur->name, (const xmlChar *)"uav")) // get the coordinates for each UAV
        {
          //get the location of the vehicles

          key = xmlNodeListGetString(doc, vh_cur->xmlChildrenNode, 1);
          printf("uav... \n");
          xmlNodePtr uav_cur = vh_cur->xmlChildrenNode;
          while (uav_cur != NULL)
          {
            if(!xmlStrcmp(uav_cur->name, (const xmlChar *)"x"))
            {
              key = xmlNodeListGetString(doc, uav_cur->xmlChildrenNode, 1);
              printf("x = %s \n", key);
              char *x_c = (char*)key;
              //x_f = atof(x_c);
              m_x_values.push_back(atof(x_c));
              xmlFree(key);
            }
            if(!xmlStrcmp(uav_cur->name, (const xmlChar *)"y"))
            {
              key = xmlNodeListGetString(doc, uav_cur->xmlChildrenNode, 1);
              printf("y = %s \n", key);
              char *y_c = (char*)key;
              //y_f = atof(y_c);
              m_y_values.push_back(atof(y_c));
              xmlFree(key);
            }
            if(!xmlStrcmp(uav_cur->name, (const xmlChar *)"z"))
            {
              key = xmlNodeListGetString(doc, uav_cur->xmlChildrenNode, 1);
              printf("z = %s \n", key);
              char *z_c = (char*)key;
              //z_f = atof(z_c);
              m_z_values.push_back(atof(z_c));
              xmlFree(key);
            }

            uav_cur = uav_cur->next;
          }

        }  // finished position details of a UAV
        vh_cur = vh_cur->next;
      }
    } //finished position details of all vehicles

    cur = cur->next;
  }
  /************  END OF XML PARSING **************/




}




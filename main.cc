// Copyright 2008 Isis Innovation Limited
// This is the main extry point for PTAM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "System.h"


using namespace std;
using namespace GVars3;

int main()
{
  cout << "Stereo PTAM" << endl;
  cout << endl;
  cout << "  Parsing settings_[1|2|3|4].cfg ...." << endl;

  //load from two settings files to get exec command (probably should be able to do this from 1 settings file)
  GUI.LoadFile("settings_2.cfg");
  GUI.LoadFile("settings_1.cfg");
  
  GUI.StartParserThread(); // Start parsing of the console input
  atexit(GUI.StopParserThread); 

  try
    {
      System s; // this is the main system class, acts as an event loop
      s.Run();
    }
  catch(CVD::Exceptions::All e)
    {
      cout << endl;
      cout << "!! Failed to run system; got exception. " << endl;
      cout << "   Exception was: " << endl;
      cout << e.what << endl;
    }
}











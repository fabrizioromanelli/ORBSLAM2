/**
* UWB Bias characterization
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <System.h>
#include <sys/stat.h>

#include "UwbApi.h"

using namespace std;

#define NODENUMBER 3
#define NODETARGET 1    // Set the correct Node (i.e. U3)
#define MEASURE_MX 400 // Number of maximum measurements

int counter = 0;

const node_id_t uwb_master_id = 0x0E9E;
const node_id_t uwb_slave_ids[NODENUMBER] = {0x0C16, 0x45BB, 0x4890};
vector<uint16_t> uwbReadings;

// void saveUWBreadings(const string &, vector<double>, vector<vector<uint16_t>>);

static void distanceFromUWBs(rall_descr_t* rall) {
  if (counter < MEASURE_MX)
    multi_range_with(uwb_master_id, uwb_slave_ids, NODENUMBER);

  uint16_t tmp;
  tmp = rall->distances[NODETARGET];

  if (tmp < 800)
  {
    uwbReadings.push_back(tmp);
    counter++;
  } // otherwise the measurements are not correct...
}


int main(int argc, char **argv)
{
  if(argc != 2)
  {
    cerr << endl << "Usage: ./uwb_bias" << endl 
                 << "         specify device file name [e.g. /dev/ttyACM0]" << endl
                 << "Example: ./Test/Live/UWB/uwb_bias /dev/ttyACM0" << endl;
    return 1;
  }

  try {
    init(argv[1]);
    sleep(1);
    // Registering callback to get distances from UWBs
    register_rall_cb(distanceFromUWBs);

    cout << endl << "-------" << endl;
    cout << "Start bias characterization ..." << endl;

    int distance;
    bool runFlag = true;

    // Main loop
    while(runFlag)
    {
      counter = 0;
      uwbReadings.clear(); // clean up stuff...
      cout << endl << "Insert distance in centimeters (-1 to stop): ";
      cin >> distance;
      if (distance == -1)
      {
        runFlag = false;
        continue;
      }

      multi_range_with(uwb_master_id, uwb_slave_ids, NODENUMBER);
      // cout << "Measuring..." << endl;
      while (counter < MEASURE_MX) 
      {
        sleep(1);
        // cout << ".";
      }
      // cout << endl;

      long int sum = 0;
      // Computing average of the measurement
      for (size_t i = 0; i < uwbReadings.size(); i++)
      {
        sum = sum + uwbReadings[i];
      }
      double averageMeasurement = (double)sum/(double)uwbReadings.size();
      cout << distance << " " << averageMeasurement << endl;
    }
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}

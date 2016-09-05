#include "include/tapi_clientlib/publisher.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Publisher::Publisher(ros::NodeHandle* nh, string nodename)
  : TapiClient(nh, nodename, PUBLISHER_DEVICE), nh(nh), nodename(nodename)
{
}

Publisher::~Publisher()
{
  for (int i = 0; i < publishers.size(); i++)
  {
    publishers[i]->shutdown();
    delete publishers[i];
  }
}
}

#include "Cinder.h"

// First waypoint, middle of lawn in front of 1189 Millbrae Ave
const float xlat = 37.59083;
const	float xlon = -122.402407;

Cinder cinder;

void setup()
{
  cinder.begin();
}

void loop()
{
  cinder.driveTo(xlat, xlon);
}

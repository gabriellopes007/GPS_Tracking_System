/* Created by Matheau Goonan
   Modified by Gabriel Lopes to fit the project
   Reference: http://forum.arduino.cc/index.php?topic=45760.0
*/

#ifndef Haversine_h
#define Haversine_h

#include "Arduino.h"

class Haversine {
   public:
      Haversine() {};
      double dtor(double fdegrees);
      double rtod(double fradians);
      double calcDistance(double lat1, double lon1, double lat2, double lon2);
      double calcBearing(double lat1, double lon1, double lat2, double lon2);
      void calcPoint(double lat1, double lon1, int intBearing, int intDistance, volatile double *lat2, volatile double *lon2);
};
#endif

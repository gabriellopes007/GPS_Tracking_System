/* Created by Matheau Goonan
   Modified by Gabriel Lopes to fit the project
   Reference: http://forum.arduino.cc/index.php?topic=45760.0
*/

#include "Arduino.h"
#include "Haversine.h"

//convert degrees to radians
double Haversine::dtor(double fdegrees) {
  return(fdegrees * PI / 180);
}

//Convert radians to degrees
double Haversine::rtod(double fradians) {
  return(fradians * 180.0 / PI);
}

//Calculate distance form lat1/lon1 to lat2/lon2 using haversine formula
//Returns distance in meters
double Haversine::calcDistance(double lat1, double lon1, double lat2, double lon2) {
  double dlon, dlat, a, c;
  double dist = 0.0;
  dlon = dtor(lon2 - lon1);
  dlat = dtor(lat2 - lat1);
  a = pow(sin(dlat/2),2) + cos(dtor(lat1)) * cos(dtor(lat2)) * pow(sin(dlon/2),2);
  c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  dist = 6378140.0f * c;  //radius of the earth (6378140 meters) in feet 20925656.2
  return(dist);
}

//Calculate bearing from lat1/lon1 to lat2/lon2
//Returns bearing in degrees
double Haversine::calcBearing(double lat1, double lon1, double lat2, double lon2) {
  lat1 = dtor(lat1);
  lon1 = dtor(lon1);
  lat2 = dtor(lat2);
  lon2 = dtor(lon2);
  
  //determine angle
  double bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)));
  //convert to degrees
  bearing = rtod(bearing);
  //use mod to turn -90 = 270
  bearing = fmod((bearing + 360.0), 360);
  bearing = dtor(bearing);
  return (bearing);
}

void Haversine::calcPoint(double lat1, double lon1, int intBearing, int intDistance, volatile double *lat2, volatile double *lon2) {
   double bearing = dtor((double)intBearing);
   double dist = (double)intDistance / 6378140;
   lat1 = dtor(lat1);
   lon1 = dtor(lon1);
   *lat2 = asin(sin(lat1) * cos(dist) + cos(lat1) * sin(dist) * cos(bearing));
   *lon2 = lon1 + atan2( sin(bearing) * sin(dist) * cos(lat1), cos(dist) - sin(lat1) * sin(*lat2) );
   *lon2 = fmod(*lon2 + 3 * PI, 2 * PI) - PI;
   *lon2 = rtod(*lon2);
   *lat2 = rtod(*lat2);
}

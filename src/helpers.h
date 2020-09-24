#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
extern string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
extern constexpr double pi();
extern double deg2rad(double x);
extern double rad2deg(double x);

// Calculate distance between two points
extern double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
extern int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) ;


// Returns next waypoint of the closest waypoint
extern int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
extern vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
extern vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) ;

#endif  // HELPERS_H
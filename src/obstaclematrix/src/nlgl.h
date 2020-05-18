// This is start of the header guard.  ADD_H can be any unique name.  By convention, we use the name of the header file.
#ifndef NLGL_H
#define NLGL_H

// nlgl function
double nlgl(double[] , double, double[]);
double nlgl_3d_map(double[] , double,double[]);

// wrap the angle
double ang_wrap(double angle);

double enuCompass(double ang);

double rad2deg(double ang);

double deg2rad(double ang);

double distance(double lat1, double lon1, double lat2, double lon2);

double bearing(double lat1, double lon1, double lat2, double lon2);


// This is the end of the header guard
#endif

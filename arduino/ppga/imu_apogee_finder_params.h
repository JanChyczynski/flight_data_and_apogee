#ifndef IMU_APOGEE_FINDER_PARAMS
#define IMU_APOGEE_FINDER_PARAMS

typedef float imu_acc_t;

int IMU_STREAK_REQ = 3; //how many consecutive readings have to be obtained to count that as an apogee
int IMU_GRAVITY_AXIS = 2; //if imu is not moving on which axis is it showing 1.0 or -1.0?
bool IMU_IS_GRAVITY_NEGATIVE = 1; //if imu is not moving is it showing -1.0?
double IMU_TOLERANCE = 0.3; //if set for 0.1 everything from 0.9 to 1.1 will count as gravity

#endif

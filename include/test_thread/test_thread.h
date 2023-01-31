#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pthread.h>      // pthread 사용하기 위한 헤더
#include <unistd.h>
#include <time.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <math.h>

#ifndef TEST_THREAD_H
#define TEST_THREAD_H

// Dynamixel Control
#define DEVICENAME                      "/dev/ttyUSB0"
#define PROTOCOL_VERSION                2.0

#define DXL1_ID                         1
#define DXL2_ID                         7
#define BAUDRATE                        2000000
#define ADDR_TORQUE_ENABLE              64
#define TORQUE_ENABLE                   1
#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           132
// Data Byte Length
#define LEN_PRO_GOAL_POSITION            4
#define LEN_PRO_PRESENT_POSITION         4

#define PI 3.141592
// #define L1
// #define L2
#endif // TEST_THREAD_H

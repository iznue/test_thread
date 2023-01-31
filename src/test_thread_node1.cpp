#include "test_thread/test_thread.h"

// pthread로 sync read&write를 이용한 모터 구동
// ROS 통신을 수행하는 2개의 while문 실행

uint8_t dxl_error = 0;                  // Dynamixel error
uint8_t param_goal_position[4];         // 각 Dynamixel에 쓰기 위한 목표 위치 값
int dxl_comm_result = COMM_TX_FAIL;     // Communication result
// int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};

bool dxl_addparam_result = false;       // addParam result
bool dxl_getdata_result = false;        // GetParam result


// Declare object
// Initialize PortHandler instance
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

// Initialize Groupsyncread instance for Present Position
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

/********************************************************************/

// 변수 선언
double i = 0;

bool is_run = true;
int Control_Cycle = 10;

// 로봇팔 길이
double L1 = 0.1;
double L2 = 0.1;

struct End_point{
  double x;
  double y;
};

struct Joint{
  double TH1;
  double TH2;
};

/********************************************************************/

// 함수 선언
void process(void);
void dxl_initialize(void);
void set_dxl_goal(int dxl_1_posi, int dxl_2_posi);
void dxl_go(void);
struct Joint Compute_IK(struct End_point EP);

/********************************************************************/


void process(void){
  struct End_point EP_goal;
  EP_goal.x = 0;
  EP_goal.y = 0.15;

  struct Joint J_goal;

}


void *p_function(void *data){
  dxl_initialize();

  static struct timespec next_time;
  // static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while(is_run){
    next_time.tv_sec += (next_time.tv_nsec + Control_Cycle*1000000)/1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + Control_Cycle*1000000)%1000000000;

    process();

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_thread_node1");
  ros::NodeHandle nh;

  pthread_t pthread;
  int thr_id;
  int status;
  char p1[] = "thread_1";

  sleep(1);

  thr_id = pthread_create(&pthread, NULL, p_function, (void*)p1);
  // thread 생성
  if(thr_id < 0){
    ROS_ERROR("pthread create error");
    exit(EXIT_FAILURE);
  }
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  is_run = false;   // 프로그램 종료 직전 is_run을 false로 바꾸고 p_function 종료
  return 0;
}


void dxl_initialize(void){
  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  // Enable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  // Add parameter storage for Dynamixel#2 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);

}


void set_dxl_goal(int dxl_1_posi, int dxl_2_posi){
  // Allocate goal position value into byte array
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_1_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_1_posi));

  // Add Dynamixel#1 goal position value to the Syncwrite storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);

  // Allocate goal position value into byte array
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_2_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_2_posi));

  // Add Dynamixel#1 goal position value to the Syncwrite storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);

}

void dxl_go(void){
  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite.txPacket();

}


struct Joint Compute_IK(struct End_point EP){
  // IK solution
  double x = EP.x;
  double y = EP.y;

  double alpha = atan2(y,x);
  double L = sqrt(pow(x,2) + pow(y,2));
  double beta = acos((pow(L1,2)+pow(L2,2)-pow(L,2))/(2*L1*L2));

  double th2 = PI - beta;

  double delta = atan2(x,y);
  double gamma = acos((pow(L,2)+pow(L1,2)-pow(L2,2))/(2*L1*L2));

  double th1 = (PI)/2 - delta - gamma;

  printf("th1 = %f, th2 = %f", th1, th2);

  struct Joint J;
  J.TH1 = th1;
  J.TH2 = th2;
  return J;

}


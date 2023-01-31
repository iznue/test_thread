#include "test_thread/test_thread.h"

uint8_t dxl_error = 0;      // Dynamixel error
// int dxl_goal_position[2] = {0,2000};
int dxl_comm_result = COMM_TX_FAIL;

bool dxl_addparam_result = false;                // addParam result
bool dxl_getdata_result = false;                 // GetParam result

uint8_t param_goal_position[4];   // 각 Dynamixel에 쓰기 위한 목표 위치 값

int32_t dxl1_present_position = 0, dxl2_present_position = 0;

// Declare object
// Initialize PortHandler, PacketHandler instance
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION);      // goal_position에만 관여
// Initialize Groupsyncread instance for Present Position
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);


double i = 0;

bool is_run = true;
int Control_Cycle = 10;   //ms, thread와 관련

// time variable
double t = 0;
double dt = 10;   //ms
double T = 1000;  //ms

// goals variable
int present_posi = 0;
int goal_posi = 1000;
int target_posi = 0;

// len variable
double L1 = 0.12;
double L2 = 0.12;


// 구조체 선언
struct End_point{
  double x;
  double y;
};

struct Joint{
  double TH1;
  double TH2;
};


// functions
void process(void);
void dxl_initailize(void);
void set_dxl_goal(int dxl_1_posi, int dxl_2_posi);
void dxl_go(void);
struct Joint Compute_IK(struct End_point EP);
struct End_point Compute_FK(struct Joint J);
int radian_to_tick1(double radian);
int radian_to_tick2(double radian);
void dxl_add_param(void);
void read_dxl_position(void);
double tick_to_radian1(int tick);
double tick_to_radian2(int tick);
struct End_point get_present_posi();

// 구조체 초기화
/*
struct End_point EP_goal;         // struct End_point EP_goal = {0,0.15};
EP_goal.x = 0;
EP_goal.y = 0.15;

struct Joint J_goal;
*/


void process(void){
  // 1-cos 궤적으로 이동
/*
  if (t <= T){
    goal_posi = present_posi + (target_posi - present_posi) * 0.5*(1-cos(PI*(t/T)));
    t++;
    set_dxl_goal(goal_posi,goal_posi);
    dxl_go();
    groupSyncWrite.clearParam();
  }
  else{
    t = 0;      // t값 초기화
    present_posi = target_posi;   // 현재 위치 재설정
  }
*/
/*
  struct End_point EP_goal;         // struct End_point EP_goal = {0,0.15};
  EP_goal.x = 0;
  EP_goal.y = 0.15;

  struct Joint J_goal;
  // J_goal.TH1 = 45;
  // J_goal.TH2 = 30;

  J_goal = Compute_IK(EP_goal);

  // EP_goal = Compute_FK(J_goal);


  double j1 = radian_to_tick1(J_goal.TH1);
  double j2 = radian_to_tick2(J_goal.TH2);
  ROS_INFO("tick = %f" , j1);
  ROS_INFO("tick = %f" , j2);
  set_dxl_goal(j1, j2);

  dxl_go();

  groupSyncWrite.clearParam();      // Clear syncwrite parameter storage
*/

  struct End_point EP_goal;
  EP_goal.x = 0;
  EP_goal.y = 0.15;

  read_dxl_position();

  struct End_point target;
  struct End_point present_position;

  present_position = get_present_posi();

  struct Joint J_goal;

  if (t<=T){
    // 1-cos으로 움직이기
    target.x = present_position.x + (EP_goal.x - present_position.x)*0.5*(1-cos(PI*t/T));
    target.y = present_position.y + (EP_goal.y - present_position.y)*0.5*(1-cos(PI*t/T));

    // target.x = 0.5*(cos(2*PI*t/T));   // 원으로 움직이기
    // target.y = 0.5*(sin(2*PI*t/T));

    J_goal = Compute_IK(target);      // 이동하는 순간 값으로

    t = t+dt;
  }

  set_dxl_goal(radian_to_tick1(J_goal.TH1), radian_to_tick2(J_goal.TH2));

  dxl_go();

  groupSyncWrite.clearParam();

  ROS_INFO("x = %lf, y = %lf", present_position.x, present_position.y);

  ROS_INFO("dxl1 = %d", dxl1_present_position);
  ROS_INFO("dxl2 = %d", dxl2_present_position);

}


void *p_function(void * data)
{

  dxl_initailize();

  dxl_add_param();

  static struct timespec next_time;
  //static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC,&next_time);
  // clock_gettime 자동완성이 뜨지 않는 경우 상단의 Help->About plugins->C++의 Clangcodemodel 체크 해제


  while(is_run)   // ()안에 1을 쓰게 되면 ctrl+c를 해도 종료가 잘 안됨 -> 전역변수 설정
  {
    next_time.tv_sec += (next_time.tv_nsec + Control_Cycle * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + Control_Cycle * 1000000) % 1000000000;

    process();

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_thread_node");
  ros::NodeHandle nh;

  pthread_t pthread;
  int thr_id;
  int status;
  char p1[] = "thread_1";

  sleep(1);

  // thread 생성
  thr_id = pthread_create(&pthread, NULL, p_function, (void*)p1);
  if(thr_id < 0)
  {
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

  is_run = false;   // 프로그램이 끝나기 전 is_run을 false로 바꾸고 p_function 종료
  return 0;
}



void dxl_initailize(void){  //open port, set baud, torque on dxl 1,2
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  // dxl_comm_result 주석처리하면 torque가 on되어도 움직임
}


void set_dxl_goal(int dxl_1_posi, int dxl_2_posi){
  // Allocate goal position value into byte array, dxl_1_posi의 바이트를 나눠서 채움
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_1_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_1_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_1_posi));

  // Add Dynamixel#1 goal position value to the Syncwrite storage
  // DXL1_ID와 목표 위치를 syncwrite 대상 dynamixel 목록에 저장함
  dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);

  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_2_posi));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_2_posi));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_2_posi));

  // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
  dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);

}


void dxl_go(void){
  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite.txPacket();
  // DXL1_ID, DXL2_ID에 동시에 명령을 전송하여 동일한 바이트를 나열된 주소에 씀
}


// 구조체 함수 정의
// 역기구학으로 theta1, theta2 구하기
struct Joint Compute_IK(struct End_point EP){       // radian -> 0~4095

  //IK soluttion
  double x = EP.x;
  double y = EP.y;
  double alpha = atan2(y,x);
  double L = sqrt(pow(x,2)+pow(y,2));
  double beta = acos((pow(L1,2)+pow(L2,2)-pow(L,2))/(2*L1*L2));
  double gamma = atan2(x,y);
  double delta = acos((pow(L1,2)+pow(L,2)-pow(L2,2))/(2*L1*L));

  double th2 = PI - beta;
  double th1 = (PI)/2 - gamma - delta;

  printf("%f , %f\n", th1, th2);

  struct Joint J;
  J.TH1 = th1;
  J.TH2 = th2;
  return J;
/*
  double x = EP.x;
  double y = EP.y;

  double c2 = (x*x + y*y - L1*L1 - L2*L2)/(2*L1*L2);
  double s2 = sqrt(1-c2*c2);
  double th2 = atan2(s2,c2);
  double th1 = atan2(y,x) - atan2(L1+L2*c2,L2*s2);

  printf("%f , %f\n",th1, th2);

  struct Joint J;
  J.TH1 = th1;
  J.TH2 = th2;
  return J;
  */
}

// theta1, theta2를 기구학 공식에 넣어서 x,y 좌표 구하기
struct End_point Compute_FK(struct Joint J){

  //FK solution
  double th1 = J.TH1;
  double th2 = J.TH2;

  double x = L1*cos(th1);
  double y = L1*sin(th1);

  struct End_point E;
  E.x = x + L2*cos(th1+th2);
  E.y = y + L2*sin(th1+th2);
  return E;
}

struct End_point get_present_posi(void){
  double th1 = tick_to_radian1(dxl1_present_position);
  double th2 = tick_to_radian2(dxl2_present_position);

  struct Joint J;
  J.TH1 = th1;
  J.TH2 = th2;

  struct End_point C;
  C = Compute_FK(J);
  return C;

}


// 라디안 각도 값을 모터 각도값으로 변환하는 함수
int radian_to_tick1(double radian){   // offset 필요
  return radian*(2048/PI) - 1023 + 4096;
}

int radian_to_tick2(double radian){
  return radian*(2048/PI);
}

// 읽어올 다이나믹셀 ID를 id list에 등록
void dxl_add_param(void){
  // Add Dynamixel goal position value to the Syncwrite storage
  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);

  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);

}

// 전역 변수 값을 업데이트, txRX, getData
void read_dxl_position(void){
  // Syncread present position
  dxl_comm_result = groupSyncRead.txRxPacket();

  // dxl_getdata_result = groupSyncRead.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  // dxl_getdata_result = groupSyncRead.isAvailable(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  // Get Dynamixel#1 present position value
  dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);


  // Get Dynamixel#2 present position value
  dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

}

double tick_to_radian1(int tick){
  double radian = (tick+1024)*(PI/(double)2048);
  return radian;
}

double tick_to_radian2(int tick){
  return tick*(PI/(double)2048);
}


#include "microstrain_3dmgx2_imu/imu_ros_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "microstrain_3dmgx2_node");   //初始化节点名。

  ros::NodeHandle nh;     //定义ros句柄。

  ImuNode in(nh);         //实例化ImuNode对象，此处调用ImuNode的构造函数。
  in.spin();              //循环接收数据，处理并发布。

  return(0);
}

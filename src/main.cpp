#include "LIVMapper.h"

// 【核心功能】ROS 节点入口，初始化并运行映射主循环
// 【输入/输出】输入: ROS 启动参数; 输出: 启动节点并阻塞运行
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  LIVMapper mapper(nh); 
  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}

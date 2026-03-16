#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

double g_waitting_time = 1;
double g_mov_dis = 1;
double g_turn_rad = 1;
double g_tar_vel = 1;
double g_ctl_dt = 0.02;
std::string g_test_item = "";

const std::vector<int> g_run_states = {0, 1, 0, 0, -1, 0}; // 0: straight, 1: turn right, -1 turn left


// void GetParameters(ros::NodeHandle n_private)
// {
//   n_private.param<double>("waitting_time", g_waitting_time, 1.0);
//   n_private.param<double>("mov_dis", g_mov_dis, 1.0);
//   n_private.param<double>("turn_rad", g_turn_rad, 1.0);
//   n_private.param<double>("tar_velocity", g_tar_vel, 1.0);
//   n_private.param<double>("ctl_time_step", g_ctl_dt, 0.02);
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "rover_driver");
//   ros::NodeHandle nh;
//   ros::NodeHandle nh_private("~");
//   GetParameters(nh_private);
//   ros::Publisher twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 0.1);
//   ros::Rate loop_rate(1/g_ctl_dt);
//   double tar_theta = 2.0*M_PI-2*atan2(g_mov_dis, g_turn_rad);
//   double turn_time = g_turn_rad*tar_theta/g_tar_vel;
//   double turn_w = g_tar_vel/g_turn_rad;

//   std::vector<double> cmd_v, cmd_w;
//   int c_move = g_mov_dis/g_tar_vel/g_ctl_dt;
//   int c_turn = turn_time/g_ctl_dt;
//   for (int i = 0; i < g_run_states.size(); i++)
//   {

//     if (g_run_states[i] == 0)
//     {
//       for (int j = 0; j < c_move; j++)
//       {
//         cmd_v.push_back(g_tar_vel);
//         cmd_w.push_back(0);
//       }
//     } 
//     else
//     {
//       for (int j = 0; j < c_turn; j++)
//       {
//         cmd_v.push_back(g_tar_vel);
//         cmd_w.push_back(turn_w*g_run_states[i]);
//       }
//     }
//   }
//   int c_wait = g_waitting_time/g_ctl_dt;
//   int counter = 0;
//   geometry_msgs::Twist twist_msg;
//   while (ros::ok())
//   {
//     int cmd_counter = counter - c_wait;
//     if (cmd_counter >= 0 && cmd_counter < cmd_v.size())
//     {
//       twist_msg.linear.x = cmd_v[cmd_counter];
//       twist_msg.angular.z = cmd_w[cmd_counter];
//     }
//     else
//     {
//       twist_msg.linear.x = 0;
//       twist_msg.angular.z = 0;
//     }
//     twist_pub_.publish(twist_msg);
//     counter++;
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
//   return 0;
// }

void GetParameters(std::shared_ptr<rclcpp::Node> node)
{
  node->declare_parameter<double>("waitting_time", 1.0);
  node->declare_parameter<double>("mov_dis", 1.0);
  node->declare_parameter<double>("turn_rad", 1.0);
  node->declare_parameter<double>("tar_velocity", 1.0);
  node->declare_parameter<double>("ctl_time_step", 0.02);

  node->get_parameter("waitting_time", g_waitting_time);
  node->get_parameter("mov_dis", g_mov_dis);
  node->get_parameter("turn_rad", g_turn_rad);
  node->get_parameter("tar_velocity", g_tar_vel);
  node->get_parameter("ctl_time_step", g_ctl_dt);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rover_driver");
  GetParameters(node);
  auto twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  double tar_theta = 2.0 * M_PI - 2 * atan2(g_mov_dis, g_turn_rad);
  double turn_time = g_turn_rad * tar_theta / g_tar_vel;
  double turn_w = g_tar_vel / g_turn_rad;

  std::vector<double> cmd_v, cmd_w;
  int c_move = g_mov_dis / g_tar_vel / g_ctl_dt;
  int c_turn = turn_time / g_ctl_dt;
  for (int i = 0; i < g_run_states.size(); i++)
  {
    if (g_run_states[i] == 0)
    {
      for (int j = 0; j < c_move; j++)
      {
        cmd_v.push_back(g_tar_vel);
        cmd_w.push_back(0);
      }
    }
    else
    {
      for (int j = 0; j < c_turn; j++)
      {
        cmd_v.push_back(g_tar_vel);
        cmd_w.push_back(turn_w * g_run_states[i]);
      }
    }
  }
  int c_wait = g_waitting_time / g_ctl_dt;
  int counter = 0;
  geometry_msgs::msg::Twist twist_msg;
  rclcpp::Rate loop_rate(1.0 / g_ctl_dt);

  while (rclcpp::ok())
  {
    int cmd_counter = counter - c_wait;
    if (cmd_counter >= 0 && cmd_counter < static_cast<int>(cmd_v.size()))
    {
      twist_msg.linear.x = cmd_v[cmd_counter];
      twist_msg.angular.z = cmd_w[cmd_counter];
    }
    else
    {
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0;
    }
    twist_pub_->publish(twist_msg);
    counter++;
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
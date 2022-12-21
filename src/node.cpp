#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "motion_smoother/macro.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class motion_smoother : public rclcpp::Node
{
  public:
  motion_smoother() : Node("motion_smoother")
  {
    this->declare_parameter<double>("gain" , 1.0);
    this->declare_parameter<bool>("linear.x_p", true);
    this->declare_parameter<bool>("linear.y_p", true);
    this->declare_parameter<bool>("linear.z_p", true);
    this->declare_parameter<bool>("angular.x_p", true);
    this->declare_parameter<bool>("angular.y_p", true);
    this->declare_parameter<bool>("angular.z_p", true);
    
    this->get_parameter("gain" , param_gain);
    this->get_parameter("linear.x_p" , linear.x_p);
    this->get_parameter("linear.y_p" , linear.y_p);
    this->get_parameter("linear.z_p" , linear.z_p);
    this->get_parameter("angular.x_p" , angular.x_p);
    this->get_parameter("angular.y_p" , angular.y_p);
    this->get_parameter("angular.z_p" , angular.z_p);

    sub_vel = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel/in" , 10 , std::bind(&motion_smoother::topic_callback , this , _1)
    );

    pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/out" , 10);

    timer = this->create_wall_timer(20ms , [this](){
      this->timer_callback();
    });

    target = RESET_TWIST;
    histry = RESET_TWIST;
  }

  private:

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
  rclcpp::TimerBase::SharedPtr timer;

  geometry_msgs::msg::Twist target;
  geometry_msgs::msg::Twist histry;

  double param_gain = 0.0;


  void
  topic_callback(const geometry_msgs::msg::Twist::SharedPtr get_msg){
    target.linear = get_msg->linear;
    target.angular = get_msg->angular;
  }

  void
  timer_callback(){
    auto rtVec = RESET_TWIST;
    double vec = 0;

    // linear x
    if(linear.x_p == true){
      vec = target.linear.x - histry.linear.x;
    vec = std::sqrt(vec * vec);
    if(vec > param_gain){
      if(target.linear.x > histry.linear.x){
        rtVec.linear.x = histry.linear.x + param_gain;
      }else{
        rtVec.linear.x = histry.linear.x - param_gain;
      }

    }else{
      rtVec.linear.x = target.linear.x;
    }

    histry.linear.x = rtVec.linear.x;

    }

    // linear y
    if(linear.y_p == true){
      vec = target.linear.y - histry.linear.y;
    vec = std::sqrt(vec * vec);
    if(vec > param_gain){
      if(target.linear.y > histry.linear.y){
        rtVec.linear.y = histry.linear.y + param_gain;
      }else{
        rtVec.linear.y = histry.linear.y - param_gain;
      }

    }else{
      rtVec.linear.y = target.linear.y;
    }
    histry.linear.y = rtVec.linear.y;

    }
    
    // linear z
    if(linear.z_p == true){
      vec = target.linear.z - histry.linear.z;
    vec = std::sqrt(vec * vec);
    if(vec > param_gain){
      if(target.linear.z > histry.linear.z){
        rtVec.linear.z = histry.linear.z + param_gain;
      }else{
        rtVec.linear.z = histry.linear.z - param_gain;
      }

    }else{
      rtVec.linear.z = target.linear.z;
    }
    histry.linear.z = rtVec.linear.z;
    }
    
    // angular x
    if(angular.x_p = true){
    vec = target.angular.x - histry.angular.x;
    vec = std::sqrt(vec * vec);
    if(vec > param_gain){
      if(target.angular.x > histry.angular.x){
        rtVec.angular.x = histry.angular.x + param_gain;
      }
      else{
        rtVec.angular.x = histry.angular.x - param_gain;
      }

    }else{
      rtVec.angular.x = target.angular.x;
    }
    histry.angular.x = rtVec.angular.x;
    }

    // angular y
    if(angular.y_p == true){
    vec = target.angular.y - histry.angular.y;
    vec = std::sqrt(vec * vec);
    if(vec > param_gain){
      if(target.angular.y > histry.angular.y){
        rtVec.angular.y = histry.angular.y + param_gain;
      }else{
        rtVec.angular.y = histry.angular.y - param_gain;
      }

    }else{
      rtVec.angular.y = target.angular.y;
    }
    histry.angular.y = rtVec.angular.y;
    }

    // angular z
    if(angular.z_p == true){
    vec = target.angular.z - histry.angular.z;
    vec = std::sqrt(vec * vec);
    if(vec > param_gain){
      if(target.angular.z > histry.angular.z){
        rtVec.angular.z = histry.angular.z + param_gain;
      }else{
        rtVec.angular.z = histry.angular.z - param_gain;
      }

    }else{
      rtVec.angular.z = target.angular.z;
    }
    histry.angular.z = rtVec.angular.z;
    }


    pub_vel->publish(rtVec);
  }
};

int main(int argc , char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<motion_smoother>());
    rclcpp::shutdown();
    return 0;
}

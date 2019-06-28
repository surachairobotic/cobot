#include "ros/ros.h"
#include "cobot_msgs/PickPlacePoseArray.h"

cobot_msgs::PickPlacePoseArray pppa;
bool b_chk = false;

void cb(const cobot_msgs::PickPlacePoseArray& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cobot_poses2paths");
  ros::NodeHandle n;

  try {
    ros::Subscriber sub_poses = n.subscribe("/cobot/image2pose/pickplace_array", 100, cb);
    ros::Rate loop_rate(50);
    while (ros::ok()) {
      if(b_chk) {
        for(int i=0; i<pppa.data.size(); i++) {
          if(pppa.data[i].b_pick && pppa.data[i].b_place) {
            ;
          }
          else if(pppa.data[i].b_pick && !pppa.data[i].b_place) {
            ;
          }
          else if(!pppa.data[i].b_pick && pppa.data[i].b_place) {
            ;
          }
          else
            ;
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  catch(int err) {
  }
  catch(const std::string &err) {
  }

  return 0;
}

void cb(const cobot_msgs::PickPlacePoseArray& msg) {
  if(!b_chk) {
    pppa = msg;
    b_chk = true;
  }
}

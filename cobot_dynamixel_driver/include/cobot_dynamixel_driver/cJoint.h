#ifndef __CJOINT_H__
#define __CJOINT_H__

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include <vector>


#define MODE_POSITION_CONTROL 3
#define MODE_VELOCITY_CONTROL 1
#define MODE_TORQUE_CONTROL 0

class cJoint{
private:
  int id, goal_pos, goal_velo, goal_torque;
  int pos, velo, current, input_voltage, temperature, load;
  double rad2val, velo2val;
  std::string name;
  
  // xml
  std::string motor_name;
  int motor_model_number;
  double cw_angle_limit, ccw_angle_limit, torque_limit, velocity_limit, acceleration_limit
  , gear_ratio, position_value, current_max;

  static dynamixel::PacketHandler *packetHandler;
  static dynamixel::PortHandler *portHandler;
  static dynamixel::GroupSyncWrite *group_write_velo, *group_write_pos_velo;
  static dynamixel::GroupSyncRead *group_read;
  static std::vector<cJoint> joints;
  static int mode;
  

private:
  void write(const int param, const int val);
  int read(const int param);
  void setup();
/*  void write1b(int addr, int val);
  void write2b(int addr, int val);
  int read1b(int addr);
  int read2b(int addr);*/

  static void load_settings();

public:
  cJoint();
  cJoint(int _id);
  void set_goal_velo(double rad_per_sec);
  void set_goal_pos_velo(double _pos, double _velo);
  void print_data() const;

  inline int get_id() const { return id; }
  inline const std::string& get_name() const { return name; }
  double get_pos() const;
  double get_velo() const;
  double get_current() const; // mA
  double get_load() const;    // current / max_current
  
  
  static std::vector<cJoint> &init();
  static void sync_velo();
  static void sync_pos_velo();
  static void sync_read();
  static void change_mode(int _mode);
  static void terminate();
  static bool is_all_reaching_pos();
  inline static std::vector<cJoint>& get_joints(){ return joints; }
  inline static cJoint* get_joint(std::string _name){
    for(int i=joints.size()-1;i>=0;i--){
      if( joints[i].name==_name )
        return &joints[i];
    }
    //ROS_ERROR("invalid joint name : %s\n", _name.c_str());
    return NULL;
  }
  
};

#endif

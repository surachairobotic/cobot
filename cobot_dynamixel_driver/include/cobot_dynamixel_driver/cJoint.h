#ifndef __CJOINT_H__
#define __CJOINT_H__

#include "cobot_dynamixel_driver/common.h"
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include <vector>


#define MODE_POSITION_CONTROL 3
#define MODE_VELOCITY_CONTROL 1
#define MODE_TORQUE_CONTROL 0


enum eADDR{
  P_MODEL_NUMBER,
  P_ID,
  P_OPERATING_MODE,
  P_RETURN_DELAY_TIME,
  P_CW_ANGLE_LIMIT,
  P_CCW_ANGLE_LIMIT,
  P_VELOCITY_LIMIT,
  P_ACCELERATION_LIMIT,
  P_MAX_TORQUE,
  P_TORQUE_ENABLE,
  P_GOAL_POSITION,
  P_GOAL_VELOCITY,
  P_GOAL_ACCELERATION,
  P_GOAL_TORQUE,
  P_TORQUE_LIMIT,
  P_PRESENT_POSITION,
  P_PRESENT_VELOCITY,
  P_PRESENT_CURRENT,
  P_PRESENT_LOAD,
  P_PRESENT_INPUT_VOLTAGE,
  P_PRESENT_TEMPERATURE,
  P_TORQUE_CONTROL_MODE
};


class cJoint{
private:
  int id, goal_pos, goal_velo, goal_torque;
  int pos, velo, current, input_voltage, temperature, load;
  double rad2val, velo2val, acc2val;
  std::string name;
  
  // xml
  std::string motor_name;
  int motor_model_number;
  double cw_angle_limit, ccw_angle_limit, torque_limit, velocity_limit, acceleration_limit
  , gear_ratio, position_value, current_max;

  static dynamixel::PacketHandler *packetHandler;
  static dynamixel::PortHandler *portHandler;
  static dynamixel::GroupSyncWrite *group_write_velo, *group_write_pos_velo;
  static dynamixel::GroupSyncRead *group_read_sync;
  static dynamixel::GroupBulkRead *group_read_bulk;
  static std::vector<cJoint> joints;
  static int mode;
  static int ADDR[32][2];

private:
  void write(const int param, const int val);
  int read(const int param);
  void setup();
/*  void write1b(int addr, int val);
  void write2b(int addr, int val);
  int read1b(int addr);
  int read2b(int addr);*/

  static void load_settings(const char *xml_file);

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
  static bool is_all_reaching_goal_pos();
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



class TiXmlNode;

double mystof(const std::string &str);
std::string get_attr(TiXmlNode *parent, const char *child_name, const char *attr);

#endif

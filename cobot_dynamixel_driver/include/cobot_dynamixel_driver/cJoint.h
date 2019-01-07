#ifndef __CJOINT_H__
#define __CJOINT_H__

#include "cobot_dynamixel_driver/common.h"
#include "dynamixel_sdk/dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include <vector>


#define MODE_POSITION_CONTROL 3
#define MODE_VELOCITY_CONTROL 1
#define MODE_TORQUE_CONTROL 0


enum eADDR{
  P_MODEL_NUMBER,
  P_MODEL_INFORMATION,
  P_FIRMWARE_VERSION,
  P_ID,
  P_BAUD_RATE,
  P_RETURN_DELAY_TIME,
  P_OPERATING_MODE,
  P_HOMING_OFFSET,
  P_MOVING_THRESHOLD,
  P_TEMPERATURE_LIMIT,
  P_MAX_VOLTAGE_LIMIT,
  P_MIN_VOLTAGE_LIMIT,
  P_ACCELERATION_LIMIT,
  P_TORQUE_LIMIT,
  P_VELOCITY_LIMIT,
  P_MAX_POSITION_LIMIT,
  P_MIN_POSITION_LIMIT,
  P_EXTERNAL_PORT_MODE_1,
  P_EXTERNAL_PORT_MODE_2,
  P_EXTERNAL_PORT_MODE_3,
  P_EXTERNAL_PORT_MODE_4,
	P_SHUTDOWN,
	
  P_TORQUE_ENABLE,
  P_LED_RED,
  P_LED_GREEN,
  P_LED_BLUE,
	P_VELOCITY_I_GAIN,
	P_VELOCITY_P_GAIN,
	P_POSITION_P_GAIN,
  P_GOAL_POSITION,
  P_GOAL_VELOCITY,
  P_GOAL_TORQUE,
  P_GOAL_ACCELERATION,
  P_CW_ANGLE_LIMIT,
  P_CCW_ANGLE_LIMIT,
  P_MAX_TORQUE,
  P_PRESENT_POSITION,
  P_PRESENT_VELOCITY,
  P_PRESENT_CURRENT,
  P_PRESENT_INPUT_VOLTAGE,
  P_PRESENT_TEMPERATURE,
  P_PRESENT_LOAD,
  P_TORQUE_CONTROL_MODE,
	P_HARDWARE_ERROR_STATUS
};


class cJoint{
private:
  int mode;
  int id, goal_pos, goal_velo, goal_acc, goal_torque;
  int pos, velo, current, input_voltage, temperature, load;
  double rad2val, velo2val, acc2val;
  std::string name;
  bool b_goal_velo, b_goal_velo_acc, b_goal_pos_velo, b_goal_pos_velo_acc, b_goal_torque;
  
  // xml
  std::string motor_name;
  int motor_model_number;
  double cw_angle_limit, ccw_angle_limit, torque_limit, velocity_limit, acceleration_limit
  , gear_ratio, position_value, current_max, direction;

  static dynamixel::PacketHandler *packetHandler;
  static dynamixel::PortHandler *portHandler;
  static dynamixel::GroupSyncWrite *group_write_velo, *group_write_velo_acc, *group_write_pos_velo, *group_write_pos_velo_acc, *group_write_acc, *group_write_gain_p, *group_write_goal_torque;
  static dynamixel::GroupSyncRead *group_read_sync;
  static dynamixel::GroupBulkRead *group_read_bulk;
  static std::vector<cJoint> joints;
  static int ADDR[44][2];
  static std::vector<std::string> joint_names;
  static std::string setting_file;
  static bool b_set_home;

private:
  void write(const int param, const int val);
  int read(const int param);
  void setup();
/*  void write1b(int addr, int val);
  void write2b(int addr, int val);
  int read1b(int addr);
  int read2b(int addr);*/

  static void load_settings(const std::string &xml_file);

public:
  cJoint();
  cJoint(int _id);
	bool set_goal_torque(double torque);
  bool set_goal_velo(double rad_per_sec);
  bool set_goal_velo_acc(double _velo, double _acc);
  bool set_goal_pos_velo(double _pos, double _velo);
  bool set_goal_pos_velo_acc(double _pos, double _velo, double _acc);
  bool set_acc(double _acc);
  bool set_p_gain(int _p_gain);
  int get_p_gain();
  bool set_i_gain(int _i_gain);
  int get_i_gain();
	bool send_p_gain(int _p_gain);
  void print_data() const;
  void change_mode(int _mode);

  inline int get_id() const { return id; }
  inline const std::string& get_name() const { return name; }
  double get_pos() const;
  double get_velo() const;
  double get_current() const; // mA
  double get_load() const;    // current / max_current
  double get_info();
  
  static std::vector<cJoint> &init();
	static void sync_torque();
  static void sync_velo();
  static void sync_velo_acc();
  static void sync_pos_velo();
  static void sync_pos_velo_acc();
  static bool sync_read();
  static void terminate();
  static bool is_all_reaching_goal_pos();
  static void load_joint_name();
  static std::string get_joint_name(int id);
  inline static void set_setting_file(const std::string &name){ setting_file = name; }
  inline static std::vector<cJoint>& get_joints(){ return joints; }
  inline static cJoint* get_joint(std::string _name){
    for(int i=joints.size()-1;i>=0;i--){
      if( joints[i].name==_name )
        return &joints[i];
    }
    //ROS_ERROR("invalid joint name : %s\n", _name.c_str());
    return NULL;
  }
  static void set_home(){ b_set_home = true; }
  static void reset_goal();
  
};



class TiXmlNode;

double mystof(const std::string &str);
std::string get_attr(TiXmlNode *parent, const char *child_name, const char *attr);

#endif

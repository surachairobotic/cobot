clear all;
close all;

s = load('../results/move_joint.txt');
er_s = load('../results/move_joint_err.txt');

t = s(:,1);
p_wp = s(:,2:7);
v_wp = s(:,8:13);
p_jnt = s(:,14:19);
v_jnt = s(:,20:25);
v_cal = s(:,26:31);

er_t = er_s(:,1);
#er_j(1) = er_s(:,2);
#er_j(2) = er_s(:,3);
#er_j(3) = er_s(:,4);
#er_j(4) = er_s(:,5);
#er_j(5) = er_s(:,6);
#er_j(6) = er_s(:,7);

tt = t;
tt(1) = [];
vv_jnt = v_jnt;
vv_jnt(1,:) = [];
vv_cal = v_cal;
vv_cal(end,:) = [];
vv_wp = v_wp;
vv_wp(end,:) = [];

i=6
#for i=1:6
  figure(i);
  subplot(3,1,1);
  plot(t, p_wp(:,i), t, p_jnt(:,i), t, p_wp(:,i), 'o', t, p_jnt(:,i), '*');
  grid on;
  title(['Position ( J' num2str(i) ' )']);
  ylabel('angle(rad)');
  xlabel('time(s)');
  legend("waypoint", "move");

  subplot(3,1,2);
  plot(t, v_wp(:,i), t, v_jnt(:,i), t, v_cal(:,i), t, v_wp(:,i), 'o', t, v_jnt(:,i), '*', t, v_cal(:,i), '*');
#  plot(tt, v_jnt(:,i), t, v_cal(:,i), tt, v_jnt(:,i), '*', t, v_cal(:,i), '*');
  grid on;
  title(['Velocity ( J' num2str(i) ' )']);
  ylabel('vel(rad/s^2)');
  xlabel('time(s)');
  legend("waypoint", "move", "calculate");

  subplot(3,1,3);
  plot(er_t, er_s(:,i), er_t, er_s(:,i), 'x');  
#  plot(tt, vv_wp(:,i), tt, vv_jnt(:,i), tt, vv_cal(:,i), tt, vv_wp(:,i), 'o', tt, vv_jnt(:,i), '*', tt, vv_cal(:,i), '*');
  grid on;
#  title(['Velocity shift ( J' num2str(i) ' )']);
#  ylabel('vel(rad/s^2)');
#  xlabel('time(s)');
#  legend("waypoint", "move", "calculate");  
  title(['Position Error ( J' num2str(i) ' )']);
  ylabel('err(rad)');
  xlabel('time(s)');

#end

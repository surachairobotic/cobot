clear all;
close all;

s = load('../results/move_joint.txt');

t = s(:,1);
p_wp = s(:,2:7);
v_wp = s(:,8:13);
p_jnt = s(:,14:19);
v_jnt = s(:,20:25);
v_cal = s(:,26:31);

tt = t;
tt(1) = [];
vv_jnt = v_jnt;
vv_jnt(1,:) = [];
vv_cal = v_cal;
vv_cal(end,:) = [];
vv_wp = v_wp;
vv_wp(end,:) = [];

for i=1:6
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
  plot(tt, vv_wp(:,i), tt, vv_jnt(:,i), tt, vv_cal(:,i), tt, vv_wp(:,i), 'o', tt, vv_jnt(:,i), '*', tt, vv_cal(:,i), '*');
  grid on;
  title(['Velocity shift ( J' num2str(i) ' )']);
  ylabel('vel(rad/s^2)');
  xlabel('time(s)');
  legend("waypoint", "move", "calculate");  
end

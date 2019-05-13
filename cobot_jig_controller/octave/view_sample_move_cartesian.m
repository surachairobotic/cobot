clear all;
#close all;

s = load('../results/2019-04-19T10:22:36.480343move_cartesian.txt');

t_wp = s(:,1);
t_mv = s(:,2);
p_wp = s(:,3:5);
p_mv = s(:,6:8);
v_wp = s(:,9:11);
v_mv = s(:,12:14);

figure(10);
subplot(3,1,1);
plot(t_mv, p_wp(:,1), t_mv, p_mv(:,1), t_mv, p_wp(:,1), '*', t_mv, p_mv(:,1), '*');
grid on;
title('Position ( X )');
ylabel('axis X (m)');
xlabel('time(s)');
legend("waypoint","move");

subplot(3,1,2);
plot(t_mv, p_wp(:,2), t_mv, p_mv(:,2), t_mv, p_wp(:,2), '*', t_mv, p_mv(:,2), '*');
grid on;
title('Position ( Y )');
ylabel('axis Y (m)');
xlabel('time(s)');
legend("waypoint","move");

subplot(3,1,3);
plot(t_mv, p_wp(:,3), t_mv, p_mv(:,3), t_mv, p_wp(:,3), '*', t_mv, p_mv(:,3), '*');
grid on;
title('Position ( Z )');
ylabel('axis Z (m)');
xlabel('time(s)');
legend("waypoint","move");

###########################################

figure(11);
subplot(3,1,1);
plot(t_wp, v_wp(:,1), t_mv, v_mv(:,1), t_wp, v_wp(:,1), '*', t_mv, v_mv(:,1), '*');
grid on;
title('Velocity ( X )');
ylabel('axis X (m)');
xlabel('time(s)');
legend("waypoint","move");

subplot(3,1,2);
plot(t_wp, v_wp(:,2), t_mv, v_mv(:,2), t_wp, v_wp(:,2), '*', t_mv, v_mv(:,2), '*');
grid on;
title('Velocity ( Y )');
ylabel('axis Y (m)');
xlabel('time(s)');
legend("waypoint","move");

subplot(3,1,3);
plot(t_wp, v_wp(:,3), t_mv, v_mv(:,3), t_wp, v_wp(:,3), '*', t_mv, v_mv(:,3), '*');
grid on;
title('Velocity ( Z )');
ylabel('axis Z (m)');
xlabel('time(s)');
legend("waypoint","move");

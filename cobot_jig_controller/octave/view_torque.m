clear all;
close all;

file = '../results/torque_j5_-30.txt';

s = load(file);
t = s(:,1);
tq1 = s(:,2);
tq2 = s(:,5);
pos = s(:,3);
vel = s(:,4);

figure(1);
subplot(2,1,1);
plot(t, tq1, '.', t, tq2);
subplot(2,1,2);
plot(t, pos, t, vel);

#figure(1);
#subplot(2,1,2);
#plot(tq2);

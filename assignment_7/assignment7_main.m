
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")

%Call the first assignment to compute the Jacobians and the transformation
%matrix between the two
assign1withoutPrints; clc;
assign3withoutPrints;
clearvars -except Ja Jg Ta T_phi B C G Dq %remove all the other variables

q_d = [0 0.1   0.15  ;    %joint1
       0 pi/3  pi/6  ;    %joint2
       0 0.05  0.10 ];   %joint3

q_d_const = [0.1 ;    %joint1
             pi/3;    %joint2
             -0.1];   %joint3

Kd = diag([40, 30, 40]);



%% Load "myRobot" struct
loadMyRobotStruct; %here are set the initial conditions

%Load and open the Simulink system
open_system('jointSpaceInverseDynamicsControl');
load_system('jointSpaceInverseDynamicsControl')
model = 'jointSpaceInverseDynamicsControl';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);


%To see the differences in a torque increase:
Kp = diag([1000, 900, 1000]); %increase the Kp values to see the overshoot

set_param('jointSpaceInverseDynamicsControl/Manual Switch1','sw', '1')
out_sat = sim(in); %get high saturated torques simulation results
set_param('jointSpaceInverseDynamicsControl/Manual Switch1','sw', '0')
out_notSat = sim(in); %get high NON saturated torques simulation results

set_param('jointSpaceInverseDynamicsControl/Manual Switch1','sw', '1')
Kp = diag([300, 200, 300]);
out_sat_lowKp = sim(in); %get low saturated torques simulation results

%Plot the results
figure; tiledlayout(3,1); titles = {'d1(t)', 'theta2(t)', 'd3(t)'};
for i=1:3
    ax = nexttile;    
    plot(out_sat.posComparison{i}.Values.Time, out_sat.posComparison{i}.Values.Data(:,1), 'LineWidth',2);
    hold on    
    plot(out_notSat.posComparison{i}.Values.Time, out_notSat.posComparison{i}.Values.Data(:,1), 'LineWidth',2);
    hold on    
    plot(out_sat_lowKp.posComparison{i}.Values.Time, out_sat_lowKp.posComparison{i}.Values.Data, 'LineWidth',2);
    title(ax, titles{i});
    legend('position - HIGH Kp, SATURATED', 'position - HIGH Kp, NOT SATURATED', ...
           'position - LOW Kp, SATURATED=NOT SATURATED', 'position - reference signal');
end

%Show the start, intermediate, and goal configurations on a plot 
startConf = homeConfiguration(robot);
for i=1:3 startConf(i).JointPosition = q_d(i,1); end
endConf_1 = homeConfiguration(robot);
for i=1:3 endConf_1(i).JointPosition = q_d(i,2); end
endConf_2 = homeConfiguration(robot);
for i=1:3 endConf_2(i).JointPosition = q_d(i,3); end

figure; subplot(1,3,1); show(robot,startConf); title("Starting configuration (" + mat2str(round([startConf.JointPosition],3)) +")");
subplot(1,3,2); show(robot, endConf_1); title("Goal configuration 1 (" + mat2str(round([endConf_1.JointPosition],3)) +")");
subplot(1,3,3); show(robot, endConf_2); title("Goal configuration 2 (" + mat2str(round([endConf_2.JointPosition],3)) +")");




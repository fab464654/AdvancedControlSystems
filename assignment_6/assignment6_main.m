
clear; clc; close all;

%Add needed functions
addpath("../myFunctions/")

%Call the first assignment to compute the Jacobians and the transformation
%matrix between the two
assign1withoutPrints; clc;
assign3withoutPrints;
clearvars -except Ja Jg Ta T_phi B C G Dq %remove all the other variables

%Set the desired constant position
q_d = [0.1 pi/3 0.1].';

% q_d = [0 0 0].';

%% Set the proportional and derivative matrices
Kp = diag([40, 30, 80]);
Kd = diag([15, 5, 15]);

%%
%Load "myRobot" struct
loadMyRobotStruct;

%Load and open the Simulink system
open_system('jointSpacePDControl');
load_system('jointSpacePDControl')
model = 'jointSpacePDControl';

%Run the simulation and get the outputs
in = Simulink.SimulationInput(model);

set_param('jointSpacePDControl', 'StopTime', '5') %set simulation time
set_param('jointSpacePDControl/setOption','Value', '1')
out1 = sim(in); %get g(q(t)) simulation results
set_param('jointSpacePDControl/setOption','Value', '3')
out3 = sim(in); %get g(q_d) simulation results

figure; tiledlayout(3,1); titles = {'d1(t)', 'theta2(t)', 'd3(t)'};
for i=1:3
    ax = nexttile;    
    plot(out1.posComparison{i}.Values.Time, out1.posComparison{i}.Values.Data(:,1), 'LineWidth',2);
    hold on    
    plot(out3.posComparison{i}.Values.Time, out3.posComparison{i}.Values.Data, 'LineWidth',2);
    title(ax, titles{i});
    legend('position - g(q(t)) compensation', 'position - g(q_d) compensation', ...
           'reference signal');
end

%Show the start and goal configurations on a plot 
startConf = homeConfiguration(robot);
for i=1:3 startConf(i).JointPosition = myRobot.q0(i); end
endConf = homeConfiguration(robot);
for i=1:3 endConf(i).JointPosition = q_d(i); end
figure; subplot(1,2,1); show(robot,startConf); title("Starting configuration (" + mat2str(round([startConf.JointPosition],3)) +")");
subplot(1,2,2); show(robot, endConf); %show the robot in the "desired joint configuration"
title("Goal configuration (" + mat2str(round([endConf.JointPosition],3)) +")");


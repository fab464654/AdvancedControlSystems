clc; clear; close all;


%Add needed functions
addpath("../myFunctions/")

assign1withoutPrints; clc; %change to "base", all 3D plots are wrt base
clearvars -except T_all


%number of ref. frames
numFrames = size(T_all, 3);

%Give the standard joint positions (or change them)
loadMyRobotStruct;
config = homeConfiguration(robot);
show(robot,config);

T_all_with_base(:,:,1) = sym(eye(4)); %to add the base ref. frame
T_all_with_base(:,:,2:size(T_all,3)+1) = T_all;

T_all_numeric = double(subs(T_all_with_base, [L0,L2,L3,L4,d1(t),theta2(t),d3(t)], [0.4,0.4,0.3,0.4,[config.JointPosition]])); %wrt ∑0
clearvars -except T_all_numeric T_all
numFrames = size(T_all_numeric,3); 


%% Make the plot
f = figure('Units','normalized','OuterPosition',[0 .5 1 .5]);

for k = 1:numFrames    
    ax = subplot(1,numFrames,k);
       
    translation = T_all_numeric(1:3,4,k).';
    quaternion  = rotm2quat(T_all_numeric(1:3,1:3,k));
    plotTransforms(translation, quaternion, 'FrameSize',1)
   
    if k==1
        title("Rerence frame Σbase")
    else
        title("Rerence frame Σ"+num2str(k-2))
    end
    view(ax,30,30);
end

    
    
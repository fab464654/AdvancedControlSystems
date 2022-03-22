function [sys,x0,str,ts] = manipulator_1DOF(t,x,u,flag,myRobot_1DOF)
%CSFUNC An example MATLAB file S-function for defining a continuous system.  


%-------------------------------------------------------------------------
%My modification: to have a better Simulink visualization I modified the
%output vector in order to have [Dq q] instead of [q Dq].
%-------------------------------------------------------------------------

    switch flag,

      %%%%%%%%%%%%%%%%%%
      % Initialization %
      %%%%%%%%%%%%%%%%%%
      case 0,
        [sys,x0,str,ts]=mdlInitializeSizes(myRobot_1DOF);

      %%%%%%%%%%%%%%%
      % Derivatives %
      %%%%%%%%%%%%%%%
      case 1,
        sys=mdlDerivatives(t,x,u,myRobot_1DOF);

      %%%%%%%%%%%
      % Outputs %
      %%%%%%%%%%%
      case 3,
        sys=mdlOutputs(t,x,u);

      %%%%%%%%%%%%%%%%%%%
      % Unhandled flags %
      %%%%%%%%%%%%%%%%%%%
      case { 2, 4, 9 },
        sys = [];

      %%%%%%%%%%%%%%%%%%%%
      % Unexpected flags %
      %%%%%%%%%%%%%%%%%%%%
      otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

    end
    % end csfunc

    %
    %=============================================================================
    % mdlInitializeSizes
    % Return the sizes, initial conditions, and sample times for the S-function.
    %=============================================================================
    %
    function [sys,x0,str,ts]=mdlInitializeSizes(myRobot_1DOF)
        sizes = simsizes;
        sizes.NumContStates  = 2*myRobot_1DOF.dof; 
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = 2*myRobot_1DOF.dof;
        sizes.NumInputs      = myRobot_1DOF.dof;
        sizes.DirFeedthrough = 0;
        sizes.NumSampleTimes = 1;

        sys = simsizes(sizes);  %Dx (x_dot)  
        x0  = [myRobot_1DOF.Dq0; myRobot_1DOF.q0]; %my notation: [Dq q]
        
        str = [];
        ts  = [0 0];
    end
    % end mdlInitializeSizes
    %
    %=============================================================================
    % mdlDerivatives
    % Return the derivatives for the continuous states.
    %=============================================================================
    %
    function sys=mdlDerivatives(t,x,u,myRobot_1DOF)

        Dq = x(1:myRobot_1DOF.dof);                  %my notation: [Dq q]
        q = x((myRobot_1DOF.dof+1):(2*myRobot_1DOF.dof)); %my notation: [Dq q]
        
        B = myRobot_1DOF.I;
        F = myRobot_1DOF.F;
        G = myRobot_1DOF.G;
        
        %"Normal" myRobot_1DOF modeling 
        sys(1 : myRobot_1DOF.dof) = -inv(B)*(F*Dq + G*sin(q) - u); %my notation: [Dq q]
        
        %Uncomment to test the system with a "wrong plant modeling"
%         % 1) actually does almost nothing
%         F2 = 10;
%         sys(1 : myRobot_1DOF.dof) = -inv(B)*(F*Dq + F2*Dq^2 + G*sin(q) - u); %my notation: [Dq q]
        
        % 2) more evident position mismatch
%         F2 = 10;
%         G2 = 3; %just to change further the dynamic model
%         sys(1 : myRobot_1DOF.dof) = -inv(B)*(F*Dq + F2*Dq^2 + G*sin(q) + G2*cos(q) - u); %my notation: [Dq q]

        sys((myRobot_1DOF.dof+1):(2*myRobot_1DOF.dof)) = Dq;       %my notation: [Dq q]
    end
    % end mdlDerivatives
    %
    %=============================================================================
    % mdlOutputs
    % Return the block outputs.
    %=============================================================================
    %
    function sys=mdlOutputs(t,x,u)
        sys = x;
    end

    % end mdlOutputs
end
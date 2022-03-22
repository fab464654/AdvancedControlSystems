function [sys,x0,str,ts] = gravityTerm(t,x,u,flag,myRobot)
%CSFUNC An example MATLAB file S-function for defining a continuous system.  
%   Example MATLAB file S-function implementing continuous equations: 
%      x' = Ax + Bu
%      y  = Cx + Du
%   
%   See sfuntmpl.m for a general S-function template.
%
%   See also SFUNTMPL.
  
%   Copyright 1990-2009 The MathWorks, Inc.


    switch flag,

      %%%%%%%%%%%%%%%%%%
      % Initialization %
      %%%%%%%%%%%%%%%%%%
      case 0,
        [sys,x0,str,ts]=mdlInitializeSizes(myRobot);

      %%%%%%%%%%%%%%%
      % Derivatives %
      %%%%%%%%%%%%%%%
      case 1,
        sys=mdlDerivatives(t,x,u,myRobot);

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
    function [sys,x0,str,ts]=mdlInitializeSizes(myRobot)

        sizes = simsizes;
        sizes.NumContStates  = myRobot.dof;
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = myRobot.dof;
        sizes.NumInputs      = myRobot.dof;
        sizes.DirFeedthrough = 0;
        sizes.NumSampleTimes = 1;

        sys = simsizes(sizes);  %Dx (x_dot)      
        x0  = [myRobot.q0];
        
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
    function sys=mdlDerivatives(t,x,u,myRobot)

        q = x(1:myRobot.dof);
        G = get_G_matrix(q);
        sys(1 : myRobot.dof) = G;
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
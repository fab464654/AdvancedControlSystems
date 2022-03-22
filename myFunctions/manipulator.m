function [sys,x0,str,ts] = manipulator(t,x,u,flag,myRobot)
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
        sizes.NumContStates  = 2*myRobot.dof; 
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = 2*myRobot.dof;
        sizes.NumInputs      = myRobot.dof;
        sizes.DirFeedthrough = 0;
        sizes.NumSampleTimes = 1;

        sys = simsizes(sizes);  %Dx (x_dot)      
%         x0  = [myRobot.q0; myRobot.Dq0];
        x0  = [myRobot.Dq0; myRobot.q0]; %my notation: [Dq q]
        
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

%         q = x(1:myRobot.dof);
%         Dq = x((myRobot.dof+1):(2*myRobot.dof));

        Dq = x(1:myRobot.dof);                  %my notation: [Dq q]
        q = x((myRobot.dof+1):(2*myRobot.dof)); %my notation: [Dq q]
        
        B = get_B_matrix(q);
        C = get_C_matrix(q,Dq);
        G = get_G_matrix(q);

%         sys(1 : myRobot.dof) = Dq;
%         sys((myRobot.dof+1):(2*myRobot.dof)) = -inv(B)*(C*Dq + G - u);
        sys(1 : myRobot.dof) = -inv(B)*(C*Dq + G - u); %my notation: [Dq q]
        sys((myRobot.dof+1):(2*myRobot.dof)) = Dq;     %my notation: [Dq q]
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
% For given reference trajectory of a 7-link biped in 2D, this function
% computes the corresponding target CoG trajectory and the optimal ZMP 
% control inputs. The reference trajectory is defined in
% 'ComputeKinematicTrajectories.m'. This function also plots out various
% computed trajectories.
% 
% INPUT:    p -- vector of system parameters;
% OUTPUT:   tPLAN    -- target time trajectory;
%           xF_lPLAN -- horizontal location of the left foot;
%           yF_lPLAN -- vertical location of the left foot;
%           xF_rPLAN -- horizontal location of the right foot;
%           yF_rPLAN -- vertical location of the right foot;
%           xCOGPLAN -- target x trajectory for the CoG;
%           yCOGPLAN -- target y trajectory for the CoG.
%           Zref -- desired location of the ZMP.
%           Z    -- actual location of the ZMP.
% 
function [tPLAN, xF_lPLAN, yF_lPLAN, xF_rPLAN, yF_rPLAN, xCoGPLAN, yCoGPLAN, Zref, Z] = PlanZMPTrajectories()
    
    g = 1;               % Normalized
    % We assume a crouched position in which the hip is at a hight of
    % y=0.85. The overall CoG) is located approximately at the same height.
    hCOM = 0.85;         % Normalized
    RoverQ = 1e-6;       % Relative weight of tracking ZMP vs. minimizing dddx
    T = 0.05;            % Time step
    N = 501;             % # of time steps

    % Set up matrices of the ZMP dynamics:
    A = [1, T, T^2/2;
         0, 1,     T;
         0, 0,     1];
    B = [T^3/6;T^2/2;T];
    C = [1,0,-hCOM/g];

    % Timing vector:
    tPLAN = 0:T:(N-1)*T;
    % Prepare outputs:
    Zref = zeros(N,1);
    xF_lPLAN = zeros(1,N);
    yF_lPLAN = zeros(1,N);
    xF_rPLAN = zeros(1,N);
    yF_rPLAN = zeros(1,N);
    xCoGPLAN = zeros(1,N);
    yCoGPLAN = zeros(1,N);
    % Compute trajectories of the feet and the ZMP.  To this end, we use
    % the same function "ComputeKinematicTrajectories(tZMP(i))" that was
    % introduced in the static locomotion case.  Here, we can use more
    % agressive parameters in terms of stride configuration (longer, faster
    % strides) and we will NOT use the motion of the main body later.
    % Instead, the position of the main body (which, in the static
    % locomotion case is positioned straight above the center of the stance
    % foot and transitions to the next foot during double stance) will be
    % used as the target for the ZMP.  The original CoG trajectories
    % (xM,yM) will be discarded.
    for i =1:N
        [xF_lPLAN(i), yF_lPLAN(i), xF_rPLAN(i), yF_rPLAN(i), xM, yM] = ComputeKinematicTrajectories(tPLAN(i));
        Zref(i) = xM;
    end

    % Set up transition matrices for all steps, such that
    %   Z = Px*x0 + Pu*dddX;
    Px   = zeros(N,3);
    Pu   = zeros(N,N);
    AtoN = eye(3);
    for i = 1:N
        Pu(i,1) = C*AtoN*B;
        for j = 2:N
            if i>=j
                Pu(i,j) = Pu(i-1,j-1);
            end
        end
        AtoN = AtoN*A;
        Px(i,:) = C*AtoN;
    end

    % Solve for the optimal control inputs:
    x0 = [-0.4;0.3;0];
    dddX = -(Pu.'*Pu+RoverQ*ones(N,N))\Pu.'*(Px*x0-Zref);

    % Do a discrete simulation to compute the state X and the ZMP location
    % Z as a function of time 
    X = zeros(3,N);
    Z = zeros(1,N);
    X(:,1) = x0;
    Z(1)   = C*X(:,1);
    xCoGPLAN(1) = X(1,1);
    yCoGPLAN(1) = hCOM;
    for k = 2:N
        X(:,k) = A*X(:,k-1) + B*dddX(k-1);
        Z(k)   = C*X(:,k);
        xCoGPLAN(k) = X(1,k);
        yCoGPLAN(k) = hCOM;
    end
end



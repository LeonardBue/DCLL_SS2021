% This function computes and plots the desired constraints for the model's
% actuated joints, their rates and accelerations. The constraints are
% defined in 'targetEvolution.m'.
% 
function VisualizeConstraints() 
    %% Stet up the path:
    % Define the controller:
    controllerName = 'HZD_incomplete'; 
    % Move one folder level up and include the library folders
    % "Library\Shared", as well as the desired model and controller folders
    % from "Library\Controllers" and "Library\Models" into the path.  Make
    % sure that you downloaded these from the lecture pages and that your
    % local folder structure matches the structure given there. 
    currentDir = pwd;
    cd('..');
    if (~exist('Library','dir'))
        error('Could not include all necessary library files. Please download the library folder and make sure that you run this file from the lecture folder')
    end
    % Reset the MATLAB search path to its default value:
    path(pathdef);
    % Add to the path all shared files:
    if isunix()
       path(genpath(['Library/Controllers/',controllerName]), path);
    else
        path(genpath(['Library\Controllers\',controllerName]), path);
    end
    % Go back to original directory:
    cd(currentDir);
    
    %% Actual plotting
    % Create a grid in the phase variable theta
    set(gca,'Fontsize',20);
    n = 200;
    theta = linspace(-0.3,0.3,n);

    % Initialize the constraint vectors:
    hD        = zeros(4,n);
    dhD_dth   = zeros(4,n);
    ddhD_ddth = zeros(4,n);
    % Compute the constraints for each value of theta:
    for i = 1:n
        [hD(:,i),dhD_dth(:,i),ddhD_ddth(:,i)] = targetEvolution(theta(i));
    end

    % Display results:
    figure(101);
    clf;
    xlim([-0.3, 0.3])
    box on; grid on; hold on
    subplot(2, 1, 1)
    plot(theta, hD, 'linewidth', 2);
    set(gca,'FontSize',20)
    title('$h_D$', 'interpreter', 'latex')
    legend({'$h_{D1}$', '$h_{D2}$', '$h_{D3}$', '$h_{D4}$'}, 'interpreter', 'latex')
    subplot(2, 1, 2)
    plot(theta, dhD_dth, 'linewidth', 2);
    title('$\frac{\partial h_D}{\partial \theta}$', 'interpreter', 'latex')
    set(gca,'FontSize',20)
    
end
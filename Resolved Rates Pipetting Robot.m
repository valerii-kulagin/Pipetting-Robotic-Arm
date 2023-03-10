% Resolved-rates control for Puma 560 robot
clear;
close all

% Setup parameters 
tol_ori = [1e-2,0.5];  % [termination, brake point], rad
tol_pos = [1e-3,3e-2];  % [termination, brake point], m

v_max = 0.15; % max end effector linear velocity, m/s
w_max = 0.7; % max end effector angular velocity, rad/s
v_min = 0.07; % min end effector linear velocity, m/s
w_min = 0.14; % min end effector angular velocity, rad/s

singval_threshold = 1e-3; % threshold of triggering singularity robust pseudo-inverse
lambda = 1e-3; % singular value damping coefficient for singularity robustness
dt = 1e-2; % control cycle duration, s
iter_max = 1000; % max number of iterations for each target

% D-H parameters
% 9cm from base to waist, 4.8 cm across waist, 4.8 across arm
params = [[0.09 0.048 0 0.048 0 0.0558]' ... % d, m
          [0 0.05 0.05 0 0 0]' ... % a, m
          [0 0 pi/2 pi/2 -pi/2 0]']; % alpha, rad

% params = [[0.67183 0.1501 0 0.4331 0 0.0558]' ... % d, m
%           [0 0.4318 -0.0203 0 0 0]' ... % a, m
%           [pi/2 0 pi/2 pi/2 -pi/2 0]']; % alpha, rad

needle_len = 0.12; % length of the needle

history = []; % [joint values; end effector pose errors; singularity flag; target number]
visualize = true; 

if visualize
    % initialize the figure for animation
    figure(1);
    view(3)
    axis([-1,1,-1,1,-0.5,1.8]);
    axis equal
    grid on; hold on
    fig = gcf;
    axis_handle = fig.CurrentAxes;
    set(gcf,'CurrentAxes',axis_handle);
    set(gcf,'units','normalized','position',[0 0.05 0.5 0.75]);
end

%Change Jacobian ando other kinematics

q = [0 90 0 0 0 0]'*pi/180; % initial joint values

% Initialize for a single target pose
N = 1;
% by specifying target joint values
% qt = [pi/6; pi/6; 0; 0; pi/6; pi/6]; % initialize using target joint variables
% frames_t = Puma560_forward_kinematics(qt, params, pen_len); % target robot configuration
% Tt = frames_t(:, :, end); % target end effector pose

% or by specifying target end effector pose
% % target_pose = [0.5054 -0.6297 1.1176 -0.4434 0.6648 -0.6013 138.59*pi/180]'; % [position axis angle]
% % R_target = angvec2r(target_pose(end), target_pose(4:6));
% % Tt = [R_target target_pose(1:3);0 0 0 1];

% Initialize for a circular trajectory
N = 72; % number of targets
center = [0.6; -0.2; 1];
R = [0 0 1;
     1 0 0;
     0 1 0]; % orientation of the circle
r = 0.2; % radius
Tt = zeros(4,4,N);
for i = 1:N
    alpha = i/N*2*pi;
    ca = cos(alpha); sa = sin(alpha);
    Tt(1,4,i) = R(1,1)*r*ca + R(1,2)*r*sa + center(1);
    Tt(2,4,i) = R(2,1)*r*ca + R(2,2)*r*sa + center(2);
    Tt(3,4,i) = R(3,1)*r*ca + R(3,2)*r*sa + center(3);
    Tt(1:3,1:3,i) = R;
end

%%

% iteration starts here
for ite_target = 1:size(Tt,3) % loop through all targets
    exitflag = 0;
    singular = false;
    for ite_rr = 1:iter_max + 1 % solve for each target
        % update the current end effector pose
        frames = Puma560_forward_kinematics(q, params, needle_len);
        Tc = frames(:, :, end);

        % calculate end effector pose errors
        err_pos = Tt(1:3,4,ite_target) - Tc(1:3,4);
        [err_ax, err_ang] = get_axis_angle(Tt(1:3,1:3,ite_target)*Tc(1:3,1:3)');
        
        norm_err_pos = norm(err_pos);

        if visualize && (mod(ite_rr,5) == 1)
            cla(axis_handle);
            ht = draw_coordinates(Tt(1:3,4,ite_target), Tt(1:3,1:3,ite_target), 15e-2, 2); % draw target pose
            h = draw_Puma560(frames);
            scatter3(squeeze(Tt(1,4,:)),squeeze(Tt(2,4,:)),squeeze(Tt(3,4,:)));
            % real-time data display
            str{1}=datestr(now);
            str{2}=['Target Pose # ',num2str(ite_target),'   iteration #: ',num2str(ite_rr)];
            str{3}=['Current: ',...
                '  \theta_1=',num2str(q(1)),...
                '  \theta_2=',num2str(q(3)),...
                '  \theta_3=',num2str(q(5))];
            str{4}=['  \theta_4=',num2str(q(2)),...
                '  \theta_5=',num2str(q(4)),...
                '  \theta_6=',num2str(q(6))];
            str{5}=['Error Pos=',num2str(norm_err_pos),'  Error Ori=',num2str(err_ang)];
            disp = text(0.15, -0.15, -0.050, str, 'FontSize', 16);
            drawnow; % immediately draw the current configuration
        end

        % conditions for termination 
        if norm_err_pos < tol_pos(1) %&& err_ang < tol_ori(1) DELETE ORIENTATION CONTROL
            exitflag = 1;  % reached target within tolerance
            break % jump to the next iteration of the outer loop
        elseif ite_rr == iter_max+1 
            exitflag = 2;  % max iteration number exceeded
            break
        end

        % desired linear velocity
        % when close to target, slow down to prevent chattering
        if norm_err_pos < tol_pos(2)
            step_v = (v_max - v_min)*norm_err_pos/tol_pos(2) + v_min;
        else
            step_v = v_max;
        end
        % desired angular velocity
        if err_ang < tol_ori(2)
            step_w = (w_max - w_min)*err_ang/tol_ori(2) + w_min;
        else
            step_w = w_max;
        end

        % assemble the desired end effector twist
        v = step_v*err_pos/norm_err_pos;
        w = step_w*err_ax/norm(err_ax);
        x_dot = [v;w];

        % map to joint velocity
        J = Puma560_Jacobian(frames);
        min_singval_J = min(svd(J));
        if min_singval_J < singval_threshold
            singular = true;
            pinvJ = J'/(J*J' + lambda*eye(size(J,1))); % singularity robust pseudo-inverse
            if visualize && (mod(ite_rr,5) == 1)
                disp = [disp text(frames(1,4,end),frames(2,4,end),frames(3,4,end)+0.02,'singular','FontSize',14)];
                drawnow; % immediately draw the current configuration
            end
        else
            singular = false;
            pinvJ = pinv(J);
        end
        q_dot = pinvJ*x_dot;

        % update joint values
        % In practice here we have to consider joint limits and joint velocity limits
        q = q + q_dot*dt;

        % record for further analysis
        history = [history [q; norm_err_pos; err_ang; err_ax; singular; ite_target]];
    end
end

figure(2);
n = size(history,2);
yyaxis left
plot(1:n,history(7,:),'LineWidth',2);
ylabel('position error (m)');
yyaxis right
plot(1:n,history(8,:),'LineWidth',2);
ylabel('orientation error (rad)');
xlabel('iteration #');
grid on

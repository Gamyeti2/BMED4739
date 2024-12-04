% Resolved-rates control for Puma 560 robot
clear;
close all

video_filename = 'robot_animation_resolved_rates_project.mp4';
video = VideoWriter(video_filename,'MPEG-4'); % Create VideoWriter object
video.FrameRate = 30; % Set frame rate (adjust as needed)
open(video); % Open the video file for writing

% Delete any residual serial connection
delete(instrfindall);
% Initialize Arduino with serial port
arduino = serial('COM6', 'Baudrate', 9600);
% Open Arduino serial port
fopen(arduino);

% Setup parameters, the parameters are defined by expert, aka, yourself.
% you need to tune these parameters to meet your own requirement, such as
% how fast it will converge, and how accurate the robot will be 
tol_ori = [1e-2,0.5];  % [termination, brake point], rad
tol_pos = [1e-3,3e-2];  % [termination, brake point], m
v_max = 0.3; % max end effector linear velocity, m/s
w_max = 0.7; % max end effector angular velocity, rad/s
v_min = 0.1; % min end effector linear velocity, m/s
w_min = 0.14; % min end effector angular velocity, rad/s
singval_threshold = 1e-3; % threshold of triggering singularity robust pseudo-inverse
lambda = 1e-3; % singular value damping coefficient for singularity robustness
dt = 1e-2; % control cycle duration, s
iter = 1000; % max number of iterations for each target

% D-H parameters of the PUMA 560 robot
% params = [[0.67183 0.1501 0 0.4331 0 0.0558]' ... % d, m
%           [0 0.4318 -0.0203 0 0 0]' ... % a, m
%           [pi/2 0 pi/2 pi/2 -pi/2 0]']; % alpha, rad

params = [[0.2027682 0 0 0.0765]' ...] % d, m
          [0 0.057 0.0035 0.03556]' ... % a, m
          [pi/2 0 pi/2 0]']; % alpha, rad

% we assume a pen is added to the tip of the robot 
pen_len = 0.15; % length of the pen

history = []; % [joint values; end effector pose errors; singularity flag; target number]

% setup for the visualization 
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

%q = [0 90 0 0 0 0]'*pi/180; % initial joint values

%q = [0 -37 45 180]'*pi/180;
q = [0 0 90 180]'*pi/180;
% Initialize for a single target pose
 

% by specifying target joint values
% qt = [pi/6; pi/6; 0; 0; pi/6; pi/6]; % initialize using target joint variables
% frames_t = Puma560_forward_kinematics(qt, params, pen_len); % target robot configuration
% Tt = frames_t(:, :, end); % target end effector pose
% or by specifying target end effector pose
% target_pose = [0.5054 -0.6297 1.1176 -0.4434 0.6648 -0.6013 138.59*pi/180]'; % [position axis angle]
% R_target = angvec2r(target_pose(end), target_pose(4:6));
% Tt = [R_target target_pose(1:3);0 0 0 1];

% Initialize for a circular trajectory, you can change to the linear line
% or square or any other shape you like
N = 10; % number of targets
center = [0.25; 0.0; 0.2];
%R = [cos(-pi/4) 0 sin(-pi/4);
%     0 1 0;
%     -sin(-pi/4) 0 cos(-pi/4)]; % orientation of the circle
R = [0 0 1;
    1 0 0;
    0 1 0];
r = 0.05; % radius
Tt = zeros(4,4,N);
for i = 1:N
    alpha = i/N*2*pi;
    ca = cos(alpha); sa = sin(alpha);
    Tt(1,4,i) = R(1,1)*r*ca + R(1,2)*r*sa + center(1);
    Tt(2,4,i) = R(2,1)*r*ca + R(2,2)*r*sa + center(2);
    Tt(3,4,i) = R(3,1)*r*ca + R(3,2)*r*sa + center(3);
    Tt(1:3,1:3,i) = R;
end
Tt(:,:,end+1) = Tt(:,:,1);
Tt(:,:,end+1) = Tt(:,:,2);
% iteration starts here
for j = 1:size(Tt,3) % loop through all targets
    exitflag = 0;
    singular = false;
    check = false;
    for k = 1:iter + 1 % solve for each target
        % update the current end effector pose
        frames = Puma560_forward_kinematics(q, params, pen_len);
        Tc = frames(:, :, end);

        % calculate end effector pose errors
        err_pos = Tt(1:3,4,j) - Tc(1:3,4);
        [err_ax, err_ang] = get_axis_angle(Tt(1:3,1:3,j)*Tc(1:3,1:3)');
        %err_ax = [0;0;1]; err_ang = 0;
        norm_err_pos = norm(err_pos);

        if visualize && (mod(k,5) == 1)
            cla(axis_handle);
            ht = draw_coordinates(Tt(1:3,4), Tt(1:3,1:3), 15e-2, 2); % draw target pose
            h = draw_Puma560(frames);
            scatter3(squeeze(Tt(1,4,:)),squeeze(Tt(2,4,:)),squeeze(Tt(3,4,:)));
            % real-time data display
            str{1}=datestr(now);
            str{2}=['Target Pose # ',num2str(j),'   iteration #: ',num2str(k)];
            str{3}=['Current: ',...
                '  \theta_1=',num2str(q(1)),...
                '  \theta_3=',num2str(q(3)),];
              %  '  \theta_3=',num2str(q(5))];
            str{4}=['  \theta_2=',num2str(q(2)),...
                '  \theta_4=',num2str(q(4)),];
              %  '  \theta_6=',num2str(q(6))];
            str{5}=['Error Pos=',num2str(norm_err_pos),'  Error Ori=',num2str(err_ang)];
            disp = text(0.15, -0.15, -0.050, str, 'FontSize', 16);
            drawnow; % immediately draw the current configuration
            frame = getframe(gcf); % Capture the current figure as a frame
            writeVideo(video, frame); % Write the frame to the video
        end

        % conditions for termination 
        if norm_err_pos < tol_pos(1) %&& err_ang < tol_ori(1)
            change
            fprintf(arduino,'%s\n',vector_str);
            pause(12)
            exitflag = 1;  % reached target within tolerance
            break % jump to the next iteration of the outer loop
        elseif k == iter+1 
            change
            fprintf(arduino,'%s\n',vector_str);
            pause(12)
            exitflag = 2;  % max iteration number exceeded
            break
        end

        % desired linear velocity
        % when close to target, slow down to prevent chattering
        if norm_err_pos < tol_pos(2)
            %step_v = (v_max - v_min)*norm_err_pos/tol_pos(2) + v_min; %% v_max
            step_v = v_max;
        else
            step_v = v_max;
        end
        % desired angular velocity
        if err_ang < tol_ori(2)
            %step_w = (w_max - w_min)*err_ang/tol_ori(2) + w_min; %% w_max
            step_w = w_max;
        else
            step_w = w_max;
        end

        % assemble the desired end effector twist
        v = step_v*err_pos/norm_err_pos;
        w = step_w*err_ax/norm(err_ax);
        w = [0;0;0];
        x_dot = [v;w];

        % map to joint velocity
        J = Puma560_Jacobian(frames); % calculate the robot jacobian

        % this will allow you to check whether the robot is in singular
        % position or not. if yes, we need to use the singularity robust
        % pseudo inverse, as shown below
        min_singval_J = min(svd(J));
        if min_singval_J < singval_threshold
            singular = true;
            pinvJ = J'/(J*J' + lambda*eye(size(J,1))); % singularity robust pseudo-inverse
            if visualize && (mod(k,5) == 1)
                disp = [disp text(frames(1,4,end),frames(2,4,end),frames(3,4,end)+0.02,'singular','FontSize',14)];
                drawnow; % immediately draw the current configuration
            end
        else
            singular = false;
            pinvJ = pinv(J);
        end

        % this is the basic jacobian equation, x_dot = J * q_dot
        q_dot = pinvJ*x_dot;

        % update joint values
        % In practice here we have to consider joint limits and joint velocity limits
        if mod(k,1001) == 1
        qold = q;
        end
        q = q + q_dot*dt;
        change = q - qold;
        change = [0; change];
        % Convert vector to a string
        vector_str = sprintf('%d,', change); 
        vector_str = vector_str(1:end-1); % Remove trailing comma
 
        % Send the string to Arduino
        %writeline(arduino, vector_str);
        % for bit = 1:length(q)
        if j == 1 & ~check
            temp = [0;0;0;0;0];
            vector_strtemp = sprintf('%d, ',temp);
            vector_strtemp = vector_strtemp(1:end-1);
            fprintf(arduino,'%s\n',vector_strtemp);
            pause(12)
            check = true;
        end
        % if mod(k,1001) == 1
        % fprintf(arduino,'%s\n',vector_str);
        % pause(12)
        % end
        %while arduino.BytesAvailable > 0
        %    data = fscanf(arduino);
        %    fprintf('%s',data);
        %end
        % (q(bit))
        %end

        % record for further analysis
        history = [history [q; norm_err_pos; err_ang; err_ax; singular; j]];
    end
end
close(video)

figure(2);
n = size(history,2);
yyaxis left
plot(1:n,history(5,:),'LineWidth',2);
ylabel('position error (m)');
yyaxis right
plot(1:n,history(6,:),'LineWidth',2);
ylabel('orientation error (rad)');
xlabel('iteration #');
grid on

%% subfunctions
function frames = Puma560_forward_kinematics(q, params, pen_len)
% Calculate the forward kinematics of Puma560 using the D-H convention
% q - 6x1 joint vector
% params - 6x3 D-H parameters [d, a, alpha]
% frames - 4x4x7 D-H frames and tool frame w.r.t the spatial frame

frames = zeros(4, 4, 5);
frames(:,:,1) = DH_transform(q(1), params(1,1), params(1,2), params(1,3));
for i = 2:4 %note changed to 4
    frames(:,:,i) = frames(:,:,i - 1)*DH_transform(q(i), params(i,1), params(i,2), params(i,3));
end
T_gripper2pen = [eye(3) [0;0;pen_len]; 0 0 0 1];
frames(:,:,5) = frames(:,:,4)*T_gripper2pen;
end

function T = DH_transform(theta, d, a, alpha)
% Calculate the homogeneous transformation matrix between adjacent D-H frames

st = sin(theta); ct = cos(theta);
sa = sin(alpha); ca = cos(alpha);
T = [ct  -st*ca   st*sa  a*ct;
     st   ct*ca  -ct*sa  a*st;
     0    sa      ca     d;
     0    0       0      1];
end

function J = Puma560_Jacobian(frames)
% Calculate the Jacobian matrix of the puma 560, for velocities of the pen
% frames - 4x4x6 D-H frames w.r.t the spatial frame

% rotation = frames(1:3,1:3,:);  % rotation matrices of D-H frames
origin = frames(1:3,4,:);      % positions of origins of D-H frames
z = frames(1:3,3,:);  % the z-axis of D-H frames, aligned with joint axis

% linear velocity
Jv1 = cross([0;0;1], origin(:,:,5));
Jv2 = cross(z(:,:,1), (origin(:,:,5)-origin(:,:,1)));
Jv3 = cross(z(:,:,2), (origin(:,:,5)-origin(:,:,2)));
Jv4 = cross(z(:,:,3), (origin(:,:,5)-origin(:,:,3)));
%Jv5 = cross(z(:,:,4), (origin(:,:,5)-origin(:,:,4)));
%Jv6 = cross(z(:,:,5), (origin(:,:,5)-origin(:,:,5)));
Jv = [Jv1 Jv2 Jv3 Jv4];
% angular velocity 
Jw= [[0;0;1] z(:,:,1) z(:,:,2) z(:,:,3)];

J = [Jv;Jw];
end

function [axis, angle] = get_axis_angle(R)
% Get the rotation axis and angle from a rotation matrix.
% make sure that R belongs to SO(3) or there can be numerical issues
% axis - 3x1 unit vector

angle = acos(min((trace(R)-1)/2,1));
if abs(angle-pi) < 1e-6
    axis = sqrt([R(1,1)+1; R(2,2)+1; R(3,3)+1]/2);
elseif angle < 1e-6
    axis = [0;0;1]; % almost no rotation so axis does not matter
else
    axis = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]/(2*sin(angle));
end
end

function h = draw_Puma560(frames)
% Draw the robot
h1 = draw_coordinates([0;0;0], eye(3), 15e-2, 2); % draw base frame
h2 = draw_coordinates(frames(1:3,4,end), frames(1:3,1:3,end), 15e-2, 2); % draw end effector frame
color = 'rgbmcyk';
linewidth = [7 6 5 4 3 2 1];
hl = line([0 frames(1,4,1)], [0 frames(2,4,1)], [0 frames(3,4,1)], ...
        'LineWidth', linewidth(1), 'Color', color(1));
h = [h1; h2; hl];
for i = 2:5
    hl = line([frames(1,4,i-1) frames(1,4,i)], [frames(2,4,i-1) frames(2,4,i)], [frames(3,4,i-1) frames(3,4,i)], ...
        'LineWidth', linewidth(i), 'Color', color(i));
    h = [h; hl];
end
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
end

function arrows = draw_coordinates(p,R,length,width)
% Draw Cartesian coordinates
% length - length of the axis
% width - line width of the axis

R=length.*R; %introduce length to the orthogonal vectors
hold on
arrow_x = quiver3(p(1),p(2),p(3),R(1,1),R(2,1),R(3,1),'r','filled','LineWidth',width);   % x axis
arrow_y = quiver3(p(1),p(2),p(3),R(1,2),R(2,2),R(3,2),'g','filled','LineWidth',width);   % y axis
arrow_z = quiver3(p(1),p(2),p(3),R(1,3),R(2,3),R(3,3),'b','filled','LineWidth',width);   % z axis
arrows = [arrow_x; arrow_y; arrow_z];
end
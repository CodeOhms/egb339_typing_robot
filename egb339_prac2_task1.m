% Call the script that defines the x and y coordinates of the keyboard:
keyboard_defs

% Homogeneous transform from robot base to centre of keyboard:
keyboard_SE3 = SE3T("x", 175) * SE3T("y", -150) * SE3T("z", 2) * SE3R("z", -135*pi/180);

% Homogeneous transform from the centre of keyboard to the top left corner:
keyboard_SE3 = keyboard_SE3 * SE3T("x", -145) * SE3T("y", 60);

% Create a dictionary of keyboard button middle positions relative to robot base:
buts_mid_pos = buttons_mid_pos_rel(rows_num_buttons, rows_button_keys, buttons_x_mid_pos, buttons_y_mid_pos, keyboard_SE3);

% task1('SPNKR', buts_mid_pos);
task1(dobot, 'H', buts_mid_pos, keyboard_SE3);


function task1(dobot, word, buttons_mid_pos, keyboard_SE3)
    % Setup connection to sim.:
    addpath interface
    % dobot = coppeliaRobot();

    % Ensure it starts at origin:
    % dobot.setJointAngles([0 0 0 0]);
    % pause(2);

    % Type a word:
    type_word(dobot, word, buttons_mid_pos, keyboard_SE3);
end

function type_word(bot, word, buts_mid_pos, keyboard_SE3)
    for l_i = 1:length(word)
        % Get x and y coordinate of letter:
        but_coords = buts_mid_pos(word(l_i));

        % Press the letter button:
        button_press(bot, but_coords(1), but_coords(2), keyboard_SE3);
    end
end

function button_press(bot, x, y, keyboard_SE3)
    % Assuming the end affector is positioned above the key, move the arm down to press the key.

    end_eff_height = 40;
    press_time = 0.1;

    % Move the robot to the letter's x and y position:
    % x = 175;
    % y = -150;
    but_coords_hover = [x; y; end_eff_height];
    arm_move(bot, but_coords_hover, keyboard_SE3);
    
    % Press for a moment:
    but_coords_press = [x; y; 2];
    arm_move(bot, but_coords_press, keyboard_SE3);
    pause(press_time);
    
    % Lift the arm up:
    arm_move(bot, but_coords_hover, keyboard_SE3);
end

function arm_move(bot, coords, keyboard_SE3)
    % Block until the arm is in position.

    % Calculate current SE3 of robot arm:
    end_eff_SE3 = end_eff_SE3_calc(bot);

    % Calculate the pose from the end effector to the keyboard pose:
    end_eff_to_keyboard_SE3 = inv(inv(keyboard_SE3)* end_eff_SE3);

    % Calculate the coords relative to the end effector:
    coords;
    % coords_trans = end_eff_to_keyboard_SE3 * [coords; 1];
    % coords_trans = coords_trans./coords_trans(4);
    % coords = coords_trans(1:3, :);

    wait_time = 0.06;
    tol = 0.01;
    
    [thetas, q] = arm_q_angles(coords);
    bot.setJointAngles(q);
    forwardKinematics_q(q)

    % Wait for arm to reach position:
    pause(2)
    
    % while 1
    %     current_q = bot.getJointAngles();
    %     abs(current_q - q);
    %     if abs(current_q - q) < tol
    %         break
    %     end
    %     pause(wait_time)
    % end
end

function end_eff_SE3 = end_eff_SE3_calc(bot)
    % Current pose angles (q) of the arm:
    q_current = bot.getJointAngles();

    % Calculate the pose:
    end_eff_SE3 = end_eff_pose_calc(q_current);
end

function p = forwardKinematics_q(q)
    % q is a 4-element vector of kinematic joint angles (radians)
    % p is a 3x1 vector giving the position of the robot's end effector (millimeters)

    end_eff_pose = end_eff_pose_calc(q);
    p = [end_eff_pose(1:3, 4)];
    p = p./end_eff_pose(4,4);
end

function end_eff_pose = end_eff_pose_calc(q)
    % L0 = 138;  % height of shoulder raise joint above ground plane
    % L1 = 135;  % length of upper arm
    % L2 = 147;  % length of lower arm
    % L3 = 60;   % horizontal tool displacement from wrist passive joint
    % L4 = -70;  % vertical tool displacement from wrist passive joint

    jdisps = [138 135 147 60 -70];

    waist_SE3r = SE3R("z", q(1));
    waist_SE3t = SE3T("z", jdisps(1));
    elbow_SE3r = SE3R("y", q(2));
    elbow_SE3t = SE3T("z", jdisps(2));
    farm_SE3r = SE3R("y", q(3));
    farm_SE3t = SE3T("x", jdisps(3));
    wrist_SE3r = SE3R("y", q(4));
    wrist_SE3t = SE3T("x", jdisps(4));
    end_eff_SE3t = SE3T("z", jdisps(5));

    end_eff_pose = waist_SE3r*waist_SE3t*elbow_SE3r*elbow_SE3t*farm_SE3r*farm_SE3t*wrist_SE3r*wrist_SE3t*end_eff_SE3t;
end

function [thetas, q] = arm_q_angles(coords)
    % thetas = inverseKinematic(coords);
    thetas = inverseKinematic([150 -175 2]);
    q = jointMapping(thetas);
end

function q = jointMapping(theta)
    q = [theta(1), theta(2), theta(3)-theta(2), -theta(3)];
end

% function theta = inverseKinematic(pstar);
%     % pstar - 1x3 vector of desired robot end effector position in millimetres
%     % theta - 1x3 vector of robot joint angles in radians

%     % robot dimensions
%     L0 = 138;  % height of shoulder raise joint above ground plane
%     L1 = 135;  % length of upper arm
%     L2 = 147;  % length of lower arm
%     L3 = 60;   % horizontal displacement from wrist "passive" joint
%     L4 = -70;  % vertical displacement down from wrist "passive" joint


%     % your code in here
%     theta1 = atan(pstar(2)/pstar(1));
%     r = pstar(1)/cos(theta1);
%     a = r-L3;
%     b = L0-pstar(3)+L4;
%     delta = atan2(a,b);
%     c = b/cos(delta);
%     alpha = acos((L1^2+L2^2-c^2)/(2*L1*L2));
%     beta = acos((L1^2+c^2-L2^2)/(2*L1*c));
%     theta2 = pi - beta - delta;
%     theta3 = pi/2 - alpha + theta2;
    
%     theta = [theta1,theta2,theta3];
% end

function theta = inverseKinematic(pstar)
    % pstar - 1x3 vector of desired robot end effector position in millimetres
    % theta - 1x3 vector of robot joint angles in radians

    % robot dimensions
    L0 = 138;  % height of shoulder raise joint above ground plane
    L1 = 135;  % length of upper arm
    L2 = 147;  % length of lower arm
    L3 = 60;   % horizontal displacement from wrist "passive" joint
    L4 = -70;  % vertical displacement down from wrist "passive" joint


    % your code in here
    x = pstar(1);
    y = pstar(2);
    z = pstar(3);

    r = sqrt(x^2 + y^2);
    h = -L4 + z;
    b = abs(L0 - h);
    a = r - L3;
    c = sqrt(a^2 + b^2);
    alpha = acos((L1^2 + L2^2 - c^2)/(2*L1*L2));
    beta = asin(sin(alpha)/c * L2);

    theta = zeros(1,3);
    theta(1) = atan2(y, x);
    if h > L0
        mu = atan2(b, a);
        theta(2) = pi/2 - mu - beta;
    else
        delta_ang = atan2(a, b);
        theta(2) = pi - (beta + delta_ang);
    end
    theta(3) = pi - (alpha + pi/2 - theta(2));
end

function SE3 = SE3T(axis, displacement)
    disp_x = 0;
    disp_y = 0;
    disp_z = 0;
    if axis == "x"
        disp_x = displacement;
    elseif axis == "y"
        disp_y = displacement;
    else
        disp_z = displacement;
    end
    SE3 = eye(4);
    SE3(1:3,4) = [disp_x; disp_y; disp_z];
end

function SE3 = SE3R(axis, theta)
    SE3 = [[rotationMatrix(axis, theta); 0 0 0], [0;0;0;1]];
end

function R = rotationMatrix(axis, angle)
    % input arguments
    %  axis is a string "x", "y" or "z"
    %  angle is a scalar
    if axis=="z"
        R = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
    elseif axis=="y"
        R = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
    else
        R = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];
    end
end
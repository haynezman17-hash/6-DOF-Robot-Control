function moveCartesian(xTarget,yTarget,zTarget)

% =============================================================
% STRAIGHT-LINE CARTESIAN MOVEMENT
%
% Usage:
%
%   moveCartesian(X,Y,Z)
%
% Example:
%
%   moveXYZ(200,200,200)
%   moveCartesian(300,200,200)
%
% The current physical / MATLAB state is used as the start.
%
% The complete path is previewed once.
% User confirms once.
% Then all Cartesian waypoints execute automatically.
% =============================================================


%% =============================================================
% INPUT CHECK
%% =============================================================

if nargin ~= 3
    error('Use moveCartesian(X,Y,Z)');
end

xTarget = double(xTarget);
yTarget = double(yTarget);
zTarget = double(zTarget);

if ~isscalar(xTarget) || ...
   ~isscalar(yTarget) || ...
   ~isscalar(zTarget) || ...
   ~all(isfinite([xTarget yTarget zTarget]))

    error('X, Y and Z must be finite scalar values.');
end

targetPosition = [xTarget yTarget zTarget];


%% =============================================================
% GET ROBOT MODEL
%% =============================================================

robot = evalin('base','robot');
offsetDeg = evalin('base','offsetDeg');
controllerHomeDeg = evalin('base','controllerHomeDeg');


%% =============================================================
% GET EXISTING SERIAL CONNECTION
%% =============================================================

try
    robotSerial = evalin('base','robotSerial');
catch
    error( ...
        ['No robot connection exists.' newline ...
         'Run moveXYZ("home") first.']);
end

if isempty(robotSerial) || ~isvalid(robotSerial)
    error( ...
        ['Robot serial connection is invalid.' newline ...
         'Run moveXYZ("home") first.']);
end


%% =============================================================
% GET CURRENT ROBOT CONFIGURATION
%% =============================================================

try
    qCurrent = evalin('base','qCurrent');
catch
    error( ...
        ['Current robot configuration is unknown.' newline ...
         'Run moveXYZ("home") first.']);
end

qCurrent = reshape(qCurrent,1,[]);

if numel(qCurrent) ~= 6
    error('qCurrent must contain exactly 6 joint values.');
end


%% =============================================================
% CURRENT TCP
%% =============================================================

Tstart = getTransform( ...
    robot, ...
    qCurrent, ...
    'tool0', ...
    robot.BaseName);

startPosition = tform2trvec(Tstart);


%% =============================================================
% CARTESIAN PATH SETTINGS
%% =============================================================

% Smaller waypoint spacing = smoother Cartesian path
STEP_SIZE_MM = 2.0;

delta = targetPosition - startPosition;

pathDistance = norm(delta);


if pathDistance < 0.01
    fprintf('\nRobot is already at the requested position.\n');
    return;
end


numSegments = ...
    max(1,ceil(pathDistance / STEP_SIZE_MM));

numPoints = ...
    numSegments + 1;


%% =============================================================
% GENERATE STRAIGHT LINE
%% =============================================================

lambda = linspace(0,1,numPoints)';

pathXYZ = ...
    startPosition + lambda .* delta;


%% =============================================================
% DISPLAY PATH INFORMATION
%% =============================================================

fprintf('\n');
fprintf('=============================================\n');
fprintf('          CARTESIAN LINEAR MOVE\n');
fprintf('=============================================\n\n');

fprintf( ...
    'Start  : [%.2f %.2f %.2f] mm\n', ...
    startPosition);

fprintf( ...
    'Target : [%.2f %.2f %.2f] mm\n', ...
    targetPosition);

fprintf('Distance: %.2f mm\n',pathDistance);
fprintf('Step size: <= %.2f mm\n',STEP_SIZE_MM);
fprintf('Segments: %d\n\n',numSegments);


%% =============================================================
% PREVIEW CARTESIAN PATH
%% =============================================================

figure( ...
    'Name','Cartesian Path Preview', ...
    'NumberTitle','off', ...
    'Color','white');

plot3( ...
    pathXYZ(:,1), ...
    pathXYZ(:,2), ...
    pathXYZ(:,3), ...
    '-o', ...
    'LineWidth',2, ...
    'MarkerSize',3);

hold on;

plot3( ...
    startPosition(1), ...
    startPosition(2), ...
    startPosition(3), ...
    'o', ...
    'MarkerSize',10, ...
    'LineWidth',2);

plot3( ...
    targetPosition(1), ...
    targetPosition(2), ...
    targetPosition(3), ...
    'x', ...
    'MarkerSize',14, ...
    'LineWidth',3);

text( ...
    startPosition(1), ...
    startPosition(2), ...
    startPosition(3), ...
    '  START', ...
    'FontWeight','bold');

text( ...
    targetPosition(1), ...
    targetPosition(2), ...
    targetPosition(3), ...
    '  TARGET', ...
    'FontWeight','bold');

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

title('Requested Straight-Line Cartesian Path');

grid on;
axis equal;
axis vis3d;

view(135,25);
rotate3d on;

hold off;

drawnow;


%% =============================================================
% CONFIRM ONCE
%% =============================================================

answer = input( ...
    sprintf( ...
    '\nExecute %d Cartesian segments? (y/n): ', ...
    numSegments), ...
    's');

if ~strcmpi(answer,'y')

    fprintf('Cartesian movement cancelled.\n');
    return;

end


%% =============================================================
% EXECUTE PATH
%% =============================================================

fprintf('\nBeginning Cartesian movement...\n\n');


for k = 2:numPoints

    waypoint = pathXYZ(k,:);

    qPrevious = qCurrent;


    fprintf( ...
        'Waypoint %d/%d  [%.2f %.2f %.2f]\n', ...
        k-1, ...
        numSegments, ...
        waypoint(1), ...
        waypoint(2), ...
        waypoint(3));


    %% ---------------------------------------------------------
    % SOLVE + EXECUTE
    %
    % false = do NOT ask for confirmation inside moveToPose
    %% ---------------------------------------------------------

    try

        [~,qNew] = moveToPose( ...
            robotSerial, ...
            robot, ...
            offsetDeg, ...
            controllerHomeDeg, ...
            waypoint, ...
            qCurrent, ...
            false);

    catch ME

        assignin( ...
            'base', ...
            'qCurrent', ...
            qCurrent);

        fprintf('\n');
        fprintf('=============================================\n');
        fprintf('        CARTESIAN MOVE ABORTED\n');
        fprintf('=============================================\n\n');

        fprintf( ...
            'Failure at waypoint %d/%d\n', ...
            k-1, ...
            numSegments);

        fprintf( ...
            'Waypoint = [%.2f %.2f %.2f] mm\n\n', ...
            waypoint);

        rethrow(ME);

    end


    qNew = reshape(qNew,1,[]);


    if numel(qNew) ~= 6 || any(~isfinite(qNew))
        error('Invalid IK configuration returned.');
    end


    %% ---------------------------------------------------------
    % JOINT-JUMP SAFETY CHECK
    %% ---------------------------------------------------------

    jointChangeDeg = ...
        abs(rad2deg(qNew - qPrevious));

    MAX_WAYPOINT_JOINT_CHANGE = 8;

    if any(jointChangeDeg > MAX_WAYPOINT_JOINT_CHANGE)

        assignin( ...
            'base', ...
            'qCurrent', ...
            qCurrent);

        fprintf('\nJoint changes:\n');
        disp(jointChangeDeg);

        error( ...
            ['Cartesian move stopped because an IK waypoint ' ...
             'required more than %.1f deg of joint change.'], ...
            MAX_WAYPOINT_JOINT_CHANGE);

    end


    %% ---------------------------------------------------------
    % UPDATE CURRENT STATE
    %% ---------------------------------------------------------

    qCurrent = qNew;

    assignin( ...
        'base', ...
        'qCurrent', ...
        qCurrent);

end


%% =============================================================
% FINAL TCP
%% =============================================================

Tfinal = getTransform( ...
    robot, ...
    qCurrent, ...
    'tool0', ...
    robot.BaseName);

finalPosition = ...
    tform2trvec(Tfinal);

finalError = ...
    norm(finalPosition - targetPosition);


%% =============================================================
% STORE FINAL STATE
%% =============================================================

assignin('base','qCurrent',qCurrent);
assignin('base','robotSerial',robotSerial);


%% =============================================================
% REPORT
%% =============================================================

fprintf('\n');
fprintf('=============================================\n');
fprintf('        CARTESIAN MOVE COMPLETE\n');
fprintf('=============================================\n\n');

fprintf( ...
    'Requested TCP : [%.2f %.2f %.2f] mm\n', ...
    targetPosition);

fprintf( ...
    'Model TCP     : [%.2f %.2f %.2f] mm\n', ...
    finalPosition);

fprintf( ...
    'Position error: %.3f mm\n\n', ...
    finalError);

end
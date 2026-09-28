function [qControllerCmd, qIKModel] = moveToPose( ...
robotSerial, ...
robot, ...
offsetDeg, ...
controllerHomeDeg, ...
position, ...
qCurrent, ...
confirmMove)

% =============================================================
% MOVE TO CARTESIAN POSE
%
% Usage from moveXYZ:
%
%   [qCmd,qNew] = moveToPose( ...
%       robotSerial,robot,offsetDeg,controllerHomeDeg, ...
%       [X Y Z],qCurrent);
%
% Usage from moveCartesian:
%
%   [qCmd,qNew] = moveToPose( ...
%       robotSerial,robot,offsetDeg,controllerHomeDeg, ...
%       waypoint,qCurrent,false);
%
%
% CURRENT PHYSICAL CONVENTION
%
% J0 Base:
%   Frame 0 deg -> X0
%
% J1 Shoulder:
%   Frame 0 deg -> Y90
%
% J2 Elbow:
%   Frame 0 deg -> Z90
%
% J3 Wrist Yaw:
%   Frame 0 deg -> Servo S270
%   Positive frame motion DECREASES servo command
%
% J4 Wrist Pitch:
%   Frame 0 deg -> Servo S0
%   Direct drive
%
% J5 Wrist Roll:
%   Frame 0 deg -> Servo S0
%   Direct drive
%
% =============================================================

%% =============================================================
% OPTIONAL CONFIRMATION
%
% If omitted:
%       confirmMove = true
%
% moveXYZ therefore asks before moving.
%
% moveCartesian passes false after confirming the complete
% Cartesian path once.
%% =============================================================

if nargin < 7 || isempty(confirmMove)

confirmMove = true;

end

%% =============================================================
% VALIDATE INPUTS
%% =============================================================

qCurrent = reshape(qCurrent,1,[]);

if numel(qCurrent) ~= 6

error( ...
    'qCurrent must contain exactly 6 joint values.');

end

position = reshape(position,1,[]);

if numel(position) ~= 3

error( ...
    'position must be [X Y Z].');

end

if any(~isfinite(position))

error( ...
    'Cartesian target contains NaN or Inf.');

end

%% =============================================================
% CREATE TARGET TRANSFORM
%
% XYZ position only.
%
% Orientation is currently unconstrained.
%% =============================================================

Ttarget = eye(4);

Ttarget(1:3,4) = ...
position(:);

%% =============================================================
% CREATE IK SOLVER
%% =============================================================

ikSolver = inverseKinematics( ...
    'RigidBodyTree', ...
    robot);


%% =============================================================
% IK WEIGHTS
%
% Position is important.
% Orientation is currently unconstrained.
%% =============================================================

weights = [0 0 0 1 1 1];

%% =============================================================
% MULTI-START IK
%
% Try several starting configurations, then choose the valid
% solution requiring the least sensible joint movement from
% the CURRENT physical robot configuration.
%% =============================================================

% Current configuration
qCurrent = reshape(qCurrent,1,[]);


% Candidate seeds
%
% First candidate is ALWAYS the current robot position.
seedList = [

    qCurrent;

    % Home-like seed
    deg2rad([ ...
         0 ...
       -90 ...
        90 ...
         0 ...
         0 ...
         0 ]);

    % Alternate shoulder/elbow configurations
    deg2rad([ ...
         0 ...
       -45 ...
        45 ...
         0 ...
         0 ...
         0 ]);

    deg2rad([ ...
         0 ...
       -30 ...
        90 ...
         0 ...
         0 ...
         0 ]);

];


numSeeds = ...
    size(seedList,1);


candidateQ = ...
    zeros(numSeeds,6);


candidateValid = ...
    false(numSeeds,1);


candidateCost = ...
    inf(numSeeds,1);


%% =============================================================
% COST WEIGHTS
%
% Larger number = discourage movement of that joint more.
%
% J0 Base
% J1 Shoulder
% J2 Elbow
% J3 Yaw
% J4 Pitch
% J5 Roll
%% =============================================================

jointCostWeights = [

    1.0 ...      % J0 Base
    0.7 ...      % J1 Shoulder
    0.7 ...      % J2 Elbow
    2.0 ...      % J3 Yaw
    2.5 ...      % J4 Pitch
    2.0          % J5 Roll

];


%% =============================================================
% PHYSICAL FRAME LIMITS
%
% These should match moveToJoints.m
%% =============================================================

frameLimitsDeg = [

   -180    180;      % J0 Base
    -90     90;      % J1 Shoulder
    -90     90;      % J2 Elbow

      0    270;      % J3 Yaw

      0    135;      % J4 Pitch physical range
                     % Change this if your true mechanical
                     % pitch range is different

      0    180       % J5 Roll

];


%% =============================================================
% TRY EACH IK SEED
%% =============================================================

for n = 1:numSeeds

    qSeed = ...
        seedList(n,:);


    try

        qCandidate = ikSolver( ...
            'tool0', ...
            Ttarget, ...
            weights, ...
            qSeed);


        qCandidate = ...
            reshape(qCandidate,1,[]);


        %% -----------------------------------------------------
        % BASIC VALIDITY
        %% -----------------------------------------------------

        if numel(qCandidate) ~= 6 || ...
           any(~isfinite(qCandidate))

            continue;

        end


        %% -----------------------------------------------------
        % MODEL -> PHYSICAL FRAME
        %% -----------------------------------------------------

        candidateFrameDeg = ...
            rad2deg(qCandidate) - offsetDeg;


        %% -----------------------------------------------------
        % CHECK PHYSICAL LIMITS
        %% -----------------------------------------------------

        insideLimits = true;


        for j = 1:6

            if candidateFrameDeg(j) < frameLimitsDeg(j,1) || ...
               candidateFrameDeg(j) > frameLimitsDeg(j,2)

                insideLimits = false;
                break;

            end

        end


        if ~insideLimits
            continue;
        end


        %% -----------------------------------------------------
        % MOVEMENT FROM CURRENT ROBOT CONFIGURATION
        %% -----------------------------------------------------

        deltaDeg = ...
            rad2deg(qCandidate - qCurrent);


        % Wrap angular difference to -180 to +180
        deltaDeg = ...
            mod(deltaDeg + 180,360) - 180;


        %% -----------------------------------------------------
        % BASE MOVEMENT COST
        %% -----------------------------------------------------

        movementCost = ...
            sum( ...
            jointCostWeights .* ...
            abs(deltaDeg));


        %% -----------------------------------------------------
        % EXTRA PENALTY FOR EXTREME WRIST PITCH
        %
        % Encourages the solver not to use pitch unnecessarily.
        %% -----------------------------------------------------

        pitchAngle = ...
            candidateFrameDeg(5);


        pitchPenalty = ...
            0.03 * pitchAngle^2;


        %% -----------------------------------------------------
        % TOTAL COST
        %% -----------------------------------------------------

        candidateCost(n) = ...
            movementCost + pitchPenalty;


        candidateQ(n,:) = ...
            qCandidate;


        candidateValid(n) = ...
            true;


    catch

        % Ignore failed seed
        continue;

    end

end


%% =============================================================
% CHOOSE BEST VALID SOLUTION
%% =============================================================

validIndices = ...
    find(candidateValid);


if isempty(validIndices)

    error( ...
        ['No valid IK solution was found within the ' ...
         'physical joint limits.']);

end


[~,localBest] = ...
    min(candidateCost(validIndices));


bestIndex = ...
    validIndices(localBest);


qIKModel = ...
    candidateQ(bestIndex,:);


%% =============================================================
% DISPLAY SOLUTION SELECTION
%% =============================================================

if confirmMove

    fprintf('\nIK candidates:\n');

    for n = 1:numSeeds

        if candidateValid(n)

            fprintf( ...
                'Candidate %d: cost = %.2f\n', ...
                n, ...
                candidateCost(n));

        else

            fprintf( ...
                'Candidate %d: invalid\n', ...
                n);

        end

    end


    fprintf( ...
        '\nSelected IK candidate %d\n', ...
        bestIndex);

end

%% =============================================================
% VALIDATE SELECTED MULTI-START IK SOLUTION
%
% qIKModel was selected above from the valid multi-start
% candidates. Do NOT solve IK again here, because doing so would
% overwrite the candidate selected by the cost function.
%% =============================================================

qIKModel = ...
reshape(qIKModel,1,[]);

if numel(qIKModel) ~= 6

error( ...
    'IK solver did not return six joint values.');

end

if any(~isfinite(qIKModel))

error( ...
    'IK returned NaN or Inf joint values.');

end

%% =============================================================
% MATLAB MODEL -> PHYSICAL FRAME ANGLES
%
% qModel = qFrame + offset
%
% therefore:
%
% qFrame = qModel - offset
%% =============================================================

qIKModelDeg = ...
rad2deg(qIKModel);

qFrameDeg = ...
qIKModelDeg - offsetDeg;

%% =============================================================
% PHYSICAL FRAME LIMITS
%
% These MUST match SIX_DOF_Manipulator.m
%% =============================================================

frameLimitsDeg = [

-180    180;      % J0 Base
-90     90;      % J1 Shoulder
-90     90;      % J2 Elbow

  0    270;      % J3 Wrist Yaw

  0    135;      % J4 Wrist Pitch

  0    180       % J5 Wrist Roll

];

%% =============================================================
% CHECK PHYSICAL FRAME LIMITS
%% =============================================================

FRAME_TOL = 0.05;

for i = 1:6

lowerLimit = ...
    frameLimitsDeg(i,1);

upperLimit = ...
    frameLimitsDeg(i,2);


if qFrameDeg(i) < lowerLimit - FRAME_TOL || ...
   qFrameDeg(i) > upperLimit + FRAME_TOL

    fprintf('\n');
    fprintf('=============================================\n');
    fprintf('             MOTION REJECTED\n');
    fprintf('=============================================\n\n');

    fprintf( ...
        'Joint J%d requested frame angle: %.2f deg\n', ...
        i-1, ...
        qFrameDeg(i));

    fprintf( ...
        'Allowed range: %.2f to %.2f deg\n\n', ...
        lowerLimit, ...
        upperLimit);

    error( ...
        'IK solution exceeds physical joint limits.');

end

end

%% =============================================================
% FRAME ANGLES -> CONTROLLER COMMANDS
%% =============================================================

qControllerCmd = ...
zeros(1,6);

%% -------------------------------------------------------------
% J0 BASE
%
% Frame 0 -> X0
%% -------------------------------------------------------------

qControllerCmd(1) = ...
controllerHomeDeg(1) + qFrameDeg(1);

%% -------------------------------------------------------------
% J1 SHOULDER
%
% Frame 0 -> Y90
%% -------------------------------------------------------------

qControllerCmd(2) = ...
controllerHomeDeg(2) + qFrameDeg(2);

%% -------------------------------------------------------------
% J2 ELBOW
%
% Frame 0 -> Z90
%% -------------------------------------------------------------

qControllerCmd(3) = ...
controllerHomeDeg(3) + qFrameDeg(3);

%% -------------------------------------------------------------
% J3 WRIST YAW
%
% Frame 0 -> S270
%
% Yaw output is physically 180 deg from the earlier setup.
%
% Positive physical yaw decreases the servo command:
%
% Frame   0 -> S270
% Frame  45 -> S225
% Frame  90 -> S180
% Frame 180 -> S90
% Frame 270 -> S0
%% -------------------------------------------------------------

qControllerCmd(4) = ...
controllerHomeDeg(4) - qFrameDeg(4);

%% -------------------------------------------------------------
% J4 WRIST PITCH
%
% DIRECT DRIVE
%
% Frame 0 -> S0
%% -------------------------------------------------------------

qControllerCmd(5) = ...
controllerHomeDeg(5) + qFrameDeg(5);

%% -------------------------------------------------------------
% J5 WRIST ROLL
%
% DIRECT DRIVE
%
% Frame 0 -> S0
%% -------------------------------------------------------------

qControllerCmd(6) = ...
controllerHomeDeg(6) + qFrameDeg(6);

%% =============================================================
% CONTROLLER LIMITS
%% =============================================================

controllerLimitsDeg = [

-180    180;      % X / Base coordinate
0    180;      % Y / Shoulder
0    180;      % Z / Elbow

  0    270;      % Yaw servo
  0    135;      % Pitch servo
  0    180       % Roll servo

];

%% =============================================================
% CHECK CONTROLLER COMMAND LIMITS
%% =============================================================

COMMAND_TOL = 0.5;

for i = 1:6

if qControllerCmd(i) < ...
        controllerLimitsDeg(i,1) - COMMAND_TOL || ...
   qControllerCmd(i) > ...
        controllerLimitsDeg(i,2) + COMMAND_TOL

    fprintf('\n');
    fprintf('Controller command rejected.\n');

    fprintf( ...
        'Joint J%d command = %.2f deg\n', ...
        i-1, ...
        qControllerCmd(i));

    fprintf( ...
        'Allowed controller range = %.2f to %.2f deg\n', ...
        controllerLimitsDeg(i,1), ...
        controllerLimitsDeg(i,2));

    error( ...
        'Controller command outside physical range.');

end

end

%% =============================================================
% CLAMP TINY FLOATING-POINT ERRORS
%% =============================================================

for i = 1:6

qControllerCmd(i) = ...
    max( ...
    controllerLimitsDeg(i,1), ...
    min( ...
    controllerLimitsDeg(i,2), ...
    qControllerCmd(i)));

end

%% =============================================================
% VERIFY IK TCP
%% =============================================================

Tactual = getTransform( ...
robot, ...
qIKModel, ...
'tool0', ...
robot.BaseName);

actualPosition = ...
tform2trvec(Tactual);

positionError = ...
norm(actualPosition - position);

%% =============================================================
% DISPLAY SOLUTION
%
% Avoid printing huge amounts of information during every
% Cartesian interpolation waypoint.
%% =============================================================

if confirmMove

fprintf('\n');
fprintf('=============================================\n');
fprintf('                IK SOLUTION\n');
fprintf('=============================================\n\n');


fprintf( ...
    'Requested TCP : [%.2f %.2f %.2f] mm\n', ...
    position(1), ...
    position(2), ...
    position(3));


fprintf( ...
    'Solved TCP    : [%.2f %.2f %.2f] mm\n', ...
    actualPosition(1), ...
    actualPosition(2), ...
    actualPosition(3));


fprintf( ...
    'Position error: %.4f mm\n\n', ...
    positionError);


fprintf( ...
    'Joint                 Frame        Command\n');

fprintf( ...
    '------------------------------------------------\n');


fprintf( ...
    'J0 Base          %9.2f      X%8.2f\n', ...
    qFrameDeg(1), ...
    qControllerCmd(1));


fprintf( ...
    'J1 Shoulder      %9.2f      Y%8.2f\n', ...
    qFrameDeg(2), ...
    qControllerCmd(2));


fprintf( ...
    'J2 Elbow         %9.2f      Z%8.2f\n', ...
    qFrameDeg(3), ...
    qControllerCmd(3));


fprintf( ...
    'J3 Wrist Yaw     %9.2f      S%8.2f\n', ...
    qFrameDeg(4), ...
    qControllerCmd(4));


fprintf( ...
    'J4 Wrist Pitch   %9.2f      S%8.2f\n', ...
    qFrameDeg(5), ...
    qControllerCmd(5));


fprintf( ...
    'J5 Wrist Roll    %9.2f      S%8.2f\n', ...
    qFrameDeg(6), ...
    qControllerCmd(6));

end

%% =============================================================
% VISUALIZE SOLUTION
%
% Only show the detailed IK visualization for single-point
% moveXYZ moves.
%
% During moveCartesian, confirmMove=false, so MATLAB does not
% redraw the robot 50-100 times during one trajectory.
%% =============================================================

if confirmMove

persistent ikFigure
persistent ikAxes


if isempty(ikFigure) || ...
   ~isvalid(ikFigure)

    ikFigure = figure( ...
        'Name','IK Physical Robot Target', ...
        'NumberTitle','off', ...
        'Color','white');

    ikAxes = axes(ikFigure);

else

    figure(ikFigure);

    cla(ikAxes);

end


hold(ikAxes,'on');

grid(ikAxes,'on');

axis(ikAxes,'equal');

axis(ikAxes,'vis3d');


xlabel(ikAxes,'X (mm)');
ylabel(ikAxes,'Y (mm)');
zlabel(ikAxes,'Z (mm)');


title( ...
    ikAxes, ...
    sprintf( ...
    'IK Target [%.1f %.1f %.1f] mm', ...
    position(1), ...
    position(2), ...
    position(3)));


%% ---------------------------------------------------------
% BODY NAMES
%% ---------------------------------------------------------

bodyNamesIK = {

    'link1'
    'link2'
    'link3'
    'link4'
    'link5'
    'link6'

};


%% ---------------------------------------------------------
% ROBOT SKELETON
%% ---------------------------------------------------------

P = zeros(8,3);

% Physical J0 base
P(1,:) = [0 0 0];


for i = 1:6

    Tjoint = getTransform( ...
        robot, ...
        qIKModel, ...
        bodyNamesIK{i}, ...
        robot.BaseName);

    P(i+1,:) = ...
        Tjoint(1:3,4)';

end


% TCP
P(8,:) = ...
    actualPosition;


plot3( ...
    ikAxes, ...
    P(:,1), ...
    P(:,2), ...
    P(:,3), ...
    '-o', ...
    'LineWidth',4, ...
    'MarkerSize',8);


%% ---------------------------------------------------------
% LABELS
%% ---------------------------------------------------------

text( ...
    ikAxes, ...
    P(1,1), ...
    P(1,2), ...
    P(1,3), ...
    '  J0 BASE', ...
    'FontWeight','bold');


labels = {

    'J1 SHOULDER'
    'J2 ELBOW'
    'J3 WRIST YAW'
    'J4 WRIST PITCH'
    'J5 WRIST ROLL'
    'LINK6'

};


for i = 1:6

    text( ...
        ikAxes, ...
        P(i+1,1), ...
        P(i+1,2), ...
        P(i+1,3), ...
        ['  ' labels{i}], ...
        'FontSize',9, ...
        'FontWeight','bold');

end


%% ---------------------------------------------------------
% REQUESTED TCP
%% ---------------------------------------------------------

plot3( ...
    ikAxes, ...
    position(1), ...
    position(2), ...
    position(3), ...
    'rx', ...
    'MarkerSize',15, ...
    'LineWidth',3);


text( ...
    ikAxes, ...
    position(1), ...
    position(2), ...
    position(3), ...
    '  TARGET', ...
    'FontWeight','bold');


%% ---------------------------------------------------------
% SOLVED TCP
%% ---------------------------------------------------------

plot3( ...
    ikAxes, ...
    actualPosition(1), ...
    actualPosition(2), ...
    actualPosition(3), ...
    'go', ...
    'MarkerSize',10, ...
    'LineWidth',3);


%% ---------------------------------------------------------
% WRIST AXES
%% ---------------------------------------------------------

frameLength = 35;


for i = 4:6

    Tjoint = getTransform( ...
        robot, ...
        qIKModel, ...
        bodyNamesIK{i}, ...
        robot.BaseName);


    origin = ...
        Tjoint(1:3,4);


    xAxis = ...
        Tjoint(1:3,1);

    yAxis = ...
        Tjoint(1:3,2);

    zAxis = ...
        Tjoint(1:3,3);


    % X
    quiver3( ...
        ikAxes, ...
        origin(1),origin(2),origin(3), ...
        xAxis(1),xAxis(2),xAxis(3), ...
        frameLength, ...
        'r', ...
        'LineWidth',2);


    % Y
    quiver3( ...
        ikAxes, ...
        origin(1),origin(2),origin(3), ...
        yAxis(1),yAxis(2),yAxis(3), ...
        frameLength, ...
        'g', ...
        'LineWidth',2);


    % Z
    quiver3( ...
        ikAxes, ...
        origin(1),origin(2),origin(3), ...
        zAxis(1),zAxis(2),zAxis(3), ...
        frameLength, ...
        'b', ...
        'LineWidth',3);

end


%% ---------------------------------------------------------
% VIEW
%% ---------------------------------------------------------

allPoints = [
    P;
    position;
    actualPosition
];


padding = 100;


xMin = min(allPoints(:,1)) - padding;
xMax = max(allPoints(:,1)) + padding;

yMin = min(allPoints(:,2)) - padding;
yMax = max(allPoints(:,2)) + padding;

zMin = min(allPoints(:,3)) - padding;
zMax = max(allPoints(:,3)) + padding;


if xMax-xMin < 300

    c = (xMin+xMax)/2;

    xMin = c-150;
    xMax = c+150;

end


if yMax-yMin < 300

    c = (yMin+yMax)/2;

    yMin = c-150;
    yMax = c+150;

end


if zMax-zMin < 300

    c = (zMin+zMax)/2;

    zMin = c-150;
    zMax = c+150;

end


xlim(ikAxes,[xMin xMax]);
ylim(ikAxes,[yMin yMax]);
zlim(ikAxes,[zMin zMax]);


view(ikAxes,135,25);

rotate3d(ikFigure,'on');

hold(ikAxes,'off');

drawnow;

end

%% =============================================================
% CONFIRM SINGLE-POINT MOVE
%
% For moveCartesian:
%
% confirmMove = false
%
% so this entire section is skipped.
%% =============================================================

if confirmMove

answer = input( ...
    '\nMove physical robot to this pose? (y/n): ', ...
    's');


if ~strcmpi(answer,'y')

    fprintf('\nMovement cancelled.\n');


    % Robot did not physically move, therefore return the old
    % robot state instead of the newly calculated IK state.

    qIKModel = ...
        qCurrent;


    return;

end

end

%% =============================================================
% EXECUTE SOLVED JOINT CONFIGURATION
%% =============================================================

qIKModel = moveToJoints( ...
    robotSerial, ...
    offsetDeg, ...
    controllerHomeDeg, ...
    qCurrent, ...
    qIKModel, ...
    confirmMove);

end
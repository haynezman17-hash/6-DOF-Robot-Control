function qTarget = moveToJoints( ...
    robotSerial, ...
    offsetDeg, ...
    controllerHomeDeg, ...
    qCurrent, ...
    qTarget, ...
    confirmMove)

% =============================================================
% COORDINATED 6-JOINT PHYSICAL MOVEMENT
%
% qCurrent and qTarget are MATLAB model configurations [rad].
%
% J0-J2:
%   Coordinated Marlin G1 stepper movement
%
% J3:
%   Wrist Yaw - direct drive, reversed direction
%
% J4:
%   Wrist Pitch - 2:1 pulley reduction
%   Servo rotates 2 deg for 1 deg physical pitch rotation
%
% J5:
%   Wrist Roll - direct drive
%
% Goal:
%   All six commanded trajectories start together and finish
%   at approximately the same time.
% =============================================================


%% =============================================================
% OPTIONAL CONFIRMATION
%% =============================================================

if nargin < 6 || isempty(confirmMove)
    confirmMove = true;
end


%% =============================================================
% VALIDATE CONFIGURATIONS
%% =============================================================

qCurrent = reshape(qCurrent,1,[]);
qTarget  = reshape(qTarget,1,[]);


if numel(qCurrent) ~= 6 || numel(qTarget) ~= 6

    error( ...
        'qCurrent and qTarget must each contain 6 joint values.');

end


if any(~isfinite(qCurrent)) || ...
   any(~isfinite(qTarget))

    error( ...
        'Joint configuration contains NaN or Inf.');

end


%% =============================================================
% DRIVE RATIOS
%
% Ratio definition:
%
% controller servo motion
% ----------------------- = DRIVE_RATIO
% physical joint motion
%
%% =============================================================

YAW_RATIO   = 1.0;      % direct drive
PITCH_RATIO = 2.0;      % 2:1 reduction
ROLL_RATIO  = 1.0;      % direct drive


%% =============================================================
% MODEL -> PHYSICAL FRAME ANGLES
%
% qModel = qFrame + offset
%
% therefore:
%
% qFrame = qModel - offset
%% =============================================================

currentFrameDeg = ...
    rad2deg(qCurrent) - offsetDeg;


targetFrameDeg = ...
    rad2deg(qTarget) - offsetDeg;


%% =============================================================
% PHYSICAL FRAME LIMITS
%
% IMPORTANT:
%
% Pitch servo limit = 0 to 135 deg.
%
% Because pitch is 2:1:
%
% 135 deg servo / 2
%       =
% 67.5 deg physical pitch
%% =============================================================

frameLimitsDeg = [

   -180    180;      % J0 Base

    -90     90;      % J1 Shoulder

    -90     90;      % J2 Elbow

      0    270;      % J3 Wrist Yaw
                     % Direct drive

      0    135;    % J4 Wrist Pitch
                     % 2:1 reduction

      0    180       % J5 Wrist Roll
                     % Direct drive

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


    if targetFrameDeg(i) < lowerLimit - FRAME_TOL || ...
       targetFrameDeg(i) > upperLimit + FRAME_TOL

        fprintf('\n');
        fprintf('=============================================\n');
        fprintf('             MOTION REJECTED\n');
        fprintf('=============================================\n\n');


        fprintf( ...
            'J%d requested frame angle = %.2f deg\n', ...
            i-1, ...
            targetFrameDeg(i));


        fprintf( ...
            'Allowed physical range = %.2f to %.2f deg\n\n', ...
            lowerLimit, ...
            upperLimit);


        error( ...
            'Target configuration exceeds physical joint limits.');

    end

end


%% =============================================================
% FRAME -> CONTROLLER
%% =============================================================

currentCmd = zeros(1,6);

targetCmd = zeros(1,6);


%% -------------------------------------------------------------
% J0 BASE
%
% Frame 0 -> controller X0
%% -------------------------------------------------------------

currentCmd(1) = ...
    controllerHomeDeg(1) + ...
    currentFrameDeg(1);


targetCmd(1) = ...
    controllerHomeDeg(1) + ...
    targetFrameDeg(1);


%% -------------------------------------------------------------
% J1 SHOULDER
%
% Frame 0 -> controller Y90
%% -------------------------------------------------------------

currentCmd(2) = ...
    controllerHomeDeg(2) + ...
    currentFrameDeg(2);


targetCmd(2) = ...
    controllerHomeDeg(2) + ...
    targetFrameDeg(2);


%% -------------------------------------------------------------
% J2 ELBOW
%
% Frame 0 -> controller Z90
%% -------------------------------------------------------------

currentCmd(3) = ...
    controllerHomeDeg(3) + ...
    currentFrameDeg(3);


targetCmd(3) = ...
    controllerHomeDeg(3) + ...
    targetFrameDeg(3);


%% -------------------------------------------------------------
% J3 WRIST YAW
%
% DIRECT DRIVE
%
% Yaw direction is reversed relative to the physical frame.
%
% Example:
%
% Frame 0 deg  -> S270
% Frame 30 deg -> S240
% Frame 90 deg -> S180
%% -------------------------------------------------------------

currentCmd(4) = ...
    controllerHomeDeg(4) - ...
    YAW_RATIO * currentFrameDeg(4);


targetCmd(4) = ...
    controllerHomeDeg(4) - ...
    YAW_RATIO * targetFrameDeg(4);


%% -------------------------------------------------------------
% J4 WRIST PITCH
%
% 2:1 PULLEY REDUCTION
%
% Servo rotates twice as far as physical pitch.
%
% Example:
%
% Physical pitch  0 deg   -> Servo S0
% Physical pitch 20 deg   -> Servo S40
% Physical pitch 45 deg   -> Servo S90
% Physical pitch 67.5 deg -> Servo S135
%% -------------------------------------------------------------

currentCmd(5) = ...
    controllerHomeDeg(5) + ...
    PITCH_RATIO * currentFrameDeg(5);


targetCmd(5) = ...
    controllerHomeDeg(5) + ...
    PITCH_RATIO * targetFrameDeg(5);


%% -------------------------------------------------------------
% J5 WRIST ROLL
%
% DIRECT DRIVE
%
% Example:
%
% Physical roll 0 deg  -> Servo S0
% Physical roll 45 deg -> Servo S45
% Physical roll 90 deg -> Servo S90
%% -------------------------------------------------------------

currentCmd(6) = ...
    controllerHomeDeg(6) + ...
    ROLL_RATIO * currentFrameDeg(6);


targetCmd(6) = ...
    controllerHomeDeg(6) + ...
    ROLL_RATIO * targetFrameDeg(6);


%% =============================================================
% CONTROLLER LIMITS
%% =============================================================

controllerLimitsDeg = [

   -180    180;      % J0 / X

      0    180;      % J1 / Y

      0    180;      % J2 / Z

      0    270;      % J3 Yaw servo

      0    270;      % J4 Pitch servo

      0    180       % J5 Roll servo

];


%% =============================================================
% CHECK CONTROLLER LIMITS
%% =============================================================

COMMAND_TOL = 0.5;


for i = 1:6

    if targetCmd(i) < ...
            controllerLimitsDeg(i,1) - COMMAND_TOL || ...
       targetCmd(i) > ...
            controllerLimitsDeg(i,2) + COMMAND_TOL

        fprintf('\n');
        fprintf('=============================================\n');
        fprintf('         CONTROLLER LIMIT REJECTED\n');
        fprintf('=============================================\n\n');


        fprintf( ...
            'J%d controller target = %.2f deg\n', ...
            i-1, ...
            targetCmd(i));


        fprintf( ...
            'Allowed controller range = %.2f to %.2f deg\n\n', ...
            controllerLimitsDeg(i,1), ...
            controllerLimitsDeg(i,2));


        error( ...
            'Controller target is outside physical limits.');

    end

end


%% =============================================================
% CLAMP SMALL FLOATING-POINT ERRORS
%% =============================================================

for i = 1:6

    targetCmd(i) = ...
        max( ...
        controllerLimitsDeg(i,1), ...
        min( ...
        controllerLimitsDeg(i,2), ...
        targetCmd(i)));

end


%% =============================================================
% CALCULATE MOVEMENT
%% =============================================================

deltaCmd = ...
    targetCmd - currentCmd;


absDeltaCmd = ...
    abs(deltaCmd);


%% =============================================================
% ALREADY AT TARGET?
%% =============================================================

if all(absDeltaCmd < 0.01)

    if confirmMove

        fprintf( ...
            '\nRobot is already at the requested joint configuration.\n');

    end

    return;

end


%% =============================================================
% DISPLAY / CONFIRM
%% =============================================================

if confirmMove

    fprintf('\n');
    fprintf('=============================================\n');
    fprintf('          COORDINATED JOINT MOVE\n');
    fprintf('=============================================\n\n');


    fprintf('Physical joint -> controller:\n\n');


    fprintf( ...
        'J0 Base      : %7.2f -> %7.2f\n', ...
        currentCmd(1), ...
        targetCmd(1));


    fprintf( ...
        'J1 Shoulder  : %7.2f -> %7.2f\n', ...
        currentCmd(2), ...
        targetCmd(2));


    fprintf( ...
        'J2 Elbow     : %7.2f -> %7.2f\n', ...
        currentCmd(3), ...
        targetCmd(3));


    fprintf( ...
        'J3 Yaw       : %7.2f -> %7.2f   [1:1]\n', ...
        currentCmd(4), ...
        targetCmd(4));


    fprintf( ...
        'J4 Pitch     : %7.2f -> %7.2f   [2:1]\n', ...
        currentCmd(5), ...
        targetCmd(5));


    fprintf( ...
        'J5 Roll      : %7.2f -> %7.2f   [1:1]\n\n', ...
        currentCmd(6), ...
        targetCmd(6));


    fprintf('Target physical frame angles:\n\n');

    fprintf( ...
        'J0 = %7.2f deg\n', ...
        targetFrameDeg(1));

    fprintf( ...
        'J1 = %7.2f deg\n', ...
        targetFrameDeg(2));

    fprintf( ...
        'J2 = %7.2f deg\n', ...
        targetFrameDeg(3));

    fprintf( ...
        'J3 = %7.2f deg\n', ...
        targetFrameDeg(4));

    fprintf( ...
        'J4 = %7.2f deg physical -> %.2f deg servo\n', ...
        targetFrameDeg(5), ...
        targetCmd(5));

    fprintf( ...
        'J5 = %7.2f deg\n\n', ...
        targetFrameDeg(6));


    answer = input( ...
        'Move physical robot? (y/n): ', ...
        's');


    if ~strcmpi(answer,'y')

        fprintf('\nMovement cancelled.\n');

        qTarget = qCurrent;

        return;

    end

end


%% =============================================================
% MOTION SETTINGS
%
% These values control how quickly the coordinated move occurs.
%% =============================================================

STEPPER_MAX_SPEED = 30;       % controller degrees / second

SERVO_MAX_SPEED = 45;         % servo degrees / second

SERVO_DT = 0.020;             % 20 ms update period

MIN_MOVE_TIME = 0.25;         % minimum move duration

MAX_FEED_RATE = 3000;         % Marlin units/min


%% =============================================================
% CALCULATE REQUIRED MOVE TIME FOR EACH AXIS
%
% IMPORTANT:
%
% Pitch delta is already expressed in SERVO degrees here.
%
% Example:
%
% 30 deg physical pitch
%       ->
% 60 deg servo travel
%
% Therefore the servo timing automatically accounts for the
% 2:1 pulley ratio.
%% =============================================================

requiredTime = zeros(1,6);


requiredTime(1:3) = ...
    absDeltaCmd(1:3) ./ STEPPER_MAX_SPEED;


requiredTime(4:6) = ...
    absDeltaCmd(4:6) ./ SERVO_MAX_SPEED;


%% =============================================================
% COMMON MOVE TIME
%
% The joint requiring the longest movement determines the
% duration for the complete six-axis movement.
%% =============================================================

MOVE_TIME = ...
    max(requiredTime);


MOVE_TIME = ...
    max( ...
    MOVE_TIME, ...
    MIN_MOVE_TIME);


%% =============================================================
% STEPPER DISTANCE
%% =============================================================

stepperVector = ...
    deltaCmd(1:3);


stepperDistance = ...
    norm(stepperVector);


%% =============================================================
% CALCULATE STEPPER FEED RATE
%
% F is controller units/min.
%% =============================================================

if stepperDistance > 1e-6

    feedRate = ...
        stepperDistance * 60 / MOVE_TIME;


    feedRate = ...
        min( ...
        feedRate, ...
        MAX_FEED_RATE);


    feedRate = ...
        max( ...
        feedRate, ...
        1);

else

    feedRate = 1;

end


%% =============================================================
% SERVO TRAJECTORY SETTINGS
%% =============================================================

numServoSteps = ...
    max( ...
    1, ...
    ceil(MOVE_TIME / SERVO_DT));


actualServoDT = ...
    MOVE_TIME / numServoSteps;


%% =============================================================
% DISPLAY TRAJECTORY INFORMATION
%% =============================================================

if confirmMove

    fprintf('\n');
    fprintf('=============================================\n');
    fprintf('          MOTION TIMING\n');
    fprintf('=============================================\n\n');


    fprintf( ...
        'Move duration      : %.3f s\n', ...
        MOVE_TIME);


    fprintf( ...
        'Stepper feed       : %.1f units/min\n', ...
        feedRate);


    fprintf( ...
        'Servo updates      : %d\n', ...
        numServoSteps);


    fprintf( ...
        'Servo update period: %.2f ms\n\n', ...
        actualServoDT*1000);

end


%% =============================================================
% ENABLE STEPPERS
%% =============================================================

writeline( ...
    robotSerial, ...
    "M17");


writeline( ...
    robotSerial, ...
    "M18 S0");


writeline( ...
    robotSerial, ...
    "G90");


%% =============================================================
% SYNCHRONIZE CURRENT SERVO COMMANDS
%
% Ensure MATLAB's assumed starting servo position matches the
% controller command before starting the coordinated trajectory.
%% =============================================================

writeline( ...
    robotSerial, ...
    sprintf( ...
    "M280 P0 S%.1f", ...
    currentCmd(4)));


writeline( ...
    robotSerial, ...
    sprintf( ...
    "M280 P1 S%.1f", ...
    currentCmd(5)));


writeline( ...
    robotSerial, ...
    sprintf( ...
    "M280 P2 S%.1f", ...
    currentCmd(6)));


pause(0.05);


%% =============================================================
% PREPARE STEPPER COMMAND
%% =============================================================

stepperCmd = sprintf( ...
    "G1 X%.3f Y%.3f Z%.3f F%.1f", ...
    targetCmd(1), ...
    targetCmd(2), ...
    targetCmd(3), ...
    feedRate);


if confirmMove

    fprintf( ...
        'Stepper command:\n%s\n\n', ...
        stepperCmd);

end


%% =============================================================
% START STEPPER MOVEMENT
%
% IMPORTANT:
%
% NO M400 HERE.
%
% The stepper motion begins, then the servo trajectory starts
% immediately.
%% =============================================================

writeline( ...
    robotSerial, ...
    stepperCmd);


%% =============================================================
% COORDINATED WRIST TRAJECTORY
%
% All three wrist joints use the same normalized trajectory
% variable so their COMMANDS reach their targets together.
%
% Smoothstep:
%
% sSmooth = 3s^2 - 2s^3
%
% gives softer acceleration and deceleration than a linear ramp.
%% =============================================================

trajectoryTimer = tic;


for k = 1:numServoSteps

    %% Normalized trajectory position

    s = ...
        k / numServoSteps;


    %% Smoothstep profile

    sSmooth = ...
        3*s^2 - 2*s^3;


    %% ---------------------------------------------------------
    % J3 YAW
    %% ---------------------------------------------------------

    yawCmd = ...
        currentCmd(4) + ...
        sSmooth * deltaCmd(4);


    %% ---------------------------------------------------------
    % J4 PITCH
    %
    % Already includes the 2:1 pulley ratio because currentCmd
    % and targetCmd are servo-side values.
    %% ---------------------------------------------------------

    pitchCmd = ...
        currentCmd(5) + ...
        sSmooth * deltaCmd(5);


    %% ---------------------------------------------------------
    % J5 ROLL
    %% ---------------------------------------------------------

    rollCmd = ...
        currentCmd(6) + ...
        sSmooth * deltaCmd(6);


    %% ---------------------------------------------------------
    % CLAMP COMMANDS
    %% ---------------------------------------------------------

    yawCmd = ...
        max( ...
        0, ...
        min( ...
        270, ...
        yawCmd));


    pitchCmd = ...
        max( ...
        0, ...
        min( ...
        270, ...
        pitchCmd));


    rollCmd = ...
        max( ...
        0, ...
        min( ...
        180, ...
        rollCmd));


    %% ---------------------------------------------------------
    % SEND YAW
    %% ---------------------------------------------------------

    writeline( ...
        robotSerial, ...
        sprintf( ...
        "M280 P0 S%.1f", ...
        yawCmd));


    %% ---------------------------------------------------------
    % SEND PITCH
    %% ---------------------------------------------------------

    writeline( ...
        robotSerial, ...
        sprintf( ...
        "M280 P1 S%.1f", ...
        pitchCmd));


    %% ---------------------------------------------------------
    % SEND ROLL
    %% ---------------------------------------------------------

    writeline( ...
        robotSerial, ...
        sprintf( ...
        "M280 P2 S%.1f", ...
        rollCmd));


    %% ---------------------------------------------------------
    % MAINTAIN TRAJECTORY TIMING
    %% ---------------------------------------------------------

    desiredTime = ...
        k * actualServoDT;


    elapsedTime = ...
        toc(trajectoryTimer);


    remainingTime = ...
        desiredTime - elapsedTime;


    if remainingTime > 0

        pause(remainingTime);

    end

end


%% =============================================================
% FORCE EXACT FINAL SERVO COMMANDS
%% =============================================================

writeline( ...
    robotSerial, ...
    sprintf( ...
    "M280 P0 S%.1f", ...
    targetCmd(4)));


writeline( ...
    robotSerial, ...
    sprintf( ...
    "M280 P1 S%.1f", ...
    targetCmd(5)));


writeline( ...
    robotSerial, ...
    sprintf( ...
    "M280 P2 S%.1f", ...
    targetCmd(6)));


%% =============================================================
% WAIT FOR STEPPER MOVEMENT TO COMPLETE
%% =============================================================

writeline( ...
    robotSerial, ...
    "M400");


%% =============================================================
% COMPLETE
%% =============================================================

if confirmMove

    actualTime = ...
        toc(trajectoryTimer);


    fprintf('\n');
    fprintf('=============================================\n');
    fprintf('       COORDINATED MOVE COMPLETE\n');
    fprintf('=============================================\n\n');


    fprintf( ...
        'Requested duration : %.3f s\n', ...
        MOVE_TIME);


    fprintf( ...
        'Servo loop time    : %.3f s\n\n', ...
        actualTime);

end

end
function moveXYZ(varargin)

% =============================================================
% ROBOT POSITION COMMAND
%
% Usage:
%
%   moveXYZ("home")
%       Full physical homing procedure.
%
%   moveXYZ("ready")
%       Return to post-homing pose WITHOUT running G28.
%
%   moveXYZ(X,Y,Z)
%       Cartesian point-to-point move.
% =============================================================


%% =============================================================
% MASTER VARIABLES
%% =============================================================

robot = evalin('base','robot');
offsetDeg = evalin('base','offsetDeg');
controllerHomeDeg = evalin('base','controllerHomeDeg');
qHomeModel = evalin('base','qHomeModel');


%% =============================================================
% SERIAL CONNECTION
%% =============================================================

robotSerial = [];


try

    existingSerial = ...
        evalin('base','robotSerial');

    if ~isempty(existingSerial) && isvalid(existingSerial)
        robotSerial = existingSerial;
    end

catch
end


if isempty(robotSerial)

    SERIAL_PORT = "COM4";
    BAUD_RATE   = 115200;

    availablePorts = ...
        serialportlist("available");


    if ~any(strcmpi(availablePorts,SERIAL_PORT))

        error( ...
            '%s is not available.', ...
            SERIAL_PORT);

    end


    fprintf( ...
        '\nConnecting to robot on %s...\n', ...
        SERIAL_PORT);


    robotSerial = ...
        serialport( ...
        SERIAL_PORT, ...
        BAUD_RATE);

    configureTerminator( ...
        robotSerial, ...
        "LF");

    pause(2);

    flush(robotSerial);


    writeline(robotSerial,"M17");
    writeline(robotSerial,"M18 S0");


    assignin( ...
        'base', ...
        'robotSerial', ...
        robotSerial);


    fprintf('Robot connected.\n');

end


%% =============================================================
% TEXT COMMANDS
%% =============================================================

if nargin == 1 && ...
   (ischar(varargin{1}) || isstring(varargin{1}))

    command = ...
        lower(string(varargin{1}));


    %% =========================================================
    % HOME
    %% =========================================================

    if command == "home"

        fprintf('\n');
        fprintf('=============================================\n');
        fprintf('              HOMING ROBOT\n');
        fprintf('=============================================\n\n');


        writeline(robotSerial,"M17");
        writeline(robotSerial,"M18 S0");


        %% ELBOW / Z

        fprintf('Homing elbow / Z...\n');

        writeline(robotSerial,"G28 Z");
        writeline(robotSerial,"M400");

        writeline(robotSerial,"G90");
        writeline(robotSerial,"G1 Z180 F3000");
        writeline(robotSerial,"M400");


        %% SHOULDER / Y

        fprintf('Homing shoulder / Y...\n');

        writeline(robotSerial,"G28 Y");
        writeline(robotSerial,"M400");

        writeline(robotSerial,"G90");
        writeline(robotSerial,"G1 Y90 Z90 F3000");
        writeline(robotSerial,"M400");


        %% BASE / X

        fprintf('Homing base / X...\n');

        writeline(robotSerial,"G28 X");
        writeline(robotSerial,"M400");


        %% WRIST

        fprintf('Positioning wrist...\n');

        writeline(robotSerial, ...
            sprintf( ...
            "M280 P0 S%.1f", ...
            controllerHomeDeg(4)));

        writeline(robotSerial, ...
            sprintf( ...
            "M280 P1 S%.1f", ...
            controllerHomeDeg(5)));

        writeline(robotSerial, ...
            sprintf( ...
            "M280 P2 S%.1f", ...
            controllerHomeDeg(6)));

        pause(1);


        %% RESET STATE

        qCurrent = ...
            reshape(qHomeModel,1,[]);


        assignin( ...
            'base', ...
            'qCurrent', ...
            qCurrent);


        assignin( ...
            'base', ...
            'robotSerial', ...
            robotSerial);


        %% REPORT HOME TCP

        Thome = getTransform( ...
            robot, ...
            qCurrent, ...
            'tool0', ...
            robot.BaseName);

        homeXYZ = ...
            tform2trvec(Thome);


        fprintf('\nRobot homing complete.\n');

        fprintf( ...
            'Home TCP = [%.2f %.2f %.2f] mm\n\n', ...
            homeXYZ(1), ...
            homeXYZ(2), ...
            homeXYZ(3));

        return;

    end


    %% =========================================================
    % READY
    %% =========================================================

    if command == "ready"

        fprintf('\n');
        fprintf('=============================================\n');
        fprintf('          RETURNING TO READY\n');
        fprintf('=============================================\n\n');


        %% Current state must be known

        try

            qCurrent = ...
                evalin('base','qCurrent');

        catch

            error( ...
                ['Robot position is unknown. ' ...
                 'Run moveXYZ("home") first.']);

        end


        qCurrent = ...
            reshape(qCurrent,1,[]);


        %% Ready is exactly home configuration

        qReady = ...
            reshape(qHomeModel,1,[]);


        %% Coordinated move - NO homing switches

        qReady = moveToJoints( ...
            robotSerial, ...
            offsetDeg, ...
            controllerHomeDeg, ...
            qCurrent, ...
            qReady, ...
            true);


        %% Update state

        qCurrent = qReady;


        assignin( ...
            'base', ...
            'qCurrent', ...
            qCurrent);


        %% Report

        Tready = getTransform( ...
            robot, ...
            qCurrent, ...
            'tool0', ...
            robot.BaseName);

        readyXYZ = ...
            tform2trvec(Tready);


        fprintf( ...
            'READY TCP = [%.2f %.2f %.2f] mm\n\n', ...
            readyXYZ(1), ...
            readyXYZ(2), ...
            readyXYZ(3));

        return;

    end


    error( ...
        ['Unknown command.' newline newline ...
         'Use:' newline ...
         'moveXYZ("home")' newline ...
         'moveXYZ("ready")' newline ...
         'moveXYZ(X,Y,Z)']);

end


%% =============================================================
% XYZ COMMAND
%% =============================================================

if nargin ~= 3

    error( ...
        ['Use:' newline ...
         'moveXYZ("home")' newline ...
         'moveXYZ("ready")' newline ...
         'moveXYZ(X,Y,Z)']);

end


x = double(varargin{1});
y = double(varargin{2});
z = double(varargin{3});


if ~isscalar(x) || ...
   ~isscalar(y) || ...
   ~isscalar(z) || ...
   ~all(isfinite([x y z]))

    error( ...
        'X, Y and Z must be finite scalar values.');

end


targetPosition = ...
    [x y z];


%% =============================================================
% CURRENT STATE
%% =============================================================

try

    qCurrent = ...
        evalin('base','qCurrent');

catch

    error( ...
        ['Robot position is unknown. ' ...
         'Run moveXYZ("home") first.']);

end


qCurrent = ...
    reshape(qCurrent,1,[]);


if numel(qCurrent) ~= 6

    error( ...
        'qCurrent must contain 6 joint values.');

end


%% =============================================================
% DISPLAY
%% =============================================================

Tstart = getTransform( ...
    robot, ...
    qCurrent, ...
    'tool0', ...
    robot.BaseName);

startXYZ = ...
    tform2trvec(Tstart);


fprintf('\n');
fprintf('=============================================\n');
fprintf('                XYZ MOVE\n');
fprintf('=============================================\n\n');


fprintf( ...
    'Start  : [%.2f %.2f %.2f] mm\n', ...
    startXYZ(1), ...
    startXYZ(2), ...
    startXYZ(3));


fprintf( ...
    'Target : [%.2f %.2f %.2f] mm\n\n', ...
    targetPosition(1), ...
    targetPosition(2), ...
    targetPosition(3));


%% =============================================================
% IK + COORDINATED MOVEMENT
%% =============================================================

[~,qSolved] = moveToPose( ...
    robotSerial, ...
    robot, ...
    offsetDeg, ...
    controllerHomeDeg, ...
    targetPosition, ...
    qCurrent, ...
    true);


qCurrent = ...
    reshape(qSolved,1,[]);


%% =============================================================
% UPDATE SHARED STATE
%% =============================================================

assignin( ...
    'base', ...
    'qCurrent', ...
    qCurrent);

assignin( ...
    'base', ...
    'robotSerial', ...
    robotSerial);


%% =============================================================
% REPORT
%% =============================================================

Tfinal = getTransform( ...
    robot, ...
    qCurrent, ...
    'tool0', ...
    robot.BaseName);

finalXYZ = ...
    tform2trvec(Tfinal);


fprintf('\nXYZ move complete.\n');

fprintf( ...
    'Model TCP = [%.2f %.2f %.2f] mm\n\n', ...
    finalXYZ(1), ...
    finalXYZ(2), ...
    finalXYZ(3));

end
%% ============================================================
% 14. TCP WORKSPACE GENERATION
%
% Samples the physical joint limits and calculates the reachable
% position of tool0 (TCP) using the existing rigidBodyTree.
%
% IMPORTANT:
%   - Does NOT modify the robot model
%   - Does NOT modify joint limits
%   - Does NOT modify the home visualization
%   - Uses the existing TCP offset
%% ============================================================

fprintf('\n');
fprintf('=============================================\n');
fprintf('          GENERATING TCP WORKSPACE\n');
fprintf('=============================================\n\n');


%% ------------------------------------------------------------
% WORKSPACE SAMPLING RESOLUTION
%
% Number of samples for J0-J2.
% Increasing these gives a denser workspace but increases
% computation time.
%% ------------------------------------------------------------

nJ0 = 31;
nJ1 = 31;
nJ2 = 31;


%% ------------------------------------------------------------
% ARM JOINT SAMPLE VECTORS
%
% Workspace position is dominated by:
%
%   J0 = Base
%   J1 = Shoulder
%   J2 = Elbow
%
% Wrist joints are initially held at their home configuration.
%% ------------------------------------------------------------

q0Samples = linspace( ...
    modelLimitsDeg(1,1), ...
    modelLimitsDeg(1,2), ...
    nJ0);

q1Samples = linspace( ...
    modelLimitsDeg(2,1), ...
    modelLimitsDeg(2,2), ...
    nJ1);

q2Samples = linspace( ...
    modelLimitsDeg(3,1), ...
    modelLimitsDeg(3,2), ...
    nJ2);


%% ------------------------------------------------------------
% PREALLOCATE WORKSPACE ARRAY
%% ------------------------------------------------------------

numWorkspacePoints = ...
    nJ0 * nJ1 * nJ2;

workspaceXYZ = ...
    zeros(numWorkspacePoints,3);

pointIndex = 1;


%% ------------------------------------------------------------
% GENERATE REACHABLE TCP POSITIONS
%% ------------------------------------------------------------

for i0 = 1:nJ0

    for i1 = 1:nJ1

        for i2 = 1:nJ2

            % Start from existing home configuration
            qWorkspace = qHomeModel;

            % Replace J0-J2 with sampled model angles
            qWorkspace(1) = deg2rad(q0Samples(i0));
            qWorkspace(2) = deg2rad(q1Samples(i1));
            qWorkspace(3) = deg2rad(q2Samples(i2));

            % Forward kinematics to TCP
            Tworkspace = getTransform( ...
                robot, ...
                qWorkspace, ...
                'tool0', ...
                robot.BaseName);

            % Save TCP XYZ position
            workspaceXYZ(pointIndex,:) = ...
                Tworkspace(1:3,4)';

            pointIndex = pointIndex + 1;

        end

    end

end


%% ============================================================
% 15. WORKSPACE STATISTICS
%% ============================================================

workspaceX = workspaceXYZ(:,1);
workspaceY = workspaceXYZ(:,2);
workspaceZ = workspaceXYZ(:,3);


fprintf('Workspace points generated: %d\n\n', ...
    size(workspaceXYZ,1));

fprintf('TCP workspace limits:\n');

fprintf( ...
    'X: %.2f to %.2f mm\n', ...
    min(workspaceX), ...
    max(workspaceX));

fprintf( ...
    'Y: %.2f to %.2f mm\n', ...
    min(workspaceY), ...
    max(workspaceY));

fprintf( ...
    'Z: %.2f to %.2f mm\n', ...
    min(workspaceZ), ...
    max(workspaceZ));


%% ------------------------------------------------------------
% MAXIMUM RADIAL REACH
%% ------------------------------------------------------------

radialReach = ...
    sqrt(workspaceX.^2 + workspaceY.^2);

fprintf( ...
    '\nMaximum horizontal radial reach: %.2f mm\n', ...
    max(radialReach));

fprintf( ...
    'Maximum distance from base origin: %.2f mm\n', ...
    max(vecnorm(workspaceXYZ,2,2)));


%% ============================================================
% 16. PLOT TCP WORKSPACE
%% ============================================================

figure( ...
    'Name','Robot TCP Workspace', ...
    'NumberTitle','off', ...
    'Color','white');

hold on;
grid on;
axis equal;
axis vis3d;


%% ------------------------------------------------------------
% WORKSPACE POINT CLOUD
%% ------------------------------------------------------------

scatter3( ...
    workspaceX, ...
    workspaceY, ...
    workspaceZ, ...
    5, ...
    workspaceZ, ...
    'filled');


%% ------------------------------------------------------------
% MARK BASE
%% ------------------------------------------------------------

plot3( ...
    0, ...
    0, ...
    0, ...
    'ko', ...
    'MarkerSize',10, ...
    'MarkerFaceColor','k');

text( ...
    0, ...
    0, ...
    0, ...
    '  BASE', ...
    'FontSize',11, ...
    'FontWeight','bold');


%% ------------------------------------------------------------
% MARK HOME TCP POSITION
%% ------------------------------------------------------------

plot3( ...
    tcpHome(1), ...
    tcpHome(2), ...
    tcpHome(3), ...
    'rp', ...
    'MarkerSize',14, ...
    'MarkerFaceColor','r');

text( ...
    tcpHome(1), ...
    tcpHome(2), ...
    tcpHome(3), ...
    '  HOME TCP', ...
    'FontSize',11, ...
    'FontWeight','bold');


%% ------------------------------------------------------------
% WORKSPACE PLOT FORMATTING
%% ------------------------------------------------------------

title('6-DOF Robot TCP Reachable Workspace');

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

colorbar;

view(135,25);


%% ------------------------------------------------------------
% AUTOMATIC WORKSPACE PLOT LIMITS
%% ------------------------------------------------------------

workspacePadding = 50;

xlim([ ...
    min(workspaceX)-workspacePadding, ...
    max(workspaceX)+workspacePadding]);

ylim([ ...
    min(workspaceY)-workspacePadding, ...
    max(workspaceY)+workspacePadding]);

zlim([ ...
    min(workspaceZ)-workspacePadding, ...
    max(workspaceZ)+workspacePadding]);


%% ============================================================
% 17. TOP VIEW OF WORKSPACE
%% ============================================================

figure( ...
    'Name','Robot Workspace - Top View', ...
    'NumberTitle','off', ...
    'Color','white');

scatter( ...
    workspaceX, ...
    workspaceY, ...
    5, ...
    workspaceZ, ...
    'filled');

hold on;
grid on;
axis equal;

plot( ...
    0, ...
    0, ...
    'ko', ...
    'MarkerSize',10, ...
    'MarkerFaceColor','k');

title('TCP Workspace - Top View');

xlabel('X (mm)');
ylabel('Y (mm)');

colorbar;


%% ============================================================
% 18. SIDE VIEW OF WORKSPACE
%% ============================================================

figure( ...
    'Name','Robot Workspace - Side View', ...
    'NumberTitle','off', ...
    'Color','white');

scatter( ...
    workspaceX, ...
    workspaceZ, ...
    5, ...
    workspaceY, ...
    'filled');

hold on;
grid on;
axis equal;

plot( ...
    0, ...
    0, ...
    'ko', ...
    'MarkerSize',10, ...
    'MarkerFaceColor','k');

title('TCP Workspace - Side View');

xlabel('X (mm)');
ylabel('Z (mm)');

colorbar;


fprintf('\nWorkspace generation complete.\n');

rotate3d on;

hold off;


%% ============================================================
% 14. READY
%% ============================================================

fprintf('\n');
fprintf('=============================================\n');
fprintf('          KINEMATIC MODEL READY\n');
fprintf('=============================================\n\n');

fprintf('Home robot:\n');
fprintf('    moveXYZ("home")\n\n');

fprintf('Move robot:\n');
fprintf('    moveXYZ(X,Y,Z)\n\n');
%% vroomMFvroom.m
% Let's get it

%% Extraction and Definitions
% Read track data from the DXF
dxf = DXFtool('racetrack.dxf'); 
clc; close all; % Stop dxf from automatically plotting

% List imported entities
dxf.list;

%% Plot Racetrack
% Extract track points from the DXF
dxf.entities(2).spline;
trackpoints = zeros(199, 2); % Track storing 199x2 vector
trackpoints(:, 1) = ans.X;
trackpoints(:, 2) = ans.Y;

% Compute the normals for width plot
track_width = 15;

N = size(trackpoints, 1);
extended_track_left = zeros(N, 2);
extended_track_right = zeros(N, 2);

for i = 1:N
    % Find tangent with difference between points
    tangent = trackpoints(mod(i,N)+1, :) - trackpoints(i, :);
    
    % Edge case
    if all(tangent == 0) && i > 1
        tangent = trackpoints(i, :) - trackpoints(i-1, :);
    end

    % Normalize tangent vector
    tangent = tangent / norm(tangent);
    
    % Get normal vector
    normal = [-tangent(2), tangent(1)];
   
    % Create boundary lines
    extended_track_left(i, :) = trackpoints(i, :) + (track_width / 2) * normal;
    extended_track_right(i, :) = trackpoints(i, :) - (track_width / 2) * normal;
end

% {
% Plot original track and defined width
figure;
plot(trackpoints(:, 1), trackpoints(:, 2), 'k--', ...  
     extended_track_left(:, 1), extended_track_left(:, 2), 'r-', ... 
     extended_track_right(:, 1), extended_track_right(:, 2), 'b-'); 
%plot(optimized_path(:, 1), optimized_path(:, 2), 'g-');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Racetrack Layout');
legend('Track', 'Outside Boundary', 'Inside Boundary', 'Optimized Track')
axis equal;
grid on;
%}
%% Plot Curve Radii
radii_curves = zeros(N, 1);

for i = 2:(N-1)
    % Find dy/dx = (yp-yn) / (xp-xn)
    dy_dx = (trackpoints(i+1, 2) - trackpoints(i-1, 2)) / (trackpoints(i+1, 1) - trackpoints(i-1, 1));
    
    % Find d2y/dx2 = (yp-2yi+yn) / ((xp-xi)*(xi-xn))
    d2y_dx2 = (trackpoints(i+1, 2) - 2*trackpoints(i, 2) + trackpoints(i-1, 2)) / ((trackpoints(i+1, 1) - trackpoints(i, 1))*(trackpoints(i, 1) - trackpoints(i-1, 1)));
    
    % Find R = [1+(dy/dx)^2]^(3/2) / |(d2y/dx2)|
    curvature = (1 + dy_dx^2)^(3/2) / abs(d2y_dx2);
   
    % Erroneous curves (needs work)
    radii_curves(i) = curvature;
end

% Edge case
radii_curves(1) = radii_curves(2);
radii_curves(N) = radii_curves(N-1);
% {
% Plot radii of curvature
figure;
x = 1:N;
y = radii_curves;
semilogy(x,y);
xlabel('Point Index');
ylabel('Radius of Curvature (m)');
title('Radii of Racetrack Curvatures');
legend('Radius')
grid on;
%}
%% Plot Rider Activity
% Dynamics
simulation_duration = 1000; % Total simulation time (s)
time_step = 5; % Time step (s)

% Plotting setup
figure; hold on; grid on; axis equal;
plot(trackpoints(:, 1), trackpoints(:, 2), 'k--', ...  
     extended_track_left(:, 1), extended_track_left(:, 2), 'r-', ... 
     extended_track_right(:, 1), extended_track_right(:, 2), 'b-'); 
h = plot(trackpoints(1, 1), trackpoints(1, 2), 'o', 'MarkerFaceColor', 'k'); % Plot initial position of the bike
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Zoom Zoom');
legend('Track', 'Outside Boundary', 'Inside Boundary', 'Bike')

% Variables
current_velocity = [0; 0]; 
current_position = trackpoints(1, :); 
bike_path = current_position; 

max_speed = 50; % m/s
max_centripetal_acceleration = 2; % m/s^2
max_tangential_acceleration = 5; % m/s^2

% Simulation loop
for t = 0:time_step:simulation_duration
    
    % Find closest matching point
    [~, closest_index] = min(vecnorm(trackpoints - current_position(1,:), 2, 2));
    
    % Find direction to nearest point
    if closest_index < size(trackpoints, 1)
        next_point = trackpoints(closest_index + 1, :);
    else
        next_point = trackpoints(1, :); 
    end
    
    direction_to_next_point = next_point - current_position;
    distance_to_next_point = norm(direction_to_next_point);
    direction_to_next_point = direction_to_next_point / distance_to_next_point; % Normalize

    % Find velocity based on radius
    radius_of_curvature = radii_curves(closest_index);
    if radius_of_curvature > 0
        max_speed_for_curvature = sqrt(max_centripetal_acceleration * radius_of_curvature);
    else
        max_speed_for_curvature = max_speed;
    end
    
    % Acceleration
    speed = norm(current_velocity);
    if speed < max_speed_for_curvature
        acceleration = min(max_tangential_acceleration, (max_speed_for_curvature - speed) / time_step);
        speed = speed + acceleration * time_step;
    else
        % Maintain speed
        speed = max_speed_for_curvature;
    end
    
    % Update velocity and position
    current_velocity = direction_to_next_point * speed;
    new_position = current_position + current_velocity * time_step;
    
    % Check if new position overshoots next point and adjust
    if norm(new_position - current_position) > distance_to_next_point
        new_position = next_point; 
        current_velocity = [0; 0]; 
    end
    
    % Store and update positions
    bike_path = [bike_path; new_position];
    current_position = new_position;
    
    % Plot current position
    set(h, 'XData', current_position(1), 'YData', current_position(2)); 
    drawnow;
    
    % Check for end of simulation
    if t + time_step >= simulation_duration
        break;
    end
end

% Hold off the plot
hold off;

%% List of More Considerations
% Tire grip
% Friction
% Rider reaction (change of apex)
% Aerodynamics
% Weather
% Engine
% Battery consumption
% Tire wear

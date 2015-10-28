function cover_grid(serPort)

    % FOR SIMULATOR, SET real = 0;
    % FOR PHYSICAL ROOMBA, SET real=1;
    real = 0;
    eps_x = 0.1; 
    eps_y = 0.15;  
    
    % Two parameter sets for simulator and roomba
    if real == 1
        pause_time = 0.4;  
        speed = .1; 
    else
        pause_time = 1;  % probably need to increase
        speed = 0.1; 
    end

    % Start clock
    tic;
    since_update = toc;
    curr_position = position(0, 0, 0);
    new_position = curr_position;
        
    % Start loop to navigate, stop after not updating in 30 seconds
    while since_update < 15
        
        % Set random turn angle
        %rng(5) % allows for predictable random number
        rad_angle = rand + rand;
        pos = randi(1,2);
        if pos == 1
            rad_angle = rad_angle * -1;     
        end        
            
        % Initialize distance & angle sensor
        [Distance] = DistanceSensorRoomba(serPort);
        [AngleR] = AngleSensorRoomba(serPort);
        % DO WE NEED THIS new_position = new_position * position(AngleR, Distance, 0);
        
        % Move
        SetFwdVelRadiusRoomba(serPort, speed, rad_angle);
        pause(pause_time);
        
        % MIGHT NEED TO STOP WHEELS
        % Get positions, will need for locating on occupany grid
        [Distance] = DistanceSensorRoomba(serPort); 
        [AngleR]=AngleSensorRoomba(serPort); 
        new_position = new_position * position(AngleR, Distance, 0);
        
        % *********************************
        % NEED TO UPDATE LOCATION IN GRID
        
        % Check for bump
        [r, l, dr, dl, dc, f] = BumpsWheelDropsSensorsRoomba(serPort); 
        if l == 1 || f == 1 || r == 1
            % Set block value to blocked
            tic;
            % move around obstacles
            break; % use for now
        % else if block value isn't already free
            % Set block value to free
            %tic;
        else % block has already been traversed, so no update, and don't change the grid block value
            
        end
        
        % **********************************
        
        since_update = toc;

    end
    
    display('Bumped or its been 15 seconds')

end

% ************************* Position ********************************

function a = position(theta, x,y)
    a = eye(4);
    a(1,1) = cos(theta);
    a(1,2) = -sin(theta);
    a(2,1) = sin(theta);
    a(2,2) = cos(theta); 
    a(1,4) = x;
    a(2,4) = y;
end

% ****************** Move around the obstacle ************************

function [new_position, stop, pos_cache] = move_around_obstacle(serPort, curr_position, eps_y, real, pos_cache)
    
    % Reset distance_traveled (around obstacle) to 0
    distance_traveled = 0;
    stop = 0;
    
    % Two sets of parameters
    if real == 1
        pause_time = 0.4; 
        obstacle_goal = 4.0; 
        speed = .1; 
    else
        pause_time = 0.1; 
        obstacle_goal = 4.0; 
        speed = 0.05; 
    end
    
    % Save coordinates for intial bump, to determine when to stop 
    start = [curr_position(1,4), curr_position(2,4)];
    
    % Until we are back on the line, continue navigating the obstacle
    while back_on_line == 0
        
        % Turn roomba until parallel with wall
        [curr_position, distance_traveled, same_pos, pos_cache] = ...
            handle_turn(serPort, curr_position, distance_traveled, start, real, pos_cache)
        
        % If this is the starting position, break out and stop moving
        if(same_pos == 1)
            stop = 1;
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            pause(pause_time);
            break;
        end
        
        % Otherwise follow the wall, until a bump or until no roomba is no
        % longer parallel
        [curr_position, distance_traveled, back_on_line, same_pos, pos_cache] = ...
            follow_straight_wall(serPort, eps_y, curr_position, distance_traveled, start, real, pos_cache);
        
        % If this is the starting position, break out and stop moving        
        if(same_pos == 1)
            stop = 1;
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            pause(pause_time);
            break;
        end
        
    end  
    
    % If we are back on the path, orient roomba to face the correct
    % position
    if back_on_line
        back_on_line
        [curr_position, pos_cache] = turn_back_onto_line(serPort, curr_position, acos(curr_position(1,1)), pos_cache);
        pause(pause_time/2.0);
    end
    
    % Update position
    new_position = curr_position;
end

% ************************ Handle turn *****************************

function [new_position, distance_traveled, same_pos, pos_cache] = ...
    handle_turn(serPort, curr_position, distance_traveled, start, real, pos_cache)
    new_position = curr_position;
    start_x = start(1,1);
    start_y = start(1,2);
    same_pos = 0;
    
    if real == 1
        pause_time = 0.1; 
        angle = 10;
        speed = .025; 
    else
        pause_time = 0.1; 
        angle = 5;
        speed = 0.2; 
    end
    
    w = WallSensorReadRoomba(serPort);
    while isnan(w)
        w = WallSensorReadRoomba(serPort);
    end
    
    [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
            BumpsWheelDropsSensorsRoomba(serPort);
        
    if(abs(curr_position(1,4) - start_x) < 0.3 && abs(curr_position(2,4) - start_y) < 0.3 && distance_traveled > 0.5)
        same_pos = 1;
    else
        if f == 1 || l==1 % front or left
             display ('turning left'); 
             [l, f, r, w, new_position, distance_traveled, pos_cache] = ...
                turn_left_until_parallel(serPort, curr_position, distance_traveled, real, pos_cache);

        elseif r == 1 % right bumper
            display ('turning left');  
             [AngleR] = AngleSensorRoomba(serPort); 
             turnAngle(serPort, speed, angle);
             pause(pause_time); 
             [AngleR]=AngleSensorRoomba(serPort); 
             new_position = curr_position * position(AngleR, 0, 0);        
             pos_cache = [pos_cache; new_position(2,4), new_position(1,4)];
        elseif w == 0 % lost wall
            display ('turning right'); 
            [l, f, r, w, new_position, distance_traveled, pos_cache] = ...
                turn_right_until_wall(serPort, curr_position, distance_traveled, real, pos_cache);
        end
    end
end

% ************************ Follow wall *****************************

function [new_position, distance_traveled, back_on_line, same_pos, pos_cache] = ...
    follow_straight_wall(serPort, eps_y, curr_position, distance_traveled, start, real, pos_cache)

    % Re-intialize distance and angle to record
    [Distance] = DistanceSensorRoomba(serPort);
    [AngleR] = AngleSensorRoomba(serPort); 
    
    % Update the position
    new_position = curr_position; 
    y = new_position(2,4);
    x = new_position(1,4);
    
    % Intialize values, save starting position passed in
    back_on_line = 0;
    start_x = start(1,1);
    start_y = start(1,2);
    same_pos = 0;
    
    % Two parameter sets for simulator and roomba
    if real == 1
        pause_time = 0.4; 
        obstacle_goal = 4.0; 
        speed = .1; 
    else
        pause_time = 0.1; 
        obstacle_goal = 4.0; 
        speed = 0.05; 
    end
    
    % Until we are back on the line, or bump/lose walls, continue
    while 1
        
        % We are back on the line
        if (abs(y) < eps_y) && x > 0 && (distance_traveled > .5)
            SetFwdVelRadiusRoomba(serPort, 0.1, inf);
            pause(pause_time);
            back_on_line = 1;
            pause(0.1);
            l=0;
            f=0;
            r=0;
            break;
        end
        
        % Move forward
        SetFwdVelRadiusRoomba(serPort,speed , inf);      
        pause(pause_time);
        
        % Read sensors
        [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
            BumpsWheelDropsSensorsRoomba(serPort); 
        w = WallSensorReadRoomba(serPort);
        while isnan(w)
            w = WallSensorReadRoomba(serPort);
        end
        
        % If we bump or lose contact, break out of while loop
        if f==1 || l==1 || w == 0
            display('lost wall');
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            pause(pause_time); 
            [Distance] = DistanceSensorRoomba(serPort); 
            [AngleR]=AngleSensorRoomba(serPort); 
            distance_traveled = distance_traveled + abs(Distance)
            new_position = new_position * position(AngleR,Distance, 0);
            pos_cache = [pos_cache; new_position(2,4), new_position(1,4)];
            break;
        end
        
        % Update distance and angle values
        [Distance] = DistanceSensorRoomba(serPort); 
        [AngleR] = AngleSensorRoomba(serPort); 
        distance_traveled = distance_traveled + abs(Distance)
        new_position = new_position * position(AngleR, Distance, 0)
        pos_cache = [pos_cache; new_position(2,4), new_position(1,4)];
        
        % If we are in the same position as our initial bump
        if(abs(new_position(1,4) - start_x) < 0.3 && abs(new_position(2,4) - start_y) < 0.3 && distance_traveled > 0.5 )
            same_pos = 1;
            break;
        end
        
        y = new_position(2,4);
        x = new_position(1,4);
    end    
end

% ************************ Turn left until parallel *****************************

function [l, f, r, w, new_position, distance_traveled, pos_cache] = ...
    turn_left_until_parallel(serPort, curr_position, distance_traveled, real, pos_cache)
    if real == 1
        pause_time = 0.2; 
        speed = .05;
        turn_rad = eps;
    else
        pause_time = 0.01;  
        speed = 0.05;
        turn_rad = .1;
    end
    
    [AngleR]=AngleSensorRoomba(serPort);
    [Distance] = DistanceSensorRoomba(serPort);
    w = WallSensorReadRoomba(serPort);
    while isnan(w)
        w = WallSensorReadRoomba(serPort);
    end
    w = 0;
    new_position = curr_position;
    [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
        BumpsWheelDropsSensorsRoomba(serPort)

    while l == 1 || f == 1 || r == 1 || w == 0

        SetFwdVelRadiusRoomba(serPort, speed, turn_rad)%.1
        pause(pause_time);
        %SetFwdVelRadiusRoomba(serPort, 0, inf);
        
        [AngleR]=AngleSensorRoomba(serPort); 
        [Distance] = DistanceSensorRoomba(serPort);
        distance_traveled = distance_traveled + abs([Distance]);
        new_position = new_position * position(AngleR, Distance, 0)
        pos_cache = [pos_cache; new_position(2,4), new_position(1,4)];
        [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
            BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers
        w = WallSensorReadRoomba(serPort)
        while isnan(w)
            w = WallSensorReadRoomba(serPort)
        end
        if w == 1 && r==0 && f==0
            break;
        end
        c = new_position(1,4);
        d = new_position(2,4);
        %scatter(c, d)
    end

    SetFwdVelRadiusRoomba(serPort, 0, inf);
    pause(.1);
end

% ************************ Turn right until wall *****************************

function [l, f, r, w, new_position, distance_traveled, pos_cache] = ...
    turn_right_until_wall(serPort, curr_position, distance_traveled, real, pos_cache)

    [AngleR]=AngleSensorRoomba(serPort);
    [Distance] = DistanceSensorRoomba(serPort);
    new_position = curr_position;

    if real == 1
        pause_time = 0.2; 
        turn_rad = -.17;
        speed = .1; 
    else
        pause_time = 0.01; 
        turn_rad = -0.1;
        speed = 0.05; 
    end
    
    w = WallSensorReadRoomba(serPort);
    while isnan(w)
        w = WallSensorReadRoomba(serPort);
    end
    
    [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
        BumpsWheelDropsSensorsRoomba(serPort);
    
    SetFwdVelRadiusRoomba(serPort, speed, inf);
    pause(pause_time*1.5); 
    SetFwdVelRadiusRoomba(serPort, 0, inf);
    [AngleR] = AngleSensorRoomba(serPort); 
    [Distance] = DistanceSensorRoomba(serPort);
    new_position = new_position * position(AngleR, Distance, 0);
    pos_cache = [pos_cache; new_position(2,4), new_position(1,4)];
    while w == 0
        SetFwdVelRadiusRoomba(serPort, speed, turn_rad); 
        pause(pause_time);
        %SetFwdVelRadiusRoomba(serPort, 0, inf);

        [AngleR] = AngleSensorRoomba(serPort); 
        [Distance] = DistanceSensorRoomba(serPort);
        distance_traveled = distance_traveled + abs(Distance);
        new_position = new_position * position(AngleR, Distance, 0);
        pos_cache = [pos_cache; new_position(2,4), new_position(1,4)];
            %c = new_position(1,4);
            %d = new_position(2,4);
            %scatter(c, d)
        w = WallSensorReadRoomba(serPort);
        while isnan(w)
            w = WallSensorReadRoomba(serPort);
        end
        [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
            BumpsWheelDropsSensorsRoomba(serPort);
    end
    SetFwdVelRadiusRoomba(serPort, 0, inf);
end
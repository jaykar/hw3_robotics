function cover_grid(serPort)

    % FOR SIMULATOR, SET real = 0;
    % FOR PHYSICAL ROOMBA, SET real=1;
    real = 0;
    
    % Two parameter sets for simulator and roomba
    if real == 1
        pause_time = 0.4;  
        speed = .1; 
    else
        pause_time = 0.1;  
        speed = 0.1; 
    end

    % Start clock
    tic;
    since_update = toc;
    
    % Check for bump
    [r, l, dr, dl, dc, f] = BumpsWheelDropsSensorsRoomba(serPort); 
    if l == 1 || f == 1 || r == 1
        % move around obstacles
    end
    
    % Start loop to navigate, stop after not updating in 30 seconds
    while since_update < 15
        % Set random turn angle
        %rng(5) % allows for predictable random number
        rad_angle = rand + rand;
        pos = randi(1,2);
        if pos == 1
            rad_angle = rad_angle * -1;     
        end        
  
        % Move
        SetFwdVelRadiusRoomba(serPort, speed, rad_angle);
        pause(pause_time);
        
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
        
        
        since_update = toc;
        
        % We dont want it to only move in a circle, because then if it
        % doesn't hit an obstacle, it might only map out a circle. Instead,
        % if its been a certain amt of time, have it change tactics?
        
    end
    
    display('Bumped or its been 15 seconds')

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



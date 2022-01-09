%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% (C) 2022 FRC1736 Robot Casserole
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2D Launched Ball Trajectory calculator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;

graphics_toolkit qt

close all
clear h

graphics_toolkit qt

h.ax = axes ("position", [0.05 0.42 0.9 0.5]);


function update_plot (obj, init = false)
  
  h = guidata (obj);

  %Nominal Iteration Parameters 
  launch_x_ft_nom = 1.0 + get(h.launch_x_dist_slider, "value") * 30.0;  %Distance between launch point and goal.  
  launch_z_ft_nom = 0.5 + get(h.launch_height_slider, "value") * 5.0; %Launch point of the ball height off the ground in ft. Max is 0.69m
  launch_angle_deg_nom = 20 + get(h.launch_angle_slider, "value") * 70; %angle between floor and launch point
  launch_speed_mps_nom = 5.0 + get(h.launch_speed_slider, "value") * 20.0; % Launch speed in meters per seconds
  ball_collision_eff = sqrt(2.5)/sqrt(3.0); % From the game manual - a ball dropped from 3 ft bouncs back up to 2.5 ft = sqrt(2.5)/sqrt(3.0) = .91 efficency
  
  % Parameter Spread
  launch_speed_mps_spread = get(h.launch_speed_spread_slider, "value") * 5.0;
  if(launch_speed_mps_spread != 0)
    launch_speeds = linspace(-1.0, 1.0, 5) .* launch_speed_mps_spread .+ launch_speed_mps_nom;
  else
    launch_speeds = [launch_speed_mps_nom];
  endif
  
  launch_angle_spread = get(h.launch_angle_spread_slider, "value") * 20.0;
  if(launch_angle_spread != 0)
    launch_angles = linspace(-1.0, 1.0, 5) .* launch_angle_spread .+ launch_angle_deg_nom;
  else
    launch_angles = [launch_angle_deg_nom];
  endif
  
  launch_x_dist_spread = get(h.launch_x_dist_spread_slider, "value") * 4.0;
  if(launch_x_dist_spread != 0)
    launch_dists = linspace(-1.0, 1.0, 5) .* launch_x_dist_spread .+ launch_x_ft_nom;
  else
    launch_dists = [launch_x_ft_nom];
  endif
  
  launch_height_spread = get(h.launch_height_spread_slider, "value") * 0.5;
  if(launch_height_spread != 0)
    launch_heights = linspace(-1.0, 1.0, 5) .* launch_height_spread .+ launch_z_ft_nom;
  else
    launch_heights = [launch_z_ft_nom];
  endif  
  

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Update GUI Labels
  set (h.launch_speed_label, "string", sprintf ("Speed:  %.1f +/- %.1f  m/s", launch_speed_mps_nom, launch_speed_mps_spread));
  set (h.launch_angle_label, "string", sprintf ("Angle:  %.1f +/- %.1f  deg", launch_angle_deg_nom, launch_angle_spread));
  set (h.launch_x_dist_label, "string", sprintf("Dist:   %.1f +/- %.1f  ft", launch_x_ft_nom, launch_x_dist_spread));
  set (h.launch_height_label, "string", sprintf("Height: %.1f +/- %.1f  ft:", launch_z_ft_nom, launch_height_spread));

  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Simulation constants - should probably stay as is.
  Ts = 0.01; %10ms sample rate for solver (10ms intervals)
  goal_top_height_m = 2.64; %Height of the top rim of the high goal - 8ft 1 in
  goal_top_diameter_m = 1.3208; %Diameter of the top rim of the goal - 4 ft 4 in
  goal_bottom_height_m = goal_top_height_m - 0.5760; % Height of the bottom rim of the goal above ground - about 1.9 ft below top.
  goal_bottom_diameter_m = 0.67; %Diameter of the bottom rim of the goal about 2.2 ft or so from cad
  ball_diameter_m = 0.2286; % 9in ball 
  ball_rad_m = ball_diameter_m/2;
  g_mps = 9.81; %Gravitational constant
  density_air_kgpm3 = 1.225; %desnity of air per https://en.wikipedia.org/wiki/Density_of_air
  m_ball_kg = 0.270; %Total guess at the weight of the "fuel" in kg.
  cD = 0.507; %Drag coefficent for tennis ball per https://twu.tennis-warehouse.com/learning_center/aerodynamics2.php
  

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Derived constants and sim vectors
  frontal_area_m2 = 0.5 * (4*pi*(0.5*ball_diameter_m)^2); %Half the total surface area is the frontal area
  Vt_mps = sqrt((2*m_ball_kg*g_mps)/(cD*density_air_kgpm3*frontal_area_m2)); %Terminal velocity in meters per second


  % Goal and inner wall points and vectors
  top_goal_x_close = 0.0;
  top_goal_x_far = top_goal_x_close + goal_top_diameter_m;
  top_goal_z = goal_top_height_m;
  bottom_goal_x_close = top_goal_x_close + (goal_top_diameter_m - goal_bottom_diameter_m)/2;
  bottom_goal_x_far = bottom_goal_x_close + goal_bottom_diameter_m;
  bottom_goal_z = goal_bottom_height_m; 
  vert_top_far = [top_goal_x_far, top_goal_z, 0];
  vert_top_close = [top_goal_x_close, top_goal_z, 0];
  vert_bottom_far = [bottom_goal_x_far, bottom_goal_z, 0];
  vert_bottom_close = [bottom_goal_x_close, bottom_goal_z, 0];

  % Wall unit and normal ectors
  wall_uv_far = (vert_top_far - vert_bottom_far)/norm(vert_top_far - vert_bottom_far);
  wall_uv_close = (vert_top_close - vert_bottom_close)/norm(vert_top_close - vert_bottom_close);
  wall_nv_far = [wall_uv_far(2),-1.0 * wall_uv_far(1), 0]; 
  wall_nv_close = [wall_uv_close(2),-1.0 * wall_uv_close(1), 0]; 
    

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Init Plot
  cla(h.ax)
  hold on;

  %draw floor and goal
  rectangle(h.ax, 'Position', [-1.0*0.3048*launch_x_ft_nom,-0.05,0,0.05], 'FaceColor', [0,0,0]);
  rectangle(h.ax, 'Position', [top_goal_x_close,top_goal_z,goal_top_diameter_m,0.05], 'FaceColor', [1,0,0]);
  rectangle(h.ax, 'Position', [bottom_goal_x_close,bottom_goal_z,goal_bottom_diameter_m,-0.05], 'FaceColor', [1,0,0]);
  plot(h.ax, [top_goal_x_close, bottom_goal_x_close] , [top_goal_z,bottom_goal_z]);
  plot(h.ax, [top_goal_x_far, bottom_goal_x_far] , [top_goal_z,bottom_goal_z]);

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Solution Iteration
  
  totalRuns = length(launch_speeds) * length(launch_angles) * length(launch_dists) * length(launch_heights);
  successRuns = 0;
  curRun = 0;

  for launch_speed_mps = launch_speeds
    for launch_angle_deg = launch_angles
      for launch_x_ft = launch_dists
        for launch_z_ft = launch_heights
          curRun += 1;
          set (h.results_label, "string", sprintf ("Recalculating Scenario %.1f of %.1f", curRun,totalRuns));


          i = 1; %simulation step
          
          % Per-iteration constant recalcs
          launch_angle_rad = pi/180*launch_angle_deg;
          launch_x_m = 0.3048*launch_x_ft;
          launch_z_m = 0.3048*launch_z_ft;
          
          % Clear old data from arrays
          time = zeros(1, 10);
          pos_x = zeros(1, 10);
          pos_z = zeros(1, 10);
          vel_x = zeros(1, 10);
          vel_z = zeros(1, 10);

          %See http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node29.html for some 
          %references on trajectory calculation

          %Initial conditions
          time(i) = i*Ts;
          pos_x(i) = -1.0*launch_x_m;
          pos_z(i) = launch_z_m;
          vel_x(i) = launch_speed_mps*cos(launch_angle_rad);
          vel_z(i) = launch_speed_mps*sin(launch_angle_rad);

          failed = 0;

          %calculate trajectory until terminal case (ball hits floor or ball hits goal)
          while(1)

            % Evaluate bounce conditions
            if(and(pos_x(i) > top_goal_x_close, pos_x(i) < top_goal_x_far, pos_z(i) < top_goal_z + ball_rad_m))
              velVec = [vel_x(i), vel_z(i), 0];
              posVec = [pos_x(i), pos_z(i), 0];
              distFar = distPointToLine(posVec, vert_top_far, vert_bottom_far);
              if(distFar <= ball_rad_m)
                velAlongWall = wall_uv_far * dot(wall_uv_far, velVec);
                velPerpWall  = wall_nv_far * dot(wall_nv_far, velVec);
                velPerpWall *= -1 * ball_collision_eff;
                vel_x(i) = velAlongWall(1) + velPerpWall(1);
                vel_z(i) = velAlongWall(2) + velPerpWall(2);
              endif
              
              distClose = distPointToLine(posVec, vert_top_close, vert_bottom_close);
              if(distClose <= ball_rad_m)
                velAlongWall = wall_uv_close * dot(wall_uv_close, velVec);
                velPerpWall  = wall_nv_close * dot(wall_nv_close, velVec);
                velPerpWall *= -1 * ball_collision_eff;
                vel_x(i) = velAlongWall(1) + velPerpWall(1);
                vel_z(i) = velAlongWall(2) + velPerpWall(2);
              endif
            endif

            % Step simulation forward
            i = i + 1;
            time(i) = i*Ts;
            vel_x(i) = vel_x(i-1)/(1+Ts*g_mps/Vt_mps);
            vel_z(i) = (-Ts*g_mps + vel_z(i-1))/(1+Ts*g_mps/Vt_mps);
            pos_x(i) = pos_x(i-1) + vel_x(i)*Ts;
            pos_z(i) = pos_z(i-1) + vel_z(i)*Ts;
            
            % Check end conditions
            if(and(pos_x(i) > bottom_goal_x_close, pos_z(i) < bottom_goal_z))
              % Too low, hit below the upper HUB
              failed = 1;
              break; 
            elseif(and(pos_x(i) > top_goal_x_far, pos_z(i) < top_goal_z + ball_rad_m))
              % Too far, overshot
              failed = 1;
              break; 
            elseif(and(pos_z(i) < bottom_goal_z + ball_rad_m*0.5, vel_z(i) < 0))
              if(or(pos_x(i) < bottom_goal_x_close, pos_x(i) > bottom_goal_x_far))
                  % Fell outside target plane inside the upper HUB
                failed = 1;
              else
                  % In the required plane, we're good!
                failed = 0;
              endif
              break;              
            endif 

          endwhile
          


          %draw ball
          %rectangle('Position', [pos_x(i)-ball_rad_m,pos_z(i)-ball_rad_m,2*ball_rad_m,2*ball_rad_m], 'FaceColor', [0.5, 0.5, 0.5], 'Curvature', [1, 1]);

          if(failed == 1)
            color = 'red';
          else
            color = 'green';
            successRuns += 1;
          endif
          
          %Plot trajectory
          plot(h.ax, pos_x, pos_z, "color", color);
      
        endfor
      endfor
    endfor
  endfor  

  set (h.results_label, "string", sprintf ("Goals Made: %.1f%%", 100.0*successRuns/totalRuns));


  axis(h.ax, "equal");

endfunction

h.launch_speed_label = uicontrol ("style", "text",
                           "units", "normalized",
                           "string", "",
                           "horizontalalignment", "left",
                           "position", [0.00 0.05 0.3 0.07]);

h.launch_speed_slider = uicontrol ("style", "slider",
                            "units", "normalized",
                            "string", "slider",
                            "callback", @update_plot,
                            "value", 0.4,
                            "position", [0.33 0.05 0.3 0.04]);
                            
h.launch_speed_spread_slider = uicontrol ("style", "slider",
                            "units", "normalized",
                            "string", "slider",
                            "callback", @update_plot,
                            "value", 0.0,
                            "position", [0.66 0.05 0.3 0.04]);   
   
h.launch_x_dist_label = uicontrol ("style", "text",
                           "units", "normalized",
                           "string", "",
                           "horizontalalignment", "left",
                           "position", [0.00 0.15 0.3 0.07]);
   
h.launch_x_dist_slider = uicontrol ("style", "slider",
                            "units", "normalized",
                            "string", "slider",
                            "callback", @update_plot,
                            "value", 0.5,
                            "position", [0.33 0.15 0.3 0.04]);
                            
h.launch_x_dist_spread_slider = uicontrol ("style", "slider",
                            "units", "normalized",
                            "string", "slider",
                            "callback", @update_plot,
                            "value", 0.0,
                            "position", [0.66 0.15 0.3 0.04]); 
       
h.launch_angle_label = uicontrol ("style", "text",
                           "units", "normalized",
                           "string", "",
                           "horizontalalignment", "left",
                           "position", [0.00 0.25 0.3 0.07]);
       
h.launch_angle_slider = uicontrol ("style", "slider",
                            "units", "normalized",
                            "string", "slider",
                            "callback", @update_plot,
                            "value", 0.25,
                            "position", [0.33 0.25 0.3 0.04]);
                            
h.launch_angle_spread_slider = uicontrol ("style", "slider",
                            "units", "normalized",
                            "string", "slider",
                            "callback", @update_plot,
                            "value", 0.0,
                            "position", [0.66 0.25 0.3 0.04]);   
           
h.launch_height_label = uicontrol ("style", "text",
                           "units", "normalized",
                           "string", "",
                           "horizontalalignment", "left",
                           "position", [0.00 0.35 0.3 0.07]);
           
h.launch_height_slider = uicontrol ("style", "slider",
                            "units", "normalized",
                            "string", "slider",
                            "callback", @update_plot,
                            "value", 0.75,
                            "position", [0.33 0.35 0.3 0.04]);
                            
h.launch_height_spread_slider = uicontrol ("style", "slider",
                            "units", "normalized",
                            "string", "slider",
                            "callback", @update_plot,
                            "value", 0.0,
                            "position", [0.66 0.35 0.3 0.04]);  
                            
h.results_label = uicontrol ("style", "text",
                           "units", "normalized",
                           "string", "",
                           "horizontalalignment", "left",
                           "position", [0.04 0.90 0.3 0.06]);                            
  
set (gcf, "color", get(0, "defaultuicontrolbackgroundcolor"))
guidata (gcf, h)
update_plot (gcf, true);

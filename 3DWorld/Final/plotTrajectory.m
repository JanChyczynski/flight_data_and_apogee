function [] =  plotTrajectory(data)
    %%
    % PACKAGE_SETUP (package_setup.m)
    %
    % flypath3d package check & configuration file
    % Last updated: 2016-08-09
    %
    % SYNTAX:
    %
    %    package_setup
    % 
    % --- flypath.m ---
    filename = strcat(cd,'\flypath.m'); 
    if exist(filename,'file') ~= 2
       warning('file flypath.m not found!'); 
    end
    
    % --- new_object.m ---
    filename = strcat(cd,'\new_object.m'); 
    if exist(filename,'file') ~= 2
       warning('file new_object.m not found!');
    end
    
    %%
    data_short = data(:, 2:7); %data without time 
    
 
    %Min, max for borders plot
    
    xmin = min(data_short(:,1));
    xmax = max(data_short(:,1));

    if xmin <0
        xmin = xmin*1.1;
    else
        xmin = xmin*0.9;
    end
    if xmax >0
        xmax = xmax*1.1;
    else
        xmax = xmax*0.9;
    end
    
    ymin = min(data_short(:,2))*1.1;
    ymax = max(data_short(:,2))*1.1;
    if ymin <0
        ymin = ymin*1.1;
    else
        ymin = ymin*0.9;
    end
    if ymax >0
        ymax = ymax*1.1;
    else
        ymax = ymax*0.9;
    end
    
    zmin = min(data_short(:,3)*1.1);
    zmax = max(data_short(:,3))*1.1;
    if zmin <0
        zmin = zmin*1.1;
    else
        zmin = zmin*0.9;
    end
    if zmax >0
        zmax = zmax*1.1;
    else
        zmax = zmax*0.9;
    end

    max_scale = (max([xmax-xmin ymax-ymin zmax-zmin])) / 40;
    
    
    % create an object
    new_object('aircraft.mat',data_short,...
    'model','missile.mat','scale',max_scale,...
    'edge',[0 0 0],'face',[0 0 0],'alpha',1,...
    'path','on','pathcolor',[.89 .0 .27],'pathwidth',0.5);
     
    %Screensize
    get(0,'ScreenSize');
    
    % aircraft trajectory animation
    flypath('aircraft.mat',...
        'animate','on','step',1,...
        'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
        'font','Georgia','fontsize',10,...
        'view',[45 30],...
        'xlim',[xmin xmax],'ylim',[ymin ymax],'zlim',[zmin zmax],...
        'output','test.gif');
    
    pause(1);
    close;
    
    x_data = data_short(:,1);
    y_data = data_short(:,2);
    z_data = data_short(:,3);
    
    pitch_data = data_short(:, 4);
    yaw_data = data_short(:,5);
    roll_data = data_short(:,6);
    
    t = data(:,1); 
    
    %Plots afterwards
    figure('Name','Measured Data','NumberTitle','off', "MenuBar","none", "ToolBar","none");
    sgtitle("Great Title!");
    
    %3D Plot
    subplot(2,5,[1,2,6,7]);
    plot3(x_data, y_data, z_data);
    grid on;
    title("Trajectory of the object");
    xlabel("x [m]");
    ylabel("y [m]");
    zlabel("z [m]");
    
    %X
    subplot(2,5,3);
    plot(t, x_data);
    grid on;
    title("X-Position");
    xlabel("t [s]");
    ylabel("x [m]");
    
    %Y
    subplot(2,5,4);
    plot(t, y_data);
    grid on;
    title("Y-Position");
    xlabel("t [s]");
    ylabel("y [m]");
    
    %Z
    subplot(2,5,5);
    plot(t, z_data);
    grid on;
    title("Z-Position");
    xlabel("t [s]");
    ylabel("z [m]");
    
    %Pitch
    subplot(2,5,8);
    plot(t, pitch_data);
    grid on;
    title("Pitch angle");
    xlabel("t [s]");
    ylabel("pitch [rad]");
    
    %Yaw
    subplot(2,5,9);
    plot(t, yaw_data);
    grid on;
    title("Yaw angle");
    xlabel("t [s]");
    ylabel("yaw [rad]");
    
    %Roll
    subplot(2,5,10);
    plot(t, roll_data);
    grid on;
    title("Roll angle");
    xlabel("t [s]");
    ylabel("roll [rad]");
    
    %Fullscreen
    set(gcf,'WindowState','maximized');

end

%Author: Roark Menezes, Z5116870
%Program: Solution for AAS, T1.2020, Project2.part 2....
%Incorporated EKF program from DemoEKF_2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PROJECT COMPLETED IN MATLAB 2017B%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function LaserProcessor(file)

global ABCD;            % I use a global variable, to easily share it, in many functions. You may obtain similar functionality using other ways.

ABCD.flagPause=0;

% In case the caller does not specify the input argument, we propose a
% default one, assumed to be in the same folder where we run this example from.
if ~exist('file','var'), file ='Laser__2.mat'; end;
if ~exist('IMUFile','var'), IMUFile ='IMU_dataC.mat'; end
if ~exist('SpeedFile','var'), SpeedFile ='Speed_dataC.mat'; end;

% load data file.
load(file); 
% The variable that contains the data structure is named "dataL"
% (because it was created under that name and saved)  
load(IMUFile);
load(SpeedFile);
% now, after loading, the data is in a variable named "dataL";



% --------------------------------------
% Create graphical object for refreshing data during program execution.
% Local Plot graphic handles
figure(1) ; clf(); 

LocalPlot.handle1 = plot(0,0,'b.');      % to be used for showing the laser points

axis([-10,10,0,20]);                         % focuses the plot on this region (of interest, close to the robot)
xlabel('X (meters)');
ylabel('Y (meters)');
%legend(LocalPlot.handle1, 'Laser Points');

LocalPlot.handle2 = title('');           % create an empty title..
hold on;
LocalPlot.handle3 = plot(0, 0, '+r');
%legend(LocalPlot.handle3, 'High Intensity Points');
hold on;
LocalPlot.handle4 = scatter(0, 0, '*g');
%legend(LocalPlot.handle4, 'Detected OOIs');
zoom on ;  grid on;

%---------------------------------
disp('Showing laser scans, in Cartesian representation');
fprintf('\nThere are [ %d ] laser scans in this dataset (file [%s])\n',dataL.N,file);


uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});

% Global Plot graphic handles
figure(2); clf();

GlobalPlot.handle1 = title('');                     % Global plot title
hold on;
GlobalPlot.handle2 = scatter(0, 0, '*r');           % Global position
%legend(GlobalPlot.handle2, 'Current Position');

axis([-4,6.3,-1,8]);                                % focuses the plot on this region (of interest, close to the robot)
xlabel('X (meters)');
ylabel('Y (meters)');
hold on;
GlobalPlot.handle3 = plot(0, 0, '+g');              % Global initial OOIs
%legend(GlobalPlot.handle3, 'Landmarks (Initial OOIs)');
%hold on;
GlobalPlot.handle4 = plot(0, 0, '+b');              % Global currently detected OOIs
%legend(GlobalPlot.handle4, 'Detected OOIs');
hold on;
GlobalPlot.handle5 = quiver(0, 0, 0, 0);            % Direction of UGV
hold on;
GlobalPlot.handle6 = plot(0,0);                    %  For distance between Current and Initial OOIs
set(GlobalPlot.handle6, 'color', [0, 0, 0]);
zoom on ;  grid on;
hold off;

uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackB,1});
uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@MyCallBackB,2});
% Now, loop through the avaialable scans..

% % Plot for EKF
% EKFPlot.handle1 = title('EKF Path');
% hold on;
% EKFPlot.handle2 = plot(0, 0, '.b');
% axis([-1.5 3.5 -1 5]);
% xlabel('X (m)');
% ylabel('Y (m)');

N = dataL.N; 
skip=3;     % in this example I skip some of the laser scans.
i=1;
j=2;

% global noise and EKF constants
stdDevGyro = 1.5 * pi/180;
stdDevSpeed = 0.4;
sdev_rangeMeasurement = 0.2;
sdev_bearingMeasurement = 1.5;
Q = diag( [ (0.01)^2 ,(0.01)^2 , (1*pi/180)^2]) ;
% Initialise EKFData struct
InitSimulation(stdDevSpeed,stdDevGyro,sdev_rangeMeasurement,sdev_bearingMeasurement,Q);

% get the Dead Reckoning coords and gyroz bias
[coords, bias] = Attitude(IMU, Vel);
% convert from degrees back to radians
% We will use this bias when estimating using EKF, the bias will be removed
% from the noisy gyroz (remembering noise is already incorporated in the
% real data)
bias = bias * pi/180;
global labels;
global running;
global Xe;
running = 1;
while running,             
    
    if (ABCD.flagPause), pause(0.2) ; continue ; end;
    if i>N, break ;  end;
        
    % Native time expressed via uint32 numbers, where 1 unint means
    % [1/10,000]second (i.e. 0.1 millisecond)
    % (BTW: we do not use the time, in this task)
    t =  double(dataL.times(i)-dataL.times(1))/10000;
    % t: time expressed in seconds, relative to the time of the first scan.
    
    scan_i = dataL.Scans(:,i);
    % Remove bias from the gyroz value, which is stored in IMU.DATAf(6,
    % :)
    MyProcessingOfScan(scan_i,t,LocalPlot,i,j,coords, GlobalPlot, IMU.DATAf(6, 5*i) - bias, Vel.speeds(5*i));   % some function to use the data...

    pause(0.001) ;                   % wait for ~10ms (approx.)
    %if exist(labels, 'var') == 1
    try
        delete(labels);
    catch
        fprintf("Couldn't delete labels object");
    end
    %end
    i=i+skip;
    j = j + 1;
end

% Plot the EKF Path (using global variable Xe)
figure(7); clf;
plot( Xe(1, :), Xe(2, :));
xlabel('X (m)');
ylabel('Y (m)');
title('Estimated Path using EKF');
figure(8); clf;

fprintf('\nDONE!\n');
plot(0:1:length(Xe(3,:))-1, Xe(3,:)*180/pi);
xlabel('Sample Number');
ylabel('Heading (deg)');
title('Heading from EKF Estimate');

% Read Matlab Help for explanation of FOR loops, and function double( ) and pause()


return;
end
%-----------------------------------------
function MyProcessingOfScan(scan,t,mh,i,j,coords,gp,gz,vel)
    % Create global variables for Xe and P, since they are functions of
    % past inputs. 
    global Xe P;
    % I made this function, to receive the following parameters/variables:
    % 'scan' : scan measurements to be shown.
    % 't':  associated time.    
    % 'i' : scan number.
    % 'mh'  : struct contaning handles of necessary graphical objects.
    
    angles = [0:360]'*0.5* pi/180 ;         % Associated angle for each range of scan
    % same as in "dataL.angles".
    
    % scan data is provided as a array of class uint16, which encodes range
    % and intensity (that is the way the sensor provides the data, in that
    % mode of operation)
    
    MaskLow13Bits = uint16(2^13-1); % mask for extracting the range bits.
    % the lower 13 bits are for indicating the range data (as provided by this sensor)
    maskE000 = bitshift(uint16(7),13)  ;
    rangesA = bitand(scan,MaskLow13Bits) ;
    intensities = bitand(scan,maskE000); 
    % rangesA now contains the range data of the scan, expressed in CM, having uint16 format.
    
    % now I convert ranges to meters, and also to floating point format
    ranges    = 0.01*double(rangesA); 
    X = cos(angles).*ranges;
    Y = sin(angles).*ranges;  
    ii = find(intensities~=0);
    
    % and then refresh the data in the plots...
    set(mh.handle1,'xdata',X,'ydata',Y);

    % and some text...
    s= sprintf('Cartesian Plot (Local Frame)\n Laser scan # [%d] at time [%.3f] secs',i,t);
    set(mh.handle2,'string',s);
    
    set(mh.handle3, 'xdata', X(ii), 'ydata', Y(ii));
    OOIs = ExtractOOIs(ranges,intensities) ;
    PlotOOIs(OOIs, mh);
    rotation = PlotGlobalOOI(OOIs, coords, i, gp, t);
    if(i == 1)
        Xe = [0;0;pi/2];
        P = zeros(3, 3);
    end
    
    % The next Xe and P are a function of EKF, which takes in the previous Xe and P.
    [Xe(:, j), P] = EKF(P, Xe(:, j-1), gz, vel, rotation);
    % Use Matlab help for learning the functionality of  uint16(), bitand()
    % , set( ), sprintf()...
    return;
end
function InitSimulation(stdDevSpeed,stdDevGyro,sdev_rangeMeasurement,sdev_bearingMeasurement,Q)
    % Make a struct for the given noise sdev data.
    global EKFData;
    EKFData.sdev_bearingMeasurement = sdev_bearingMeasurement*pi/180;
    EKFData.sdev_rangeMeasurement=sdev_rangeMeasurement;
    EKFData.Pu = [stdDevSpeed^2, 0; 0, stdDevGyro^2];
    EKFData.Q = Q;
end
% ---------------------------------------
% Callback function. I defined it, and associated it to certain GUI button,
function MyCallBackA(~,~,x)   
    global ABCD;
    global running;
    if (x==1)
       ABCD.flagPause = ~ABCD.flagPause; %Switch ON->OFF->ON -> and so on.
       return;
    end;
    if (x==2)
        
        disp('you pressed "END NOW"');
        %uiwait(msgbox('Ooops, you still need to implement this command!','?','modal'));
        closereq();
        disp('Program Terminated');
        running = 0;
        return;
    end;
    return;    
end

function MyCallBackB(~,~,x)   
    global ABCD;
    global running;
    if (x==1)
       ABCD.flagPause = ~ABCD.flagPause; %Switch ON->OFF->ON -> and so on.
       return;
    end;
    if (x==2)
        
        disp('you pressed "END NOW"');
        %uiwait(msgbox('Ooops, you still need to implement this command!','?','modal'));
        closereq();
        disp('Program Terminated');
        running = 0;
        return;
    end;
    return;    
end
% PART A, B (Collaborated with Andrew Lui (Z5114845))
function r = ExtractOOIs(ranges,intensities)
    r.N = 0;
    r.Centers = [];
    r.Sizes   = [];
    maxIndex = 361;  
    minIndex = 0;
    HR = find(intensities > 0); % Indices of highly reflective points
    if (size(HR) > 0)
        groupedHR = [diff(HR')~=1,true]; % consecutive indices of HR grouped into 1 
        newRanges = HR(groupedHR);
        for i=1:size(newRanges) 
        currentIndex = newRanges(i); % iterate through indices of HR 
        
        % range diff to the left and right of HR point
        r_right = 0;
        r_left = 0;
        
        counter = 0;
        
        % threshold for range difference between points to be considered a
        % single object
        threshold = 0.2;
        
        %indices of left most and right most points of cluster
        end_right = 0;
        end_left = 0;
        
        % Check right
        if(currentIndex == maxIndex) 
            end_right = maxIndex;
        else
            while (r_right <= threshold && currentIndex + counter < maxIndex)
                end_right = currentIndex + counter;
                r_right = ranges(currentIndex+counter) - ranges(currentIndex);              
                counter = counter + 1;
            end
            % when the while loop fails, "right" will have the fail value
            % so go back a step
            end_right = end_right - 1;
        end
        
        % Check left
        counter = 0;
        if(currentIndex == 1) 
            end_left = 1;
        else
            while (r_left <= threshold && currentIndex-counter > minIndex)
                end_left = currentIndex-counter;
                r_left = ranges(currentIndex-counter) - ranges(currentIndex);
                counter = counter + 1;
            end
            % when the while loop fails, "left" will have the fail value
            % so go back a step (forward one index value)
            end_left = end_left + 1;
        end
        
        r1 = ranges(end_left);
        r2 = ranges(end_right);
        total(i) = sqrt(r1^2 + r2^2 - 2*r1*r2*cos(0.5*(end_right - end_left)*pi/180));
        
        lefts(i) = end_left;
        rights(i) = end_right;
        
        center(1,i) = ((r1 * cos(end_left*0.5*pi/180)) + (r2 * cos(end_right*0.5*pi/180))) / 2;
        center(2,i) = ((r1 * sin(end_left*0.5*pi/180)) + (r2 * sin(end_right*0.5*pi/180))) / 2;
        
        end
        r.N = size(newRanges,1);
        r.Sizes = total;
        r.Centers = center;
    end
    
return;
end
    
function PlotOOIs(OOIs, mh)
    if OOIs.N<1, return ; end;
    % your part....
    set(mh.handle4,'xdata',OOIs.Centers(1, :), 'ydata', OOIs.Centers(2, :));
    return;
end

% PART C (Collaborated with Andrew Lui (Z5114845))
% Tweaked from project 1 to output bias as well
function [coords, bias] = Attitude(IMU, Vel)
    NIMU = IMU.N; % number of scans IMU
    
    gyros = IMU.DATAf(4:6, :) * 180/pi;
    speeds = Vel.speeds;

    yaw(1) = 90;
    yawbias(1) = 90;
    wz = gyros(3, :);
    
    % remove bias using average displacement from 0 for first 20 seconds
    bias = sum(wz(1:20/0.005))/(20/0.005);
    wzbias = wz - bias;
    
    % time step
    dt = 0.005; % 5ms
    
    for i = 2:NIMU
        yaw(i) = yaw(i-1) + dt *wz(i-1);
    end
    
    for i = 2:NIMU
        yawbias(i) = yawbias(i-1) + dt *wzbias(i-1);
    end

    timeEnd = NIMU * dt;
    timeReal = 0:dt:timeEnd-dt;
    time = timeReal / 50;
    
    % Plot heading vs time
    figure(3)
    title('Yaw Rate Integrated');
    grid on;
    yr = plot(time, yaw);
    hold on;
    yb = plot(time, yawbias);
    xlabel('Time Sample');
    ylabel('Heading (Deg)');
    legend([yr; yb], 'Biased', 'Corrected');
    
    % Plot wz
    figure(4);
    plot(timeReal, wzbias);
    title('Yaw Rate');
    xlabel('Time (s)');
    ylabel('Wz (deg/s)');
    grid on;
    
    % Plot speed
    figure(5);
    plot(timeReal, speeds);
    title('Longitudinal Velocity');
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    grid on;
    
    % PART D
    % Dead Reckoning Process
    % Pose = (x, y, theta) where x and y are the 2D co-ordinates and theta
    % is the heading (yaw) calculated above.
    
    % We use discrete time version of the kinematic model based on angular
    % rate
    
    % starting pose is [0 0 90]
    x(1) = 0;
    y(1) = 0;
    dt = 0.005;
    
    for i = 2:NIMU
        x(i) = x(i-1) + dt*speeds(i-1)*cos(yawbias(i-1)*pi/180);
        y(i) = y(i-1) + dt*speeds(i-1)*sin(yawbias(i-1)*pi/180);
    end
    coords(1, :) = x;
    coords(2, :) = y;
    coords(3, :) = yawbias*(pi/180);
    figure(6)
    plot(x, y);
    title('Path of the UGV');
    xlabel('X (m)');
    ylabel('Y (m)');
end 

% PART E
function rotation = PlotGlobalOOI(OOIs, coords, i, gp, t)
    % Given the OOIs in local frame and the coords of the vehicle in
    % navigational frame, plot the OOIs in the global frame
    % coords = pose for all time
    % OOIs = struct where OOIs.Centers has coords of OOIs
    global initialCenters
    global labels;
    % Set the title of the figure
    s= sprintf('Navigational Frame Plot \n Laser scan # [%d] at time [%.3f] secs',i,t);
    set(gp.handle1, 'string', s);
    % A time constant = total running time / total times (222.89/8192)
    T = 222.89/8192;
    
    % approximate current time of experiment
    C_T = i * T;
    
    % round to nearest 0.005 (time step of the IMU scans) and then get the
    % time step number for IMU scans
    rounded_time = C_T - mod(C_T, 0.005);
    step = round(rounded_time/0.005);
    rotation = [];
    if(step > 44578)
        return;
    end
    % Apply rotation matrix by current yaw angle to transform to
    % navigational frame layout
    
    % Get the heading at this time
    coord_angle = coords(3, step);
    theta = coord_angle - pi/2;
    d = 0.46; 
    
    % Apply the rotation
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    position = [coords(1, step); coords(2, step)];
    
    % Plot the initially detected OOIs
    
    % If there are currently detected OOIs, plot them
    labels = [];
    
    if(isempty(OOIs.Centers()) == 0)
        rot_ooi = [OOIs.Centers(1, :); OOIs.Centers(2, :)];
        rotation = R * rot_ooi;
        rotation(1, :) = rotation(1, :) + d*cos(pi - coord_angle) + position(1);
        rotation(2, :) = rotation(2, :) + d*sin(pi - coord_angle) + position(2);
        
        % obtain the rotated x and y coords
        rotx = rotation(1, :);
        roty = rotation(2, :);
        
        % For the OOIs detected at the initial time
        if(i == 1)
            initialCenters = rotation;
        end
    
        initials = initialCenters;
        
        set(gp.handle3, 'xdata', initials(1, :), 'ydata', initials(2, :));

        % for the currently detected OOIs
        set(gp.handle4, 'xdata', rotx, 'ydata', roty);
        % PART F
        % We need to iterate through the current OOIs and obtain a distance
        % from each of these OOIs to the initially detected OOIs. Pairs whose
        % distance between each other is <= 40cm (given tolerance) can me
        % matched. 
        
        % If not scan 0
        counter = 1;
        if(i ~= 1)
            for counti = 1:length(initials)
                for countj = 1:length(rotx)
                    x1val = initials(1, counti);
                    y1val = initials(2, counti);
                    x2val = rotx(countj);
                    y2val = roty(countj);

                    % compute the distance between the initial OOI and current OOI
                    dist = sqrt((x2val - x1val)^2 + (y2val - y1val)^2);

                    % If the distance is within the tolerance, label it on the
                    % graph
                    if dist <= 0.4
                        % displace just to the right of current OOI
                        labels(counter) = text(rotx(countj) + 0.1, roty(countj), num2str(6 - counti));
                        counter = counter + 1;
                        
                        figure(2);
                        % draw lines between the current and initial OOI
                        set(gp.handle6, 'xdata', [x1val; x2val], 'ydata', [y1val;y2val]);
                    end
                end
            end
        end
    end
    % plot position
    set(gp.handle2, 'xdata', position(1), 'ydata', position(2));
    
    % show the direction of the UGV
    set(gp.handle5, 'xdata', position(1), 'ydata', position(2));
    set(gp.handle5, 'udata', 1*cos(coord_angle), 'vdata', 1*sin(coord_angle));
    
    
end
function [Xe, P] = EKF(P, Xe, gyroz, speed, OOIs)
    % Declare the global variables we need
    global EKFData
    global initialCenters
    % Declare the Jacobian and P matrices, and run the process model on Xe
    dt = 0.027*3;
    J = [ [1,0,-dt*speed*sin(Xe(3))  ]  ; [0,1,dt*speed*cos(Xe(3))] ;    [ 0,0,1 ] ] ;
    Fu = [dt*cos(Xe(3)), 0; dt*sin(Xe(3)), 0; 0, dt]; 
    P = J*P*J'+ Fu*(EKFData.Pu)*Fu' + EKFData.Q ;
    Xe    = RunProcessModel(Xe,speed,gyroz,dt) ;
%     min(nDetectedLandmarks, size(initialCenters, 2))
    % Obtain the measured bearings and ranges
    [nDetectedLandmarks,MeasuredRanges,IDs,MeasuredBearings]=GetMeasurementsFromNearbyLandmarks(Xe, OOIs);
    if nDetectedLandmarks>0
        for u=1:min(nDetectedLandmarks, size(initialCenters, 2))
            ID = IDs(u);
            % Obtain the H matrix
            eDX = (OOIs(1,ID)-Xe(1)) ;      % (xu-x)
            eDY = (OOIs(2,ID)-Xe(2)) ;      % (yu-y)
            eDD = sqrt( eDX*eDX + eDY*eDY ) ; %   so : sqrt( (xu-x)^2+(yu-y)^2 )
            eDD2 = eDD^2; % (xu - x)^2 + (yu - y)^2 (common denom for 2nd row of H matrix)

            H = [  -eDX/eDD , -eDY/eDD , 0 ; eDY/eDD2, -eDX/eDD2, -1] ;   % Jacobian of h(X); size 1x3
            eeDX = (initialCenters(1,ID)-Xe(1)) ;
            eeDY = (initialCenters(2,ID)-Xe(2)) ;
            eeDD = sqrt(eeDX*eeDX + eeDY*eeDY);
            % Obtain the expected range and bearing for the current
            % landmark (u)
            ExpectedRange = eDD ;   % just a coincidence: we already calculated them for the Jacobian, so I reuse it. 
            ExpectedBearing = atan2(eDY, eDX) - Xe(3) + pi/2;
            % Obtain the z matrix
            z(1, :) = MeasuredRanges(u) - ExpectedRange ;   
            z(2, :) = MeasuredBearings(u) - ExpectedBearing;
            z(2, :) = wrapToPi(z(2, :));
            % Obtain the R matrix
            R = [EKFData.sdev_rangeMeasurement^2, 0; 0, (EKFData.sdev_bearingMeasurement*pi/180)^2];
            
            % Obtain the EKF Xe and update the covariance matrix, P
            S = R + H*P*H' ;
            iS= inv(S);                % iS = inv(S) ;   % in this case S is 1x1 so inv(S) is just 1/S
            %iS = 1/S;
            K = P*(H'*iS) ;           % Kalman gain
            % ----- finally, we do it...We obtain  X(k+1|k+1) and P(k+1|k+1)

            Xe = Xe+K*z ;       % update the  expected value
            %P = P-K*H*P ;       % update the Covariance % i.e. "P = P-P*H'*iS*H*P"  )
            P = P - P*H'*iS*H*P;
        end
    end
end

% Process model function
function Xnext=RunProcessModel(X,speed,GyroZ,dt) 
    Xnext = X + dt*[ speed*cos(X(3)) ;  speed*sin(X(3)) ; GyroZ ] ;
end

% Function for obtain measured bearings and ranges
function [nDetectedLandmarks,ranges,IDs,bearings] = GetMeasurementsFromNearbyLandmarks(X,OOIs)
    if ~isempty(OOIs)
        dx= OOIs(1,:) - X(1) ;
        dy= OOIs(2,:) - X(2) ;
        ranges = sqrt((dx.*dx + dy.*dy)) ;
        tanval = atan2(dy, dx);
        bearings = tanval - X(3) + pi/2;
        IDs = [1:size(OOIs, 2)];
    else
        IDs=[];ranges=[];bearings=[];
    end
    nDetectedLandmarks = size(OOIs, 2);
    % I simulate I measure/detect all the landmarks, however there can be
    % cases where I see just the nearby ones.
    
end

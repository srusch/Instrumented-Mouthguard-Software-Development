% Read data from .csv file
RawMotionData_Mandible = readtable('mandible.csv');
RawMotionData_Maxilla = readtable('maxilla.csv');

% Extract measurement parameters from raw motion data
% Mandibe motion data
Al_Mandible = RawMotionData_Mandible{:, 2};
Ah_Mandible = RawMotionData_Mandible{:, 3};
Av_Mandible = RawMotionData_Mandible{:, 4};
Gl_Mandible = RawMotionData_Mandible{:, 5};
Gh_Mandible = RawMotionData_Mandible{:, 6};
Gv_Mandible = RawMotionData_Mandible{:, 7};
Roll_Mandible = RawMotionData_Mandible{:, 8};
Pitch_Mandible = RawMotionData_Mandible{:, 9};
Yaw_Mandible = RawMotionData_Mandible{:, 10};
% Maxilla motion data
Al_Maxilla = RawMotionData_Maxilla{:, 2};
Ah_Maxilla = RawMotionData_Maxilla{:, 3};
Av_Maxilla = RawMotionData_Maxilla{:, 4};
Gl_Maxilla = RawMotionData_Maxilla{:, 5};
Gh_Maxilla = RawMotionData_Maxilla{:, 6};
Gv_Maxilla = RawMotionData_Maxilla{:, 7};
Roll_Maxilla = RawMotionData_Maxilla{:, 8};
Pitch_Maxilla = RawMotionData_Maxilla{:, 9};
Yaw_Maxilla = RawMotionData_Maxilla{:, 10};

% Distance between TMJ and apex of the dental arch along the horizontal
% axis
C = 83; % mm
B = 74; % mm

% Determine amount of data points are present for each parameter
DataCount = size(Al_Mandible, 1);

% Initialise variables to store the calculated distance
MouthOpeningDistanceVH = zeros(DataCount, 1);
Dl = zeros(DataCount, 1);
Dh = zeros(DataCount, 1);
Dv = zeros(DataCount, 1);

% Initialise variables to store the orientation of the mandible relative to
% the maxilla
RelativeRoll = zeros(DataCount, 1);
RelativePitch = zeros(DataCount, 1);
RelativeYaw = zeros(DataCount, 1);

% Initialise variables to store the acceleration of the mandible relative to
% the maxilla
RelativeAccelerationL = zeros(DataCount, 1);
RelativeAccelerationH = zeros(DataCount, 1);
RelativeAccelerationV = zeros(DataCount, 1);

% Initialise variables to store the angular velocity of the mandible relative
% to the maxilla
RelativeAngularVelocityL = zeros(DataCount, 1);
RelativeAngularVelocityH = zeros(DataCount, 1);
RelativeAngularVelocityV = zeros(DataCount, 1);

% Initialise matrix to store calculated data to be written to the .csv
% output file
OutputData = zeros(DataCount, 13);

% Initialise time variable
Time = 0;
A_MandiblePlanePrev = 0;
V_MandiblePlanePrev = 0;
X_MandiblePlanePrev = 0;


% Loop through data at each timestep
for Counter = 1:DataCount

    % Calculate the orientation of the mandible relative to the maxilla
    RelativeRoll(Counter) = Roll_Mandible(Counter) - Roll_Maxilla(Counter);
    RelativePitch(Counter) = Pitch_Mandible(Counter) - Pitch_Maxilla(Counter);
    RelativeYaw(Counter) = Yaw_Mandible(Counter) - Yaw_Maxilla(Counter);

    % Calculate the acceleration of the mandible relative to the maxilla
    RelativeAccelerationL(Counter) = Al_Mandible(Counter) - Al_Maxilla(Counter);
    RelativeAccelerationH(Counter) = Ah_Mandible(Counter) - Ah_Maxilla(Counter);
    RelativeAccelerationV(Counter) = Av_Mandible(Counter) - Av_Maxilla(Counter);

    % Calculate the angular velocity of the mandible relative to the maxilla
    RelativeAngularVelocityL(Counter) = Gl_Mandible(Counter) - Gl_Maxilla(Counter);
    RelativeAngularVelocityH(Counter) = Gh_Mandible(Counter) - Gh_Maxilla(Counter);
    RelativeAngularVelocityV(Counter) = Gv_Mandible(Counter) - Gv_Maxilla(Counter);

    % Calculate the distance along the mandibular plane from acceleration
    A_MandiblePlane = (RelativeAccelerationH(Counter)*cosd(Pitch_Mandible(Counter))) - (RelativeAccelerationV(Counter)*cosd(Pitch_Mandible(Counter)));
        % Intergate to get velocity
        V = V_MandiblePlanePrev + ( (A_MandiblePlane + A_MandiblePlanePrev) / 2000 )*0.1;
        V_MandiblePlanePrev = V;
        % Integrate to get distance
        X = X_MandiblePlanePrev + ( (V + V_MandiblePlanePrev) / 2 )*0.05;
        X_MandiblePlanePrev = X;

    % Calculate the mouth opening diatnace (mm)
    MouthOpeningDistanceVH(Counter) = sqrt( (2*C*(B + X)) - 2*C*(B + X)*cosd(RelativePitch(Counter)) );

    % Calculate auxilary angles required to determine distances
    Angle1 = acosd( MouthOpeningDistanceVH(Counter) /(2*C*(B + X)) );  % degrees
    Angle2 = 180 - RelativePitch(Counter) - Angle1;                   % degrees
    Angle3 = 90 - Angle2;                                             % degrees

    % Calculate distance travelled along the vertical and horizontal axes
    Dv(Counter) = MouthOpeningDistanceVH(Counter) * cosd(Angle3);  % mm
    Dh(Counter) = sqrt(MouthOpeningDistanceVH(Counter)^2 - Dv(Counter)^2 );  % mm

    % Calculate the distance travelled along the lateral axis
    Dl(Counter) = Dv(Counter) * sind( RelativeYaw(Counter) );    %mm

    % Write the data to the table to be later saved into the output .csv
    OutputData(Counter,:) = [Time, ...
        RelativeAccelerationL(Counter), RelativeAccelerationH(Counter), RelativeAccelerationV(Counter), ...
        RelativeAngularVelocityL(Counter), RelativeAngularVelocityH(Counter), RelativeAngularVelocityV(Counter), ...
        RelativeRoll(Counter), RelativePitch(Counter), RelativeYaw(Counter), ...
        Dl(Counter), Dh(Counter), Dv(Counter)];

    % Increment time by 100ms (10 Hz sending rate)
    Time = Time + 0.1;

end

%% Output .csv file
% Write all data to output CSV file
CSV = array2table(OutputData,"VariableNames", { 'Time Stamp', ...
    'Lateral Acceleration', 'Horizontal Acceleration', ...
    'Vertical Acceleration', 'Angular Velocity (Lateral Axis)', ...
    'Angular Velocity (Horizontal Axis)', ...
    'Angular Velocity (Vertical Axis)', 'Roll', 'Pitch', 'Yaw', ...
     'Lateral Distance', 'Horizontal Distance', ...
    'Vertical Distance'});

% Write the data to the .csv output file
writetable(CSV, 'MandibularMotion.csv')

% Plots of calculated data
TimeSteps = 0:0.1:(0.1*(DataCount-1));

%% Linear acceleration
figure;

% Lateral Acceleration
subplot(3,1,1);
plot(TimeSteps, RelativeAccelerationL)
hold on
xlabel('Time [s]')
ylabel('Acceleration [mm/s^2]')
title('Lateral Axis')
grid off

% Horizontal Acceleration
subplot(3,1,2);
plot(TimeSteps, RelativeAccelerationH)
hold on
grid on
xlabel('Time [s]')
ylabel('Acceleration [mm/s^2]')
title('Horizontal Axis')
grid off

% Vertical Acceleration
subplot(3,1,3);
plot(TimeSteps, RelativeAccelerationV)
hold on
xlabel('Time [s]')
ylabel('Acceleration [mm/s^2]')
title('Vertical Axis')
grid off

sgtitle("Linear Acceleration of the Mandible Relative to the Maxilla")


%% Angular Velocity
figure;

% Angular Velocity about the lateral axis
subplot(3,1,1);
plot(TimeSteps, RelativeAngularVelocityL)
hold on
xlabel('Time [s]')
ylabel('Angular Velocity [°/s]')
title('Lateral Axis')
hold off

% Angular Velocity about the horizontal axis
subplot(3,1,2);
plot(TimeSteps, RelativeAngularVelocityH)
hold on
xlabel('Time [s]')
ylabel('Angular Velocity [°/s]')
title('Horizontal Axis')
hold off

% Angular Velocity about the vertical axis
subplot(3,1,3);
plot(TimeSteps, RelativeAngularVelocityV)
hold on
grid on
xlabel('Time [s]')
ylabel('Angular Velocity [°/s]')
title('Vertical Axis')
grid off

sgtitle("Angular Velocity of the Mandible Relative to the Maxilla")

%% Linear displacement
figure;

% Linear displacement along the lateral axis
subplot(3,1,1);
plot(TimeSteps, Dl(:,1))
hold on
xlabel('Time [s]')
ylabel('Displacement [mm]')
title('Lateral Axis')
hold off

% Linear displacement along the horizontal axis
subplot(3,1,2);
plot(TimeSteps, Dh)
hold on
xlabel('Time [s]')
ylabel('Displacement [mm]')
title('Horizontal Axis')
hold off

% Linear displacement along the vertical axis
subplot(3,1,3);
plot(TimeSteps, Dv)
hold on
xlabel('Time [s]')
ylabel('Displacement [mm]')
title('Vertical Axis')
hold off

sgtitle("Linear Displacement of the Mandible Relative to the Maxilla")

%% Angular displacement
figure;

% Angular displacement about the lateral axis
subplot(3,1,1);
plot(TimeSteps, RelativePitch)
hold on
xlabel('Time [s]')
ylabel('Displacement [°]')
title('Lateral Axis')
hold off

% Angular displacement about the horizontal axis
subplot(3,1,2);
plot(TimeSteps, RelativeRoll)
hold on
xlabel('Time [s]')
ylabel('Displacement [°]')
title('Horizontal Axis')
hold off

% Angular displacement about the vertical axis
subplot(3,1,3);
plot(TimeSteps, RelativeYaw)
hold on
xlabel('Time [s]')
ylabel('Displacement [°]')
title('Vertical Axis')
hold off

sgtitle("Angular Displacement of the Mandible Relative to the Maxilla")

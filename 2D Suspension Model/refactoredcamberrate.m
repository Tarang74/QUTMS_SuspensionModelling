clc; clear all; close all;

%% Input Variables
% A-arms pickup points
pickups = [245 307.03; %Front Upper Front Arm
    245 393.73;    %Front Upper Rear Arm
    105.94 145.76; %Front Lower Front Arm
    105.94 145.76; %Front Lower Rear Arm
    308.17 276.99; %Rear Upper Front Arm
    289.17 283.43; %Rear Upper Rear Arm
    287.61 120.78; %Rear Lower Front Arm
    268.8 122.03; %Rear Lower Rear Arm
    ];

 %Initial Spherical Holder Position
 shcoords = [502.56 298.98; %Front Upper A-arm
        531.71 104.46; %Front Lower A-arm
        490.41 298.77; % Rear Upper A-arm
        531.71 104.46]; %Rear Lower A-arm
KPI = [7.5; 11]; %Front; Rear;
tyrerad = 203.2; %mm
%% 4Bar Equations
pickups2D = [((pickups(1,1)+ pickups(2,1))/2) ((pickups(1,2)+ pickups(2,2))/2); %Front Upper
    ((pickups(3,1)+ pickups(4,1))/2) ((pickups(3,2)+ pickups(4,2))/2); %Front Lower
    ((pickups(5,1)+ pickups(6,1))/2) ((pickups(5,2)+ pickups(6,2))/2); %Rear Upper
    ((pickups(7,1)+ pickups(8,1))/2) ((pickups(7,2)+ pickups(8,2))/2); %Rear Lower
    ];

%Upper A-arm Link (a)
UWL = sqrt((pickups2D(1,1)-shcoords(1,1))^2+(pickups2D(1,2)-shcoords(1,2))^2);
%Lower A-arm Link (c)
BWL = sqrt((pickups2D(2,1)-shcoords(2,1))^2+(pickups2D(2,2)-shcoords(2,2))^2);
%Upper Spherical to Lower Spherical (b)
coupler = sqrt((shcoords(1,1)-shcoords(2,1))^2+(shcoords(1,2)-shcoords(2,2))^2);
%Upper Chassis Point to Lower Chassis Point (d)
chassisgroundlink = sqrt((pickups2D(1,1)-pickups2D(2,1))^2+(pickups2D(1,2)-pickups2D(2,2))^2);

%Theta0 at rest
th0rest = atan((pickups2D(1,1)-pickups2D(2,1))/(pickups2D(1,2)-pickups2D(2,2)));

%Theta2 at rest
th2rest=atan((shcoords(1,2)-pickups2D(1,2))/abs(shcoords(1,1)-pickups2D(1,1)))+(pi/2)+abs(atan((pickups2D(1,1)-pickups2D(2,1))/(pickups2D(1,2)-pickups2D(2,2))));

%Initial constants
K4=chassisgroundlink/UWL;
K4=chassisgroundlink/BWL;
K4=(UWL^2+BWL^2+chassisgroundlink^2-coupler^2)/(2*UWL*BWL);
A=cos(th2rest)-K4-K4*cos(th2rest)+K4;
B=-2*sin(th2rest);
C=K4-(K4+1)*cos(th2rest)+K4;

%Theta 4 at rest
th4rest=(2*atan((-B-sqrt(B^2-(4*A*C)))/(2*A)));

%Theta 3 at rest
th3rest= asin((-UWL*sin(th2rest)+BWL*sin(th4rest))/coupler);

%% Initialisation of vertical displacement calcs
%To determine the camber angle during the fluctuation of the wheel, we should determine the variation of the coupler angle th3,as a function of vertical motion z of the coupler point C
f0 = sqrt(UWL^2+chassisgroundlink^2-2*UWL*chassisgroundlink*cos(th2rest));
q0 = atan((UWL*sin(th2rest))/(chassisgroundlink-UWL*cos(th2rest)));
p0 = atan(sqrt(4*BWL^2*f0^2-(BWL^2+f0^2-coupler^2))/(BWL^2+f0^2-coupler^2));

%initial coordinates of the coupler point C and the initial value of z
xc0 = UWL*cos(th2rest)+exp(1)*cos(p0+q0-alpha);
yc0 = UWL*sin(th2rest)+exp(1)*sin(p0+q0-alpha);
z0 = -xc0*cos(th0rest)-yc0*sin(th0);

%% Calculating Ride Camber
%Additional constants
K4 = d / b;
K5 = (c^2 - a^2 - b^2 - d^2)/(2*a*c);
rideint = 0.1;
riderange= 25;
for i=1:riderange/rideint
    f = sqrt(UWL^2+chassisgroundlink^2-2*UWL*chassisgroundlink*cos(th2));
    q = atan((UWL*sin(th2))/(chassisgroundlink-UWL*cos(th2)));
    p = atan(sqrt(4*BWL^2*f^2-(BWL^2+f^2-coupler^2))/(BWL^2+f^2-coupler^2));
    xc = UWL*cos(th2)+exp(1)*cos(p+q-alpha);
    yc = UWL*sin(th2)+exp(1)*sin(p+q-alpha);
    %Displacement z in terms of xc and yc
    z = -xc*cos(th0rest)-yc*sin(th0rest); %th0 won't be static for body roll
    %Vertical displacement of wheel centre
    h = z-z0;

    D = K5 - K1 + (1-K4)*cos(th2);
    E = -2*sin(th2);
    F = K5 + K1 - (1-K4)*cos(th2);
    th3 = 2 * atan((-E-sqrt(E^2-4*D*E))/(2*D));
    %initial angler of coupler link with vertical direction is th0-th3rest so
    gamma = th3rest - th3;
end
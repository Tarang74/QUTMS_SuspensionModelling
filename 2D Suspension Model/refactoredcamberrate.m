clc; clear all; close all;

%% Input Variables
% A-arms pickup points
pickups = [245 307.03; %Front Upper Front Arm
    245 393.73;    %Front Upper Rear Arm
    105.94 145.76; %Front Lower Front Arm
    105.94 145.76; %Front Lower Rear Arm
    ];

 %Initial Spherical Holder Position
 shcoords = [502.56 298.98; %Front Upper A-arm
        531.71 104.46; %Front Lower A-arm
        ];
KPI = deg2rad(7.5);
tyrerad = 203.2; %mm
alpha = deg2rad(36.44);
e = 137.51;
% alpha = deg2rad(33);
% e = 198;
%% 4Bar Equations
pickups2D = [((pickups(1,1)+ pickups(2,1))/2) ((pickups(1,2)+ pickups(2,2))/2); %Front Upper
    ((pickups(3,1)+ pickups(4,1))/2) ((pickups(3,2)+ pickups(4,2))/2); %Front Lower
    ];

%Upper A-arm Link (a)
a = sqrt((pickups2D(1,1)-shcoords(1,1))^2+(pickups2D(1,2)-shcoords(1,2))^2);
% a = 198;
%Lower A-arm Link (c)
c = sqrt((pickups2D(2,1)-shcoords(2,1))^2+(pickups2D(2,2)-shcoords(2,2))^2);
% c = 315;
%Upper Spherical to Lower Spherical (b)
b = sqrt((shcoords(1,1)-shcoords(2,1))^2+(shcoords(1,2)-shcoords(2,2))^2);
% b = 269;
%Upper Chassis Point to Lower Chassis Point (d)
d = sqrt((pickups2D(1,1)-pickups2D(2,1))^2+(pickups2D(1,2)-pickups2D(2,2))^2);
% d = 188;

%Theta0 at rest
th0rest = atan((pickups2D(1,1)-pickups2D(2,1))/(pickups2D(1,2)-pickups2D(2,2)));
% th0rest = deg2rad(9.625);

%Theta2 at rest
th2rest=atan((shcoords(1,2)-pickups2D(1,2))/abs(shcoords(1,1)-pickups2D(1,1)))+(pi/2)+abs(atan((pickups2D(1,1)-pickups2D(2,1))/(pickups2D(1,2)-pickups2D(2,2))));
% th2rest = deg2rad(99.625);

%Initial constants
K1=d/a;
K2=d/c;
K3=(a^2+c^2+d^2-b^2)/(2*a*c);
K4 = d / c;
K5 = (b^2 - a^2 - c^2 - d^2)/(2*a*b);
A=cos(th2rest)-K1-K2*cos(th2rest)+K3;
B=-2*sin(th2rest);
C=K1-(K2+1)*cos(th2rest)+K3;

%Theta 4 at rest
th4rest=(2*atan((-B-sqrt(B^2-(4*A*C)))/(2*A)));

%Theta 3 at rest
th3rest= asin((-a*sin(th2rest)+c*sin(th4rest))/b);

%% Initialisation of vertical displacement calcs
%To determine the camber angle during the fluctuation of the wheel, we should determine the variation of the coupler angle th3,as a function of vertical motion z of the coupler point C
f0 = sqrt(a^2+d^2-(2*a*d*cos(th2rest)));
q0 = atan((a*sin(th2rest))/(d-(a*cos(th2rest))));
p0 = atan(sqrt((4*b^2*f0^2)-(b^2+f0^2-c^2))/(b^2+f0^2-c^2));

%initial coordinates of the coupler point C and the initial value of z
xc0 = a*cos(th2rest)+e*cos(p0+q0-alpha);
yc0 = a*sin(th2rest)+e*sin(p0+q0-alpha);
z0 = -xc0*cos(th0rest)-yc0*sin(th0rest);

%% Calculating Ride Camber
gamma = [];
h = [];
z=[];
theta_inc=deg2rad(.25);
theta_range=deg2rad(10);
% theta_range=deg2rad(-5):th2rest:deg2rad(5);
th2arr=[];
th3=[];
th4=[];
% xc = [];
% yc = [];
camber=[];
th2arr(1)=th2rest-deg2rad(5);
% th2arr(1)=th2rest;
for i=1:theta_range/theta_inc
%     D = K5 - K1 + (1-K4)*cos(th2arr(i));
%     E = -2*sin(th2arr(i));
%     F = K5 + K1 - (1-K4)*cos(th2arr(i));
    A=cos(th2arr(i))-K1-K2*cos(th2arr(i))+K3;
    B=-2*sin(th2arr(i));
    C=K1-(K2+1)*cos(th2arr(i))+K3;
    th4(i) = 2 * atan((-B-sqrt(B^2-4*A*C))/(2*A));
    th3(i)= asin((-a*sin(th2arr(i))+c*sin(th4(i)))/b);
    %
    f = sqrt(a^2+d^2-(2*a*d*cos(th2arr(i))));
    q = atan((a*sin(th2arr(i)))/(d-a*cos(th2arr(i))));
    p = atan(sqrt(4*b^2*f^2-(b^2+f^2-c^2))/(b^2+f^2-c^2));
    xc = (a*cos(th2arr(i)))+(e*cos(p+q-alpha));
    yc = (a*sin(th2arr(i)))+(e*sin(p+q-alpha));
    
    %Displacement z in terms of xc and yc
    z = -xc*cos(th0rest)-yc*sin(th0rest); %th0 won't be static for body roll
    %Vertical displacement of wheel centre
    h(i) = z-z0;
    %initial angler of coupler link with vertical direction is th0-th3rest so
    gamma(i) = th3(i)-th0rest-KPI;
    camber(i) = rad2deg(gamma(i));
    th2arr(i+1)=th2arr(i)+theta_inc;
end

plot(camber, h, '-')
xlabel('camber (degrees)')
ylabel('Wheel Travel (mm)')
title('Camber vs Wheel Travel')
% plot(z)
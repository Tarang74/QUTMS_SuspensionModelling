%Prep
clc;clear all; close all;

%% Variables
    %%%A-arm coords (x,y) relative to chassis origin (Fixed points to
    %%%ground) looking from front
    FUFWX = 245; FUFWY = 307.03; %Upper Front Arm
    FURWX = 245; FURWY = 393.73; %Upper Rear Arm
    FBFWX = 105.94; FBFWY = 145.76; %Lower Front Arm
    FBRWX = 105.94; FBRWY = 145.76; %Lower Rear Arm
    RUFWX = 308.17; RUFWY = 276.99; %Upper Front Arm
    RURWX = 289.17; RURWY = 283.43; %Upper Rear Arm
    RBFWX = 287.61; RBFWY = 120.78; %Lower Front Arm
    RBRWX = 268.8; RBRWY = 122.03; %Lower Rear Arm
    
    %Converting to 2D (combining )
    FUWX=(FUFWX+FURWX)/2;
    FUWY=(FUFWY+FURWY)/2;
    FBWX=(FBFWX+FBRWX)/2;
    FBWY=(FBFWY+FBRWY)/2;
    RUWX=(RUFWX+RURWX)/2;
    RUWY=(RUFWY+RURWY)/2;
    RBWX=(RBFWX+RBRWX)/2;
    RBWY=(RBFWY+RBRWY)/2;

    %%%Initial Spherical Holder Position
    SHCoords = [502.56 298.98; %Front Upper A-arm
        531.71 104.46; %Front Lower A-arm
        490.41 298.77; % Rear Upper A-arm
        531.71 104.46]; %Rear Lower A-arm

    %%%King Pin Inclination Rear & Front
    fKPI = 7.5;
    rKPI = 11;
    %%%Tyre Diametre (Rideheight)
    
    Tyre = 203.2;
    %Distance of Tyre from spherical holder (mm)
    Tyredist = 260; %Distance of chassis body to wheel

%% LengthsofLinks
%%%Front Suspension
%%%Upper Front A-arm Link (c)
FUWL = sqrt((SHCoords(1, 1)-FUWX)^2+(SHCoords(1,2)-FUWY)^2);
%%%Lower Front A-arm Link (a)
FBWL = sqrt((SHCoords(2,1)-FBWX)^2+(SHCoords(2,2)-FBWY)^2);
%%%Upper Spherical to Lower Spherical (b)
FUSBSL = sqrt((SHCoords(1,1)-SHCoords(2,1))^2+(SHCoords(1,2)-SHCoords(2,2))^2);
%%%Upper Chassis Point to Lower Chassis Point (d)
FUCBCL = sqrt((FUWX-FBWX)^2+(((FUWY-FBWY)^2)));

%%%Calc Chassis angle
Chassang = (abs(atan((FUWY-FBWY)/(FUWX-FBWX))));

%% 4BarEquations
%%%Relation to wheel travel angle of upper a-arm from +ve x axis
%theta2og wrong
theta2og=atan((SHCoords(1,2)-FUWY)/abs(SHCoords(1,1)-FUWX))+(pi/2)+abs(atan((FUWX-FBWX)/(FUWY-FBWY)));
theta_inc=0.25*(pi/180);
theta_range=5*(pi/180);
theta2pp=[];
theta2pp(1)=theta2og;
camber=[];
wtrav=[];
%%%Calculating K constants
K1=FUCBCL/FUWL;
K2=FUCBCL/FBWL;
K3=(FUWL^2+FBWL^2+FUCBCL^2-FUSBSL^2)/(2*FUWL*FBWL);

TYog=FIBSHY-(((Tyre*2)-(FIUSHY-FIBSHY))/2); %
TXog=FIBSHX+(Tyredist*sign(FIBSHX)); %

%UP
for i=1:theta_range/theta_inc
    A=cos(theta2pp(i))-K1-K2*cos(theta2pp(i))+K3;
    B=-2*sin(theta2pp(i));
    C=K1-(K2+1)*cos(theta2pp(i))+K3;
    theta4=(2*atan((-B-sqrt(B^2-(4*A*C)))/(2*A))); %
    theta3= asin((-FUWL*sin(theta2pp(i))+FBWL*sin(theta4))/FUSBSL); %
    
    %Calculating position of upper and lower spherical holders
    %angle from +ve y axis to upper wishbone (underneath)
    %Sign for if the suspension is other side creating -ve x values in
    %which case it should be UWX-UWL*cos(UpperAngle);
    thetaU= -pi + theta2pp(i)+ Chassang;
    UPSHX = FUWX + FUWL*sign(FIUSHX)*cos(thetaU);
    UPSHY = FUWY + FUWL*sin(thetaU);
    thetaL = -pi + theta4 + Chassang;
    BSHX = FBWX + FBWL*sign(FIBSHX)*cos(thetaL);
    BSHY = FBWY + FBWL*sin(thetaL);
    
    TY= BSHY-(((Tyre*2)-(FIUSHY-FIBSHY))/2);
    
    if (i == 1)
        wtrav(i) = 0;
    else 
        wtrav(i) = TY-TYog;
    end
    
    %Calc camber: Angle of USBL from x axis - KPI (Note: -90 doesnt work fix)
    camber(i) = -(((asin(((BSHX*sign(BSHX))-(UPSHX*sign(UPSHX)))/FUSBSL)))-(KPI*(pi/180)))*(180/pi);
    
    if (i < theta_range/theta_inc)
        theta2pp(i+1)=theta2pp(i)+theta_inc;
    end
end

%Down
theta2pp(i+1)=theta2og-theta_inc;
for j=1:theta_range/theta_inc
    %4 bar linkage constants
    A=cos(theta2pp(j+i))-K1-K2*cos(theta2pp(j+i))+K3;
    B=-2*sin(theta2pp(j+i));
    C=K1-(K2+1)*cos(theta2pp(j+i))+K3;
    theta4=(2*atan((-B-sqrt(B^2-(4*A*C)))/(2*A)));
    theta3= asin((-FUWL*sin(theta2pp(j+i))+FBWL*sin(theta4))/FUSBSL);
    
    %Calculating position of upper and lower spherical holders
    %angle from +ve y axis to upper wishbone (underneath)
    %Sign for if the suspension is other side creating -ve x values in
    %which case it should be UWX-UWL*cos(UpperAngle);
    thetaU= -pi + theta2pp(j+i)+ Chassang;
    UPSHX = FUWX + FUWL*sign(FIUSHX)*cos(thetaU);
    UPSHY = FUWY + FUWL*sin(thetaU);
    thetaL = -pi + theta4 + Chassang;
    BSHX = FBWX + FBWL*sign(FIBSHX)*cos(thetaL);
    BSHY = FBWY + FBWL*sin(thetaL);
    
    TY= BSHY-(((Tyre*2)-(FIUSHY-FIBSHY))/2);
    
    %Error in graph is wtrav(21)
    wtrav(j+i) = TY-TYog;
    
    %Calc camber: Angle of USBL from x axis - KPI (Note: -90 doesnt work fix)
    camber(j+i) = -(((asin(((BSHX*sign(BSHX))-(UPSHX*sign(UPSHX)))/FUSBSL)))-(KPI*(pi/180)))*(180/pi);
    
    if (j < theta_range/theta_inc)
        theta2pp(j+i+1)=theta2pp(j+i)-theta_inc;
    end
end


[~, sort_idx] = sort(wtrav);

plot(camber(sort_idx), wtrav(sort_idx), '-*')
xlabel('camber (degrees)')
ylabel('Wheel Travel (mm)')
title('Camber vs Wheel Travel')
%Prep
clc;clear all; close all;

%%%---------Variables---------%%%
    %%%Front A-arm coords (x,y) relative to chassis origin (Fixed points to
    %%%ground) looking from front
    UFWX = -234.92; UFWY = 304.15; %Upper Front Arm
    URWX = -234.92; URWY = 299.15; %Upper Rear Arm
    BFWX = -164.67; BFWY = 141.22; %Lower Front Arm
    BRWX = -169.65; BRWY = 140.74; %Lower Rear Arm
    %%%Making 2D
    UWX=(UFWX+URWX)/2;
    UWY=(UFWY+URWY)/2;
    BWX=(BFWX+BRWX)/2;
    BWY=(BFWY+BRWY)/2;

    %%%Initial Spherical Holder Position
    IUSHX = -504.25; IUSHY = 300.7; %Upper A-arm Front
    IBSHX = -530; IBSHY = 105.7; %Lower A-arm Front

    %%%King Pin Inclination Rear & Front
    KPI = 7.5;
    %%%Tyre Diametre (Rideheight)
    Tyre = 203.2;
    %Distance of Tyre from spherical holder (mm)
    Tyredist = 150; %arbitrary value

%%%----------LengthsofLinks----------%%%
%%%Front Suspension
%%%Upper Front A-arm Link (c)
UWL = sqrt((IUSHX-UWX)^2+(IUSHY-UWY)^2); %
%%%Lower Front A-arm Link (a) 
BWL = sqrt((IBSHX-BWX)^2+(IBSHY-BWY)^2); %
%%%Upper Spherical to Lower Spherical (b)
USBSL = sqrt((IUSHX-IBSHX)^2+(IUSHY-IBSHY)^2); %
%%%Upper Chassis Point to Lower Chassis Point (d)
UCBCL = sqrt((UWX-BWX)^2+(((UWY-BWY)^2))); %

%%%Calc Chassis angle
Chassang = (abs(atan((UWY-BWY)/(UWX-BWX)))); %

%%%----------4BarEquations----------%%%
%%%Relation to wheel travel angle of upper a-arm from +ve x axis
theta2og=atan((IUSHY-(UWY*sign(UWY)))/(IUSHX-(UWX*sign(UWX))))+(pi/2)+abs(atan((UWX-BWX)/(UWY-BWY))); %
theta_inc=0.25*(pi/180);
theta_range=5*(pi/180);
theta2pp=[];
theta2pp(1)=theta2og;
camber=[];
wtrav=[];
%%%Calculating K constants
K1=UCBCL/UWL;
K2=UCBCL/BWL;
K3=(UWL^2+BWL^2+UCBCL^2-USBSL^2)/(2*UWL*BWL);

TYog=IBSHY-(((Tyre*2)-(IUSHY-IBSHY))/2); %
TXog=IBSHX+(Tyredist*sign(IBSHX)); %

%UP
for i=1:theta_range/theta_inc
    A=cos(theta2pp(i))-K1-K2*cos(theta2pp(i))+K3;
    B=-2*sin(theta2pp(i));
    C=K1-(K2+1)*cos(theta2pp(i))+K3;
    theta4=(2*atan((-B-sqrt(B^2-(4*A*C)))/(2*A))); %
    theta3= asin((-UWL*sin(theta2pp(i))+BWL*sin(theta4))/USBSL); %
    
    %Calculating position of upper and lower spherical holders
    %angle from +ve y axis to upper wishbone (underneath)
    %Sign for if the suspension is other side creating -ve x values in
    %which case it should be UWX-UWL*cos(UpperAngle);
    thetaU= -pi + theta2pp(i)+ Chassang;
    UPSHX = UWX + UWL*sign(IUSHX)*cos(thetaU);
    UPSHY = UWY + UWL*sin(thetaU);
    thetaL = -pi + theta4 + Chassang;
    BSHX = BWX + BWL*sign(IBSHX)*cos(thetaL);
    BSHY = BWY + BWL*sin(thetaL);
    
    TY= BSHY-(((Tyre*2)-(IUSHY-IBSHY))/2);
    
    if (i == 1)
        wtrav(i) = 0;
    else 
        wtrav(i) = TY-TYog;
    end
    
    %Calc camber: Angle of USBL from x axis - KPI (Note: -90 doesnt work fix)
    camber(i) = -(((asin(((BSHX*sign(BSHX))-(UPSHX*sign(UPSHX)))/USBSL)))-(KPI*(pi/180)))*(180/pi);
    
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
    theta3= asin((-UWL*sin(theta2pp(j+i))+BWL*sin(theta4))/USBSL);
    
    %Calculating position of upper and lower spherical holders
    %angle from +ve y axis to upper wishbone (underneath)
    %Sign for if the suspension is other side creating -ve x values in
    %which case it should be UWX-UWL*cos(UpperAngle);
    thetaU= -pi + theta2pp(j+i)+ Chassang;
    UPSHX = UWX + UWL*sign(IUSHX)*cos(thetaU);
    UPSHY = UWY + UWL*sin(thetaU);
    thetaL = -pi + theta4 + Chassang;
    BSHX = BWX + BWL*sign(IBSHX)*cos(thetaL);
    BSHY = BWY + BWL*sin(thetaL);
    
    TY= BSHY-(((Tyre*2)-(IUSHY-IBSHY))/2);
    
    wtrav(j+i) = TY-TYog;
    
    %Calc camber: Angle of USBL from x axis - KPI (Note: -90 doesnt work fix)
    camber(j+i) = -(((asin(((BSHX*sign(BSHX))-(UPSHX*sign(UPSHX)))/USBSL)))-(KPI*(pi/180)))*(180/pi);
    
    if (j < theta_range/theta_inc)
        theta2pp(j+i+1)=theta2pp(j+i)-theta_inc;
    end
end


[~, sort_idx] = sort(wtrav);

plot(camber(sort_idx), wtrav(sort_idx), '-*')
xlabel('camber (degrees)')
ylabel('Wheel Travel (mm)')
title('Camber vs Wheel Travel')
x = [245; 307.03]; %Front Upper
y = [245; 393.73]; %Rear Upper
z = [105.94; 145.76]; %Front Lower
k = [105.94; 145.76]; %Rear Lower

%Spherical coords
u = [502.56; 298.98]; %Front Upper A-arm
l = [531.71; 104.46]; %Front Lower A-arm
KPI = 7.5;
d = 406.4;
sim('SuspensionModel');
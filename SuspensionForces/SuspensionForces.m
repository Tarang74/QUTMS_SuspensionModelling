%% HOW TO USE

% adjust inputs to match suspension parameters
% setup loadcases to match what long and lat accel you have (in gs)
% run
% copy results into ansys
% ???
% profit

%% INPUTS

% Car constants
mass_CarDriver = 300; % kg
height_CenterGravity = 0.3; % m
track = 1.5; % m
wheelbase = 2; % m
radius_TireLoaded = 0.24; % m

% Inboard (P1)
Inboard = [
    [-460,188,225]; % Inboard_UpFore
    [-705,200,188]; % Inboard_UpAft
    [-530,225,65]; % Inboard_LowFore
    [-733,230,87]; % Inboard_LowAft
    [-590,250,110]; % Inboard_PushPull
    [-500,130,180]; % Inboard_TieToe
];

% Outboard (P2)
Outboard = [
    [-610,505,352]; % Outboard_UpFore
    [-610,505,352]; % Outboard_UpAft
    [-590,520,135]; % Outboard_LowFore
    [-590,520,135]; % Outboard_LowAft
    [-610,505,352]; % Outboard_PushPull
    [-500,520,200] % Outboard_TieToe
];

% Location of applied forces from Tire on ground
AppliedForcesLocations = [
	[-587,580,0]; % Fx
    [-587,580,0]; % Fy
    [-587,580,0]; % Fz
];

% Moment Center
MomentCenter = [0,0,0];

%% LOAD CASES

% {Name, Failure Mode, [Long G, Lat G], [WT Lat, WT Long], [Fx,Fy,Fz],
% [Fx-Added Height for Driveshafts]}
loadcases = {
    "None", "Fatigue", [0,0], [], [], 0;
    "Static", "Fatigue", [0,0], [], [], 0;
    "Max Braking", "Fatigue", [-1.25,0], [], [], 0;
    "Max Turn", "Fatigue", [0,2], [], [], 0;
    "Max Speed", "Fatigue", [0,0], [], [], 0;
    "Max Accel", "Fatigue", [1,0], [], [], 0;
    "Combined 1", "Fatigue", [-1,1], [], [], 0;
    "Combined 2", "Fatigue", [0.2,1], [], [], 0;
    "Max Braking", "Yield/Buckle", [-1.9,0], [], [], 0;
    "Max Turn", "Yield/Buckle", [0,2.75], [], [], 0;
    "Max Speed", "Yield/Buckle", [0,0], [], [], 0;
    "Max Accel", "Yield/Buckle", [1.5,0], [], [], 0;
    "Combined 1", "Yield/Buckle", [-0.5,2.5], [], [], 0;
    "Combined 2", "Yield/Buckle", [0.2,2.2], [], [], 0;
    "Max Bump", "Yield/Buckle", [0,0], [], [], 0;
    "Inside No WT", "Yield/Buckle", [0,-2], [], [], 0;
    "Reverse Braking", "Yield/Buckle", [1.75,0], [], [], 0
};

% index of last fatigue load cases, note that all fatigue cases must be
% grouped together at the beginning of the load case setup
numFatigue = 8;

longLatG = cell2mat(loadcases(:,3));

% calculate WT lat and long
WT_lateral = mass_CarDriver * (longLatG(:,2) * 9.81) * height_CenterGravity / track;
WT_longitudinal = mass_CarDriver * (longLatG(:,1) * 9.81) * height_CenterGravity / wheelbase;

WT = [WT_lateral,WT_longitudinal];

% manually adjust WT lateral / longitudal values
% "Inside No WT"
WT(16,1) = 0;

% "Reverse Braking"
WT(17,2) = 0;

% store WT in loadcases cell array
loadcases(:,4) = mat2cell(WT, ones(1,size(WT,1)));

% calculate forces

Fz_applied = (mass_CarDriver * 9.81 / 4) + (WT(:,1) + WT(:,2))/2;
Fz_percent = Fz_applied / (mass_CarDriver * 9.81);

Fy_car = mass_CarDriver * (longLatG(:,2) * 9.81);
Fx_car = mass_CarDriver * (longLatG(:,1) * 9.81);
    
Fy_applied = Fy_car .* Fz_percent;
Fx_applied = Fx_car .* Fz_percent;

loadcaseForces = [Fx_applied, Fy_applied, Fz_applied];

% manually adjust value of final forces
% "None"
loadcaseForces(1,:) = [0,0,0];

% "Max Bump"
loadcaseForces(15,3) = mass_CarDriver*9.81/4 * 5;

% store forces in loadcases cell array
loadcases(:,5) = mat2cell(loadcaseForces, ones(1,size(loadcaseForces,1)));
%%

% create unit direction vectors of the difference between points
unitVectors = Outboard - Inboard;

for i = 1:6
    unitVectors(i,:) = unitVectors(i,:)/norm(unitVectors(i,:));
end

momentInboard = Inboard - repmat(MomentCenter, 6,1);
Moments = cross(momentInboard, unitVectors, 2);

A = [unitVectors'; Moments';];

%% calculate output forces for each load case
outputForces = zeros(size(loadcases,1),6);
for i = 1:size(loadcases,1)
    % adjust applied forces location according to loadcase
    load_AppliedForcesLocations = AppliedForcesLocations;
    load_AppliedForcesLocations(1,3) = load_AppliedForcesLocations(1,3) + loadcases{i,6};
    
    momentAppliedForcesLocations = load_AppliedForcesLocations - repmat(MomentCenter, 3,1);
    
    AppliedForceVectors = diag(loadcases{i,5});
    
    AppliedForceMoments = cross(momentAppliedForcesLocations, AppliedForceVectors,2);

    % calculate b of Ax=b
    
    momentSum = -sum(AppliedForceMoments,1);
    b = [-loadcases{i,5}, momentSum]';
    
    outputForces(i,:) = A\b;
end

columnNames = ["Up-Fore", "Up-Aft", "Low-Fore", "Low-Aft", "Push/Pull", "Tie/Toe"];
LoadCasesOutputForcesTable = array2table(outputForces, 'VariableNames', columnNames);

%% generate highest operating forces
Fatigue_InTension = min(min(outputForces(1:numFatigue,:),[],1),0);
Buckling = max(max(outputForces, [], 1),0);
Comp = max(max(outputForces,[],1),0);
Tens = min(min(outputForces,[],1),0);

HighestOperatingForces = [
    Fatigue_InTension;
    Buckling;
    Comp;
    Tens;
];
HighestOperatingForcesTable = array2table(round(HighestOperatingForces), 'VariableNames', columnNames, 'RowNames', ["Fatigue (in tension)", "Buckling", "Comp", "Tens"]);

%% display output

disp("Over given loadcases, here is calculated forces:");
disp(LoadCasesOutputForcesTable);
disp("Highest Operating Forces:");
disp(HighestOperatingForcesTable)



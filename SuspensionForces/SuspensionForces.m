%% INPUTS

% Inboard (P1)
Inboard_UpFore = [-460,188,225];
Inboard_UpAft = [-705,200,188];
Inboard_LowFore = [-530,225,65];
Inboard_LowAft = [-733,230,87];
Inboard_PushPull = [-590,250,110];
Inboard_TieToe = [-500,130,180];

Inboard = [
    Inboard_UpFore;
    Inboard_UpAft;
    Inboard_LowFore;
    Inboard_LowAft;
    Inboard_PushPull;
    Inboard_TieToe
];

% Outboard (P2)
Outboard_UpFore = [-610,505,352];
Outboard_UpAft = [-610,505,352];
Outboard_LowFore = [-590,520,135];
Outboard_LowAft = [-590,520,135];
Outboard_PushPull = [-610,505,352];
Outboard_TieToe = [-500,520,200];

Outboard = [
    Outboard_UpFore;
    Outboard_UpAft;
    Outboard_LowFore;
    Outboard_LowAft;
    Outboard_PushPull;
    Outboard_TieToe;
];

% Location of applied forces from Tire on ground
Fx = [-587,580,0];
Fy = [-587,580,0];
Fz = [-587,580,0];

% Moment Center
MomentCenter = [0,0,0];

%% LOAD CASE

loadCase_idx = 2;

%%

% create unit direction vectors of the difference between points
unitVectors = Outboard - Inboard;

for i = 1:6
    unitVectors(i,:) = unitVectors(i,:)/norm(unitVectors(i,:));
end

momentInboard = Inboard - repmat(MomentCenter, 6,1);
Moments = cross(unitVectors, momentInboard, 2);

A = [unitVectors'; Moments';];
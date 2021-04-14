%%D:\Mega\Programming\FYP\Matlab

Apro = 0;  %initialize with zero acceleration
Tao = 0.5; %delay
Apro = (u - Apro)/ Tao; 
%Apro = propulsive force
%u = control signal


%%Constant based on Myvi
Area    = 2.306; %area
Cd      = 0.32; %drag coefficient
pho     = 1.225;%air density
m       = 1055; %mass of vehicle + driver
Cr      = 0.01; %coefficient of rolling friction
g       = 9.81; %gravity
slope   = 0;    %slope angle

Fad = 0.5 * Cd * Area * pho * v * v; %air drag force
Frr = Cr * m * g * cos(slope);
Fslope = m * g * sin(slope);

Adrag = (Fad + Frr + Fslope) / m;
Aeff = Apro - Adrag;  %effective acceleration





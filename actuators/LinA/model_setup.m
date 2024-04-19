% clean
clear all;
clc;

%% transfer functions
s = tf('s');

%% user parameters
desired_bandwidth = 20; %desired position closed loop bandwidth [Hz]
position_scaling = 500; %scaling factor
current_scaling = 8; %current loop needs to be faster
mspc = 4; %min number of sample per cycle (>= 2 for Shannon)

%% load (motor side)
Jlm = 0;
Jgb = 8e-8; %gearbox inertial motor side [Kg*m2]
Klt = 1/(1000*2*pi*85.58*35/(24*624)); %gearhead to load ratio [m/rad]
grtot = 1000*2*pi*85.58/24; %total gear ratio


%% plant (motor)
L = 0.877e-3; %inductance [H]
R = 0.782; %resistance [Ohm]
Kt = 42.9e-3; %torque constant [Nm/A]
Kv = 223*2*pi/60; %speed constant [rad/(s*V)]
Ke = Kt; %back-emf constant [V*s/rad]
Jm = 1.55e-6 + Jgb; %motor inertia [Kg*m2]
Uin = 48; %input voltage [V]
Io = 0.151; %no load current [A]
wo = 7950*2*pi/60; %no load speed [rad/s]
Umax = 48; %nominal voltage [V]: tune this so that peak matches
Imax = 12; %max current [A]
In = 2.48; %nominal current [A]
tauGp = 0.663e-3; %mechanical time constant [s]
B = Kt*Io/wo; %viscous friction [Nm*s/rad]
B0 = 0; %static friction [Nm] --> TODO: tune it according to experiment

%% gearhead
gr = 624/35; %gearhead reduction (motor speed / load speed)

%% current controller
Tsi = 4e-5; %current loop sampling time [s]
Gi = 1/(R+s*L); %(B+s*Jm)/((B+s*Jm)*(R+s*L)+Ke*Kt); %current G(s): [V --> A]
scale = evalfr(Gi,0); %get original gain at zero frequency (used for scaling)
wcanc = max(real(pole(Gi))); %select slow pole to cancel
tau = 0.5*1/abs(wcanc); %slow pole time constant
wc = 2*pi*min(current_scaling*desired_bandwidth, 1/(mspc*Tsi)); %desired closed loop bandwidth
Ki_si = wc/scale; %integral gain [V/(A*s)]
Kp_si = tau*Ki_si; %proportional gain [V/A]
Ri = Kp_si + Ki_si/s; %current R(s): [A --> V]
Li = Gi*Ri; %current L(s) [A --> A]
Fi = minreal(Li/(1+Li)); %current F(s) [A --> A]

%% velocity controller
Tsw = 4e-4;
Kiw_si = 0.159105; %integral gain [A*rad]
Kpw_si = 0.012661; %proportional gain [A*s/rad]

%% velocity observer
Kto = Kt; %observer torque constant
Kte = Ke; %observer electrical constant
Lop = 0.4; %velocity observer position correction gain []
Low = 100; %velocity observer velocity correction gain [Hz]
Lol = 0.033e-3; %velocity observer load correction gain [Nm/rad]
LB = (1e-8)*2*pi/60; %velocity observer friction [Nm/rad]
LJ = 1e-7; %velocity observer inertia [Kg*m2]

%% position controller
Tsp = 4e-4;
FF_a_si = 0; %acceleration feed forward gain [A*s/rad]
FF_w_si = 0; %velocity feed forward gain [A*s2/rad]
Gp = Kt/(B+s*Jm); %Fi*Kt/(B+s*Jm); %position G(s) [A --> rad]
scale = evalfr(Gp, 0); %get original gain at zero frequency (used for scaling)
wcanc = max(real(pole(Gp))); %select slow pole to cancel
tau = 1/abs(wcanc); %0.075; %slow pole time constant
wc = 2*pi*min(position_scaling*desired_bandwidth, 1/(mspc*Tsp)); %desired closed loop bandwidth [Hz]
Kip_si = 20; %wc/scale; %32.117; %integral gain [A/(rad*s)]
Kpp_si = 0.8; %proportional gain [A/rad]
Kdp_si = 0.015; %derivative gain [A*s/rad] (0.11*Kup*Tup)
Rp = Kpp_si + Kip_si/s + s*Kdp_si/(Kdp_si/(10*Kpp_si)*s + 1); %position: R(s) [rad --> A]
Lp = Rp*Gp; %position L(s): [rad --> rad]
Fp = minreal(Lp/(1+Lp)); %position F(s): [rad --> rad]

%% path planner
Tspp = 4e-5;
max_ppa_si = 500000*2*pi/60; %37889*2*pi/60; %(Kt*Imax)/(Jm+Jgb+Jlm); %max acceleration [rad/s^2]
max_ppv_si = wo*(48/36); %0.63*4172*2*pi/60;
Kppph_si = max_ppa_si/max_ppv_si; %0.0067*max_ppa_si;
Kpppl_si = Kppph_si/10; %0.000247*max_ppa_si;
Hpp = 12.5;


% clean
clear all;
clc;

%% transfer functions
s = tf('s');

%% user parameters
desired_bandwidth = 20; %desired position closed loop bandwidth [Hz]
position_scaling = 40; %scaling factor
current_scaling = 30; %current loop needs to be faster
mspc = 4; %min number of sample per cycle (>= 2 for Shannon)

%% load (motor side)
Jlm = 0;
Jgb = 1.4e-6; %gearbox inertial motor side [Kg*m2]
Klt = 1/(1000*2*pi*174.72*6/(48*91)); %gearhead to load ratio [m/rad]
grtot = 1000*2*pi*174.72/48;r %total gear ratio

%% plant (motor)
L = 0.512e-3; %inductance [H]
R = 0.529; %resistance [Ohm]
Kt = 72.8e-3; %torque constant [Nm/A]
Kv = 131*2*pi/60; %speed constant [rad/(s*V)]
Ke = Kt; %back-emf constant [V*s/rad]
Jm = 7.8e-6 + Jgb; %motor inertia [Kg*m2]
no = 4670*2*pi/60; %no load speed [rad/s]
Uin = 48; %input voltage [V]
Io = 0.248; %no load current [A]
wo = 4670*2*pi/60; %no load speed [rad/s]
Umax = 48; %36; %nominal voltage [V]
Imax = 22; %max current [A]
In = 4.27; %nominal current [A]
tauGp = 0.778e-3; %mechanical time constant [s] 
B = Kt*Io/wo; %viscous friction [Nm*s/rad]
B0 = 0; %static friction [Nm] --> TODO: tune it according to experiment

%% gearhead
gr = 91/6; %gearhead reduction (motor speed / load speed)

%% current controller
Tsi = 4e-5; %current loop sampling time [s]
Gi = 1/(R+s*L); %(B+s*Jm)/((B+s*Jm)*(R+s*L)+Ke*Kt); %current G(s): [V --> A]
scale = evalfr(Gi,0); %get original gain at zero frequency (used for scaling)
wcanc = max(real(pole(Gi))); %select slow pole to cancel
tau = 1/abs(wcanc); %slow pole time constant
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
scaling_factor = 1/Tsp;
FF_a_si = 0; %acceleration feed forward gain [A*s/rad]
FF_w_si = 0; %velocity feed forward gain [A*s2/rad]
Gp = Fi*Kt/(B+s*Jm); %position G(s) [A --> rad]
scale = evalfr(Gp, 0); %0.005*evalfr(Gp, 0); %get original gain at zero frequency (used for scaling)
wcanc = max(real(pole(Gp))); %max(pole(Gp)); %select slow pole to cancel
tau = 1/abs(wcanc); %0.12*1/abs(wcanc); %slow pole time constant
wc = 2*pi*min(position_scaling*desired_bandwidth, 1/(mspc*Tsp)); %desired closed loop bandwidth [Hz]
Kip_si = 25; %integral gain [A/(rad*s)]
Kpp_si = 2; %proportional gain [A/rad]
Kdp_si = 0.028; %derivative gain [A*s/rad] (0.11*Kup*Tup)
Rp = Kpp_si + Kip_si/s + s*Kdp_si/(Kdp_si/(10*Kpp_si)*s + 1); %position: R(s) [rad --> A]
Lp = Rp*Gp; %position L(s): [rad --> rad]
Fp = minreal(Lp/(1+Lp)); %position F(s): [rad --> rad]

%% path planner
Tspp = 0.01;
max_ppa_si = 90000*2*pi/60; %(Kt*Imax)/(Jm+Jgb+Jlm); %max acceleration [rad/s^2]
max_ppv_si = wo*(48/36);
Kppph_si = max_ppa_si/max_ppv_si; 
Kpppl_si = Kppph_si;
Hpp = 12.5;



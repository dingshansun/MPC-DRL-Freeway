function [tao, kai, yita, rou_max, sigma, am, v_free, rou_crit, alpha, T, lambda, Lm, v_control, v_min, Co2, xi_ramp, xi_speed] = parameters_esti
tao=15.5; % unit: s
kai=45;
yita=54; % km^2/h
rou_max=165; % veh/km/lane
sigma=0.0100;
am=1.960;
v_free=102; % km/h
rou_crit=36.5; % veh/km/lane
alpha=0.085;
T=10; % unit: s
lambda=2;
Lm=1120; % unit: m
v_control=200; % km/h
v_min=7;
% ro=1;
% Co1=4000; % veh/h
Co2=2000; % veh/h
xi_ramp=0.4;
xi_speed=0.4;
end


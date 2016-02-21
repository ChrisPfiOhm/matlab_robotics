% differntial_drive_example


close all; 

clear all; 

T = 20; 	% sec
t = 0:0.01:T; 

d = 0.3; 	         % meters
r_wheel = 0.08;   % meters

v_l =  50*2*pi*r_wheel/ T;        % meters / sec
v_r = 48.125*2*pi*r_wheel/ T; % meters / sec

d_s_l = v_l * t; 
d_s_r = v_r * t; 

d_s_k = (d_s_l + d_s_r) *0.5;
r = d*0.5 * (d_s_l + d_s_r) / (d_s_r - d_s_l); 

d_phi = (d_s_r - d_s_l) / d; 
d_s     = 2*r *sin(d_phi / 2); 

d_x = d_s .* cos(d_phi ./ 2); 
d_y = d_s .* sin(d_phi ./ 2); 


figure(1); 
axis equal; 
plot(d_x, d_y); 


figure(2); 
plot(t, d_phi)

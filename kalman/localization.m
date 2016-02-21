clear;
close all; 
clear all; 

load -ascii poses.txt;

samples = size(poses,1)-1;

odom   =  [poses(2:samples+1,1), poses(2:samples+1,2)];
gps    =  [poses(2:samples+1,3), poses(2:samples+1,4)];
ground =  [poses(2:samples+1,5), poses(2:samples+1,6)];

dT = 0.999;
A = [1   0 dT   0;
	0   1   0 dT;
	0   0   1   0;
	0   0   0   1];
	 
B = [1 0 0 0;
	   0 1 0 0;
	   0 0 0 0;
	   0 0 0 0];
	      
H = [1 0 0 0;
	 0 1 0 0];

x = [poses(1,1) poses(1,2) 0 0]';

P = [0 0 0 0;
        0 0 0 0;
        0 0 0 0;
        0 0 0 0];

%Systemrauschen
ax = 10e-7;
ay = 10e-7;
Q = [0.25*ax 0 0.5*ax 0;
         0 0.25*ay 0 0.5*ay;
         0.5*ax 0 ax 0;
         0 0.5*ay 0 ay];
         
%Messrauschen
r = 2;
R = [r 0; 0 r];

u = [0; 0; 0; 0];
odom_alt = poses(1,1:2);
for k=1:samples,

	% GPS Messwerte
	z = [gps(k,1); gps(k,2)];
	
	% Berechnung des Steuervektors aus Odometriedaten
	%v = (odom(k,1:2) - odom_alt(1:2))' ./ dT;
	u(1:2) = odom(k,1:2) - odom_alt(1:2); %v * dT;
    
  
  odom_alt = odom(k,1:2);
        
        	% Schätzwert berechnen
	x_prio =  A*x(:,k) + B*u;
	
	% Unsicherheit der Schätzung berechnen
	P_prio = A * P * A' + Q;
		
	% "beobachtbaren" Schätzwert ermitteln
	y_e = H*x_prio;

	% Kalman-Verstärkung berechnen
	K = P_prio * H' * inv(H * P_prio * H' + R);

	% Zustand aktualisieren
	x(:,k+1) = x_prio + K * (z - y_e);
	P = (eye(4) - K * H) * P_prio;

end

plot(x(1,:),x(2,:), 'r');
hold on;
plot(ground(:,1),ground(:,2), 'g');
plot(odom(:,1),odom(:,2), 'b');
plot(gps(:,1),gps(:,2), 'x');
hold off
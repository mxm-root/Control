%==========================================================================
% Tutorial Control methods
% Topic: Linear quadratic (LQ) trajectory-tracking problem
% Authors: M.Trifonov 
% Email: trifonov.m@yahoo.com
% Date(dd-mm-yyyy): 08-04-2019
%==========================================================================
clc; clear all; close all
% Initial data for state-space model
a = -1; b = 1; c = 1;
d = 1; xi = 1;
% Components of weight matrixes F, Q, R
f = 0.05;
q = 50;
r = 0.2;
% Elements of the Riccati an linear vector diff eq equations
l=a-q*d/(r+q*d^2);
m=1./(r+q*d^2);
n=q*d/(r+q*d^2);
v=q-q^2*d^2/(r+q*d^2);
w=q-q^2*d^2/(r+q*d^2);
% Simulation parameters
tk = 8.5;  % end time
dt = 0.01; % step time
% Initial state for xx, k, g
xx = -5;  %initial state for a desired trajectory
k = f;
g = f*((xx^3 + 3*xx^2 - 6*xx - 8)/4); %
% Cycle in reverse time, calculation of the Riccati equation
i = 0;
for t = tk:-dt:0 
    % indixes
    i = i + 1;
    xx = xx + 0.01;
    % Desired trajectory
    z = (xx^3 + 3*xx^2 - 6*xx - 8)/4;
    % Riccati equation
    k = k-dt*(m*k^2 - 2.*l*k - v);   
    % linear vector diff eq
    g = g-dt*((k*m-l)*g+(k*n-w)*z + k*xi);   
    %
    tm(((tk/dt)+2)-i,1) = t;
    zm(((tk/dt)+2)-i,1) = z;
    km(((tk/dt)+2)-i,1) = k;    
    gm(((tk/dt)+2)-i,1) = g;
    
end

% Cycle for a calculation of the linear time-variant dynamic system
x = -1000; % 0 % initial state of the linear system
j = 0;
for t = 0:dt:tk
    j = j + 1;
    % Optimal control
    u = (gm(j,1)+q*d*zm(j,1)-(km(j,1)+q*d)*x)/(r+q*d^2);
    % State space model with external disturbance - Xi
    x = x + dt*(a*x + b*u + xi); % b=1
    y = c*x + d*u;              % c=1
    % Tracking error
    e = zm(j,1)-y;
    % To plotting 
    Y(j,1) = y;
    Z(j,1) = zm(j,1);
    E(j,1) = e;
end
% Plotting
figure(1);hold on;grid on
plot(tm(:,1),Z(:,1),'r','Linewidth',2);
plot(tm(:,1),Y(:,1),'--g','Linewidth',2);grid on
xlabel('\itTime \rm(\its\rm)'); ylabel('\itCurrent output y\rm(\itt\rm) \it& desired output z\rm(\itt\it)');
legend('\itz\rm(\itt\rm)','\ity\rm(\itt\rm)');
figure(2);hold on;grid on;
plot(tm(:,1),E(:,1),'b','Linewidth',2);
xlabel('\itTime \rm(\its\rm)'); ylabel('\itTracking error e\rm(\itt\rm)');

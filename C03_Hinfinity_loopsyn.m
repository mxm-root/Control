%==========================================================================
% Tutorial Control methods
% Topic: H-infinity optimal method for loopshaping control synthesis (LTI)
% Authors: M.Trifonov 
% Email: trifonov.m@yahoo.com
% Date(dd-mm-yyyy): 15-05-2017
%==========================================================================
clc; clear all; close all

% Initial data
ag=[-0.0936821262248705 1.06687351415526 -0.0237128881456641 -9.81000023640766 0.000145779305397252;...
    -0.000140591697840166 -2.09593256176475 0.996982035659188 -1.57393134006257e-05 -4.19758225184398e-06;...
    -0.0320790842867755 -28.0513094446784 -1.94268181162338 -1.25830260856921e-05 3.65902102135738e-05;...
    0 0 1 0 0;...
    -8.88178419700125e-13 316.428314486973 0 -316.428314486974 0];
bg = [-5.27608985971698 -0.926682483709498 -2.10959133319318;...
    -0.00766312054783688 -0.103553051368704 -0.249784789375376;...
    17.6570528576021 -15.7594503877227 -34.2057595816490;...
    0 0 0;...
    0 0 0];
cg = [0 1 0 0 0;0 0 0 1 0];
dg = zeros(2,3);

G = ss(ag,bg,cg,dg);
G.InputName = {'canard','elevon_in','elevon_out'};
G.OutputName = {'alpha','theta'};  

plant_poles = pole(G);
plant_zeros = tzero(G);

s = zpk('s'); % Laplace variable s
Gd = 8/s;

[K,CL,GAM] = loopsyn(G,Gd);
GAM

L = G*K;  % form the compensated loop L

T = feedback(L,eye(2));  %
T.InputName = {'alpha command','theta command'};
S = eye(2)-T;

figure(13);
step(T(1,1),'blue',5);grid on;hold on
step(T(1,2),'r',5);grid on;hold on
title('Responses to step commands for alpha');

Kr = reduce(K,8);
order(Kr)

AA=Kr.A;
BB=Kr.B;
CC=Kr.C;
DD=Kr.D;
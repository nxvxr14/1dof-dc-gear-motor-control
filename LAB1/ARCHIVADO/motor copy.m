clc
clear
close all
syms x1 x2 x3 Vin    % x1 = corriente, x2 = wm, x3 = thetam
syms Jm Jl Bm Bl Ki Kv R L n m g d
Jt = Jm + (Jl/n^2);
f1 = (1/L)*(Vin - R*x1 - Kv*x2);
f2 = (1/Jt)*(Ki*x1 - (Bm + Bl/n^2)*x2 - (m*g*d*sin(x3/n))/n);
f3 = x2;
F = [f1; f2; f3]
X = [x1; x2; x3]
Y = [x2;x3]
A = jacobian(F,X)
B = jacobian(F,Vin)
C = jacobian(Y,X)


% thetaL0 = 0
% x30 = n*thetaL0
% x10 = (m*g*d*sin(x30/n))/ki
% Vin0 = R*x10

%PARAMETROS INICIALES
Vin = 7; %Volts
R0 = 3.9654; % [ohms]
L0= 0.51e-3;
Kv0 = 0.03212e-9;
Ki0 = 0.005012; % Constante electromotriz [V/rad/s] = constante de torque [V/rad/s]
n0 = 120; % Relación de engranajes
Jm0 = 7.83067e-12; % Momento de inercia del motor
Jl0 = 0.154364; % Momento de inercia de la barra
Bm0 = 5.862e-9; % Fricción viscosa del motor
Bl0 = 0.147; % Fricción viscosa de la barra
m0 = 0.1045; % Masa de la barra [kg]
d0 = 0.06587; % Distancia del eje al centro de gravedad [m]
g0 = 9.81; % gravedad [m/s^2]
thetaL0 = 0;
x30 = n0*thetaL0;
x10 = (m0*g0*d0*sin(x30/n0))/Ki0;
x20 = 0;
Vin0 = R0*x10;

AL = double(subs(A,{x1,x2,x3,Vin,R,L,Ki,Kv,Jm,Jl,Bm,Bl,n,g,d,m},{x10,x20,x30,Vin0,R0,L0,Ki0,Kv0,Jm0,Jl0,Bm0,Bl0,n0,g0,d0,m0}))
BL = double(subs(B,{x1,x2,x3,Vin,R,L,Ki,Kv,Jm,Jl,Bm,Bl,n,g,d,m},{x10,x20,x30,Vin0,R0,L0,Ki0,Kv0,Jm0,Jl0,Bm0,Bl0,n0,g0,d0,m0}))
CL = [0 1 0; 0 0 1]

% CL = double(subs(C,{x1,x2,x3,Vin,R,L,Ki,Kv,Jm,Jl,Bm,Bl,n,g,d,m},{x10,x20,x30,Vin0,R0,L0,Ki0,Kv0,Jm0,Jl0,Bm0,Bl0,n0,g0,d0,m0}))
DL = [0;0]

sys = ss(AL,BL,CL,DL)
eig(AL)
figure(1)
step(sys)
grid
% figure(2)
% bodemag(sys)
% grid

% w3db = 0.87
% ws = 2*w3db
% Tm = 2*pi/ws

w3db = 2.61
ws = 20*w3db
Tm = 2*pi/ws

% w40db = 75.6
% ws = 2*w40db
% Tm = 2*pi/ws
% 
% Ts = 4.55
% Tm = 4.55/20

% Tm = 0.2


% sysd = c2d(sys,Tm)
% figure(1)
% step(sys,sysd)
% grid
% figure(2)
% bodemag(sys,sysd)
% grid
% 
% Gz = tf(sysd)
% Gz = zpk(sysd)
% 
% GL = sysd.a
% HL = sysd.b
% CL1 = sysd.c
% DL1 = sysd.d
% 
% eig(GL)
% eig(AL)
% abs(eig(GL))
%% Modelo de incertidumbre
Tol = 20;
Lc = ureal('Lc',L0,'Percentage',[-Tol Tol]); 
Rc = ureal('Rc',R0,'Percentage',[-Tol Tol]); 
thetaL0 = 0
x30 = n0*thetaL0
Ac = [ -Rc/L0,-Kv0/L0,0;
    Ki0/(Jm0 + Jl0/n0), -(Bm0 + Bl0/n0)/(Jm0 + Jl0/n0), -(d0*g0*m0*cos(x30/n0))/(n0*(Jm0 + Jl0/n0));
    0, 1, 0]
Bc = [1/L0;
    0;
    0]
CL = [0 0 1/n0]
DL = [0]
sysc = ss(Ac,Bc,CL,DL)
step(sysc)
grid
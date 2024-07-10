clc
clear
close all
syms x1 x2 x3 Vin    % x1 = corriente, x2 = wm, x3 = thetam
syms Jm Jl Bm Bl Ki Kv R L n m g d
Jt = Jm + Jl/n
f1 = (1/L)*(Vin - R*x1 - Kv*x2)
f2 = (1/Jt)*(Ki*x1 - (Bm + Bl/n)*x2 - m*g*d*sin(x3/n))
f3 = x2

F = [f1; f2; f3]
X = [x1; x2; x3]
A = jacobian(F,X)
B = jacobian(F,Vin)

% thetaL0 = 0
% x30 = n*thetaL0
% x10 = (m*g*d*sin(x30/n))/ki
% Vin0 = R*x10

R0 = 10
L0 = 0.04
Ki0 = 2.3
Kv0 = 1.5
Jm0 = 0.01
Jl0 = 0.02
Bm0 = 0.2
Bl0 = 0.05
n0 = 120
g0 = 9.81
d0 = 0.1
m0 = 0.1
thetaL0 = 0
x30 = n0*thetaL0
x10 = (m*g*d*sin(x30/n0))/Ki0
x20 = 0
Vin0 = R*x10

AL = double(subs(A,{x1,x2,x3,Vin,R,L,Ki,Kv,Jm,Jl,Bm,Bl,n,g,d,m},{x10,x20,x30,Vin0,R0,L0,Ki0,Kv0,Jm0,Jl0,Bm0,Bl0,n0,g0,d0,m0}))
BL = double(subs(B,{x1,x2,x3,Vin,R,L,Ki,Kv,Jm,Jl,Bm,Bl,n,g,d,m},{x10,x20,x30,Vin0,R0,L0,Ki0,Kv0,Jm0,Jl0,Bm0,Bl0,n0,g0,d0,m0}))
CL = [0 0 1/n0]
DL = [0]
sys = ss(AL,BL,CL,DL)
eig(AL)
%%                      Salida Lineal
clc
clear
close all
syms x1 x2 x3 u M g R L x10
f1 = x2;
f2 = g - x3^2/(M*x1);
f3 = -R*x3/L + u/L;
F = [f1; f2; f3]
X = [x1; x2; x3]
A = jacobian(F,X)
B = jacobian(F,u)
x20 = 0
x30 = sqrt(M*g*x10)
u0 = x30*R
Ap = subs(A,{x1,x2,x3,u},{x10,x20,x30,u0})
Bp = subs(B,{x1,x2,x3,u},{x10,x20,x30,u0})

% Modelo numerico
Mo = 0.2  % Kg
go = 9.81   % m/s^2
Ro = 8   % Ohm
Lo = 0.7  % H
x10n = 0.1  % mt
x20n = 0
x30n = sqrt(Mo*go*x10n)
u0n = x30n*Ro
AL = double(subs(A,{x1,x2,x3,u,M,g,R,L},{x10n,x20n,x30n,u0n,Mo,go,Ro,Lo}))
BL = double(subs(B,{x1,x2,x3,u,M,g,R,L},{x10n,x20n,x30n,u0n,Mo,go,Ro,Lo}))

AL1 = double(subs(Ap,{x10,M,g,R,L},{x10n,Mo,go,Ro,Lo}))
BL1 = double(subs(Bp,{x10,M,g,R,L},{x10n,Mo,go,Ro,Lo}))
CL = [1 0 0;
      0 0 1]
DL = [0;
     0]
sys = ss(AL,BL,CL,DL)
Gp = tf(sys)
Gp = zpk(sys)
eig(AL)

xini = [0; 0; 0]
%% Modelo de incertidumbre
Tol = 20;
Lc = ureal('Lc',Lo,'Percentage',[-Tol Tol]); 
Rc = ureal('Rc',Ro,'Percentage',[-Tol Tol]); 
Mc = ureal('Mc',Mo,'Percentage',[-Tol Tol]);
a22 = ureal('a22',-(2*(Mo*go*x10n)^(1/2)),'Percentage',[-Tol Tol]);
Ac = [0 1 0;
    go/x10n, 0, a22/(Mc*x10n);
    0, 0, -Ro/Lo]
Bc = [0;
     0;
     1/Lo]
sysc = ss(Ac,Bc,CL,DL) 
figure(1)
bode(sysc)
grid
figure(2)
step(sysc,0.1)
grid
%% Discretización
bodemag(sys)
grid
%%
w3db = 11.5
ws = (2*w3db)*10
Tm = 2*pi/ws
%%
w40db = 966
ws = 2*w40db
Tm = 2*pi/ws
%%
Tm = 0.01
%%
sysd = c2d(sys,Tm)
figure(1)
bodemag(sys,sysd)
grid
figure(2)
step(sys,sysd,0.4)
grid
%%
GL = sysd.a
eig(GL)
HL = sysd.b
sysd.c
sysd.d
sysd = ss(GL,HL,CL,DL,Tm)
[GL,HL] = c2d(AL,BL,Tm)
%%               Salida No Lineal
clc
clear
close all
syms x1 x2 x3 u M g R L x10
f1 = x2;
f2 = g - x3^2/(M*x1);
f3 = -R*x3/L + u/L;
F = [f1; f2; f3]
X = [x1; x2; x3]
Y = [x1; x3*u]
A = jacobian(F,X)
B = jacobian(F,u)
C = jacobian(Y,X)
D = jacobian(Y,u)
Mo = 0.2  % Kg
go = 9.81   % m/s^2
Ro = 8   % Ohm
Lo = 0.7  % H
x10n = 0.1  % mt
x20n = 0
x30n = sqrt(Mo*go*x10n)
u0n = x30n*Ro
AL = double(subs(A,{x1,x2,x3,u,M,g,R,L},{x10n,x20n,x30n,u0n,Mo,go,Ro,Lo}))
BL = double(subs(B,{x1,x2,x3,u,M,g,R,L},{x10n,x20n,x30n,u0n,Mo,go,Ro,Lo}))
CL = double(subs(C,{x1,x2,x3,u,M,g,R,L},{x10n,x20n,x30n,u0n,Mo,go,Ro,Lo}))
DL = double(subs(D,{x1,x2,x3,u,M,g,R,L},{x10n,x20n,x30n,u0n,Mo,go,Ro,Lo}))
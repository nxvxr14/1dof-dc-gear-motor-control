clc
clear
close all
syms x1 x2 x3 u1 u2 u3 a b c d
f1 = -a*x2 - b*x1^2 + u1*u2;
f2 = -c*x3*x2 + u2^2 ;
f3 = -d*x3 + u3*x1 + u1;

F = [f1; f2; f3]
X = [x1; x2; x3]
U = [u1; u2; u3]
A = jacobian(F,X)
B = jacobian(F,U)
x10 = 2;
x20 = 4;
x30 = 4;
u20 = sqrt(c*x30*x20)
u10 = (a*x20 + b*x10^2)/u20
u30 = (d*x30 - u10)/x10

A = subs(A,{x1,x2,x3,u1,u2,u3},{x10,x20,x30,u10,u20,u30})
B = subs(B,{x1,x2,x3,u1,u2,u3},{x10,x20,x30,u10,u20,u30})
%%
a1 = 2
b1 = 4
c1 = 1
d1 = 3
x10 = 2;
x20 = 4;
x30 = 4;
u20 = sqrt(c1*x30*x20)
u10 = (a1*x20 + b1*x10^2)/u20
u30 = (d1*x30 - u10)/x10
AL = double(subs(A,{a,b,c,d},{a1,b1,c1,d1}))
BL = double(subs(B,{a,b,c,d},{a1,b1,c1,d1}))
CL = [1 0 0;
       0 0 1]
DL = [0 0 0;
       0 0 0]
yo = CL*[x10;x20;x30] + DL*[u10;u20;u30] % Pto Equilibrio vector salida
xop = [x10; x20; x30]
sys =ss(AL,BL,CL,DL);
eig(AL)

%% Matriz de controlabilidad
n = 3 % Numero de estados
p = 3 % Número de actuadores
r = 2 % Número de salidas
Ahat = [AL zeros(n,r);
        -CL zeros(r,r)];
Bhat = [BL;
        -DL]    
M = ctrb(Ahat,Bhat)
rank(M)
%%
ts = 2;
s1 = -4/ts
delta = [s1, 1.8*s1, 2*s1, 11*s1, 11*s1]
K = place(Ahat,Bhat,delta)
Kest = K(:,1:n)
Ki = -K(:,n+1:end)
%% Modelo lineal en lazo cerrado para seguimiento
AA = [AL - BL*Kest, BL*Ki;
     -CL + DL*Kest, -DL*Ki]
eig(AA)
BB = [zeros(n,r);
      eye(r)];
CC = [CL - DL*Kest, DL*Ki]
DD = zeros(r,r);
sys_new = ss(AA,BB,CC,DD)
figure(1)
step(sys_new,5)
grid

% Acción de control
CC1 = -Kt;
DD1 = zeros(p,r);
sys_u = ss(AA,BB,CC1,DD1)
figure(2)
step(sys_u)
grid
%% Obsevabor de Luenberger polos reales
Vob = obsv(AL,CL)
rank(Vob) % rango del numero de estados
ts = 0.8
s1 = -4/ts
delta = [s1 5*s1 6*s1]
L = place(AL',CL',delta)'
eig(AL-L*CL) 
%% Obsevabor de Luenberger polos complejos
ts = 1
Mp = 0.1
zita = abs(log(Mp))/sqrt(pi^2+(log(Mp))^2)
wn = 4/(ts*zita)
wd = wn*sqrt(1-zita^2)
s1 = -zita*wn + wd*j
delta = [s1 conj(s1) 1.2*real(s1)]
L = (place(AL',CL',delta))'
eig(AL-L*CL) 
%% Observador de Kalman
sys_new = ss (AL,[BL BL],CL,[DL DL])
% Qn = diag([1e-05,1e-05,0.0001,1e-05])   % ruido del actuador
% Rn = diag([1e-05,1e-05]) % ruido del sensor
Qn = diag([0.32^2,0.32^2,0.1^2,0.32^2])   % ruido del actuador
Rn = diag([0.32^2,0.32^2]) % ruido del sensor
[sys_obv,L,S] = kalman(sys_new,Qn,Rn)
eig(sys_obv.a)
%% Control 1 DOF: Controlador & Observador
Ac_1 = [AL-BL*Kest-L*CL+L*DL*Kest, BL*Ki - L*DL*Ki;
        zeros(r,n), zeros(r,r)];
Bc_1 = [-L;
        eye(r)];  
Cc_1 = [-Kest, Ki];    
Dc_1 = zeros(p,r);
sys_c1 = ss(Ac_1,Bc_1,Cc_1,Dc_1)
T1 = feedback(sys*sys_c1,eye(r))
U1 = feedback(sys_c1,sys)
figure(3)
step(T1)
grid
figure(4)
step(U1)
grid
%%
Tm = 0.01
sysd = c2d(sys,Tm)
sys_d1 = c2d(sys_c1,Tm)
T1z = feedback(sysd*sys_d1,eye(r))
U1z = feedback(sys_d1,sysd)
figure(5)
step(T1z)
grid
figure(6)
step(U1z)
grid
%% Control 2 DOF: Controlador & Observador
Ac_2 = [AL-BL*Kest-L*CL+L*DL*Kest, BL*Ki - L*DL*Ki;
        zeros(r,n), zeros(r,r)];
Bc_2 = [zeros(n,r),L;
        eye(r), -eye(r)];  
Cc_2 = [-Kest, Ki];    
Dc_2 = zeros(p,2*r);
sys_c2 = ss(Ac_2,Bc_2,Cc_2,Dc_2)
Bsp = Bc_2(:,1:r);
Bpv = Bc_2(:,r+1:2*r);
ALC = [AL, BL*Cc_2;
       Bpv*CL, Ac_2+Bpv*DL*Cc_2]
BLC = [zeros(n,r);
       Bsp]
CLC = [CL, DL*Cc_2]
DLC = zeros(r,r)
sysLC = ss(ALC,BLC,CLC,DLC)
CuLC = [zeros(p,n), Cc_2]
DuLC = zeros(p,r)
sysuLC = ss(ALC,BLC,CuLC,DuLC)
figure(3)
step(sysLC,5)
grid
figure(4)
step(sysuLC)
grid
%% Discretización del control de dos grados de libertad
Tm = 0.02
sysd = c2d(sys,Tm)
sys_d2 = c2d(sys_c2,Tm)
Gp = sysd.a
Hp = sysd.b
Cp = sysd.c
Dp = sysd.d
Gc = sys_d2.a
Hsp = sys_d2.b(:,1:r)
Hpv = sys_d2.b(:,r+1:end)
Cc = sys_d2.c
GLc = [Gp Hp*Cc;
       Hpv*Cp, Gc+Hpv*Dp*Cc]
HLc = [zeros(n,r);
       Hsp]
CLc = [Cp Dp*Cc]
DLc = zeros(r,r)
CULc = [zeros(p,n), Cc]
DULc = zeros(p,r) 
sysdLc = ss(GLc, HLc, CLc, DLc, Tm)
sysULc = ss(GLc, HLc, CULc, DULc, Tm)
figure(5)
step(sysdLc,5)
grid
figure(6)
step(sysULc)
grid
%%                      DISEÑO FUNCIÓN LQG 1 DOF
Q = diag([1 10 1]) % Penalidad de los estados
QI = diag([2 3]);    % Penalidad de las variables a controlar
R = diag([1 1 1])  % Penalidad de la señal de control
Qn = diag([0.1^2,0.1^2,0.1^2]);  % Varianza de cada estado
Rn = diag([0.01^2,0.01^2]);      % Varianza de la variable de salida
QXU = blkdiag(Q,R)
QWV = blkdiag(Qn,Rn)
reg = lqg(sys,QXU,QWV,QI,'1dof')
T = feedback(sys*reg,eye(r))
U = feedback(reg,sys)
figure(1)
step(T)
grid
figure(2)
step(U)
grid
%% Diseño sistema seguimiento 2 DOF
reg = lqg(sys,QXU,QWV,QI,'2dof')

Ac_2 = reg.a;
Bc_2 = reg.b;
Cc_2 = reg.c;
Dc_2 = reg.d;

Bsp = Bc_2(:,1:r);
Bpv = Bc_2(:,r+1:2*r);
ALC = [AL, BL*Cc_2;
       Bpv*CL, Ac_2+Bpv*DL*Cc_2]
BLC = [zeros(n,r);
       Bsp]
CLC = [CL, DL*Cc_2]
DLC = zeros(r,r)
sysLC = ss(ALC,BLC,CLC,DLC)
CuLC = [zeros(p,n), Cc_2]
DuLC = zeros(p,r)
sysuLC = ss(ALC,BLC,CuLC,DuLC)
figure(3)
step(sysLC)
grid
figure(4)
step(sysuLC)
grid
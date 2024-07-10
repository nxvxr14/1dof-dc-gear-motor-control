%% MODELO v1 ESTADOS[I ThetaL WL]  SALIDA[THETA WL]

clc
clear all
% Variables simbolicas del sistema
syms u1  Vcc R Kv Ki n L m g  Bm Jl Jm Bl x1 x2 x3  d
% Modelo matematico
% Ecuaciones dinámicas

F1 = ((Vcc*u1/100)-Kv*x3*n-R*x1)/(L); %di/dt
F2 = x3; %d0l/dt = Wl
F3 = ((Ki*x1*n)-x3*(Bm*n^2+Bl)-(m*g*d*sin(x2)))/(Jl+Jm*n^2);  %dwl/dt 

%  Matrices (estados, entradas y salidas)
X = [x1;x2;x3];
Xp = [F1;F2;F3];
U = [u1];
Y = [x2];

% Definicion de los Jacobianos
A = jacobian(Xp,X)
B = jacobian(Xp,U)
C = jacobian(Y,X)
D = jacobian(Y,U)

%DATOS SIN ESTIMAR
% Vcc = 7;
% Ro = 3; %[Ohms] Resistencia motor
% Lo = 0.010; %[H] Inductancia motor 
% Kio = 0.004119; %[Nm/A]  Cte par del motor 
% Kvo = 0.004119; %[V/rad s ] Cte contraelectromotriz motor 
% Jmo = 25.83568e-7; %[Kg.m2] Inercia motor 
% Bmo = 5.873e-9; %[Kg/s] Friccion viscosa motor
% no = 120; % Relación caja de engranajes
% Jlo = 0.17244*((0.021)^2) + 0.00000798557; %[Kg.m2] Inercia barra
% Blo = 0.00247; %[Kg/s] Friccion viscosa barra
% mo = 0.17244; %[kg] Masa de la barra 
% do = 0.06862-0.021; %[m] Distancia del eje al centro de gravedad 
% go = 9.81; %[m/s^2] Gravedad 


% DATOS ESTIMADOS 0 PWM
Vcc = 7;
Ro = 5.15610211779038; %ESTIMADA
Lo = 0.667669785204658; %ESTIMADA
Kio = 0.00119530882984502; %ESTIMADA
Kvo = 0.0035694528924319; %ESTIMADA
Jmo = 2.08549279330578e-07;%ESTIMADA
Bmo = 2.02156082362446e-07 %ESTIMADA;
no = 120; % Relación de engranajes
Jlo = 0.00100697016255642; %ESTIMADA
Blo = 8.18575752786119e-06 %ESTIMADA
mo = 0.17244; % Masa de la barra [kg]
do = 0.06862-0.021; % Distancia del eje al centro de gravedad [m]
go = 9.81; % gravedad [m/s^2]

% PUNTOS DE EQUILIBRIO
X10 = 0;
X20 = 0;
X30 = 0;
U10 = 0;
Vin0 = Ro*X10

% Linealización numérica.
A_par = subs(A,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,d,g,x1,x2,x3,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,do,go,X10,X20,X30,U10});
B_par = subs(B,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,g,x1,x2,x3,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,go,X10,X20,X30,U10});
C_par = subs(C,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,g,x1,x2,x3,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,go,X10,X20,X30,U10});
D_par = subs(D,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,g,x1,x2,x3,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,go,X10,X20,X30,U10});

AL = double(subs(A_par))
BL = double(subs(B_par))
CL = double(subs(C_par))
DL = double(subs(D_par))

sys = ss(AL,BL,CL,DL)
eig(AL)
figure(1)
step(sys)
figure(2)
bodemag(sys)



% Analisis de la RGA
s = tf('s')
G = zpk(sys)
G = tf(sys)
Kst = dcgain(G)
%  Kvt = dcgain(s*G)
Kt = [0.024173608876778;
    0]
% RGA = Kt.*(inv(Kt))' % PARA MATRIX SIMETRICA
RGA = Kt.*(pinv(Kst))'
% G11


%% CONTINUA
%% Matriz de controlabilidad
n = 3 % Número de estados
p = 1 % Número de actuadores
r = 1 % Variables a controlar
Ahat = [AL, zeros(n,r);
        -CL, zeros(r,r)]
Bhat = [BL;
        -DL]
Mc = ctrb(Ahat,Bhat)
rank(Mc)  % rango debe ser de n+r

%% Control óptimo
Q = diag([1 1 200 200]) % Matriz diagonal de tamaño n+r
R = diag([0.1])           % Matriz diagonal de tamaño p

[K,S,E] = lqr(Ahat,Bhat,Q,R)
[K1,S,E] = lqi(sys,Q,R)
Kes = K(:,1:n)
Ki = -K(:,n+1:end)
%% Modelo lineal en lazo cerrado para seguimiento
AA = [AL - BL*Kes, BL*Ki;
     -CL + DL*Kes , -DL*Ki]
eig(AA)

BB = [zeros(n,r);
      eye(r)]
CC = [CL - DL*Kes, DL*Ki]
DD = zeros(r,r);
sys_new = ss(AA,BB,CC,DD)
figure(1)
step(sys_new)
grid

% Acción de control
CC1 = -K;
DD1 = zeros(p,r);
sys_u = ss(AA,BB,CC1,DD1)
figure(2)
step(sys_u)
grid

eig(sys_new)


%% Observador de Kalman
sys_new1 = ss (AL,[BL BL],CL,[DL DL])
% Qn = diag([1e-05,1e-05,0.0001,1e-05])   % ruido del actuador
% Rn = diag([1e-05,1e-05]) % ruido del sensor
Qn = diag([1e-6^2])   % ruido del actuador
Rn = diag([1e-6^2]) % ruido del sensor
[sys_obv,L,S] = kalman(sys_new1,Qn,Rn)
eig(sys_obv.a)


%% Control 2 DOF: Controlador & Observador (Unificación)
Ac_2 = [AL-BL*Kes-L*CL+L*DL*Kes, BL*Ki - L*DL*Ki;
        zeros(r,n), zeros(r,r)];
Bc_2 = [zeros(n,r),L;
        eye(r), -eye(r)];  
Cc_2 = [-Kes, Ki];    
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
step(sysLC)
grid
figure(4)
step(sysuLC)
grid


%% Discretización del control de dos grados de libertad
Tm = 0.03
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
step(sysdLc)
grid
figure(6)
step(sysULc)
grid

%% DISCRETA
%% DISCRETIZAR PLANTA
Tm = 0.03
fc = (2*pi/Tm)/2
sysd = c2d(sys,Tm)
GL = sysd.a
HL = sysd.b
figure(1)
bodemag(sys,sysd)
grid
figure(2)
step(sys,sysd,4)
grid
eig(GL)
%% Matriz de controlabilidad
n = 3 % Numero de estados
p = 1 % N�mero de actuadores
r = 1 % N�mero de salidas
Ghat = [GL zeros(n,r);
        -CL*GL eye(r)]
Hhat = [HL;
       -CL*HL]
Mcon = ctrb(Ghat,Hhat)
rank(Mcon)
%% Control por ubicaci�n de polos reales
ts = 4;
muestras = ts/Tm
s1 = -4/ts;
delta = [s1 6*s1 7*s1 8*s1]
pd = exp(Tm*delta)
Kt = place(Ghat,Hhat,pd)
Kest = Kt(:,1:n)
Ki = -Kt(:,n+1:end)

%% Modelo lineal en lazo cerrado
AA = [GL - HL*Kest, HL*Ki;
     -CL*GL + CL*HL*Kest , eye(r) - CL*HL*Ki]
eig(AA)
BB = [zeros(n,r);
      eye(r)]
CC = [CL, zeros(r,r)]
DD = zeros(r,r);
sys_new = ss(AA,BB,CC,DD,Tm)
CU = [-Kest Ki]
DU = zeros(p,r)
sysU = ss(AA,BB,CU,DU,Tm)
figure(1)
step(sys_new)
grid
figure(2)
step(sysU)
grid

%% Obsevabor de Luenberger - Polos reales
Vob = obsv(GL,CL)
rank(Vob)
ts = 2
muestras = ts/Tm  % M�nimo 8 muestras
s1 = -4/ts
pol_obv_c = [s1 1.2*s1 5*s1]
pol_obv_d = exp(pol_obv_c*Tm)
L = place(GL',CL',pol_obv_d)'
eig(GL-L*CL)

%% Observador de Kalman
sys_new = ss (GL,[HL HL],CL,[DL DL],Tm)
Qn = diag([1e-6^2])   % ruido del actuador
Rn = diag([1e-6^2]) % ruido del sensor
[sys_obv,L,S] = kalman(sys_new,Qn,Rn)
eig(sys_obv.a)

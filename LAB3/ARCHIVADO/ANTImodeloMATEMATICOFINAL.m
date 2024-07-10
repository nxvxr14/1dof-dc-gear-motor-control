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
Y = [x2,x3];

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



%% Matriz de controlabilidad
n = 3 % Número de estados
p = 1 % Número de actuadores
r = 2 % Variables a controlar
Ahat = [AL, zeros(n,r);
        -CL, zeros(r,r)]
Bhat = [BL;
        -DL]
Mc = ctrb(Ahat,Bhat)
rank(Mc)  % rango debe ser de n+r

%% Control óptimo
Q = diag([0.5 5 17]) % Matriz diagonal de tamaño n+r
R = diag([0.1])           % Matriz diagonal de tamaño p
[K,S,E] = lqr(Ahat,Bhat,Q,R)
[K1,S,E] = lqi(sys,Q,R)
Kes = K(:,1:n)
Ki = -K(:,n+1:end)
%% Modelo lineal en lazo cerrado para seguimiento
AA = [AL - BL*Kes, BL*Ki;
     -CL + DL*Kes , -DL*Ki]
eig(AA)
% AA = [Ac - Bc*Kes, Bc*Ki;
%      -CL + DL*Kes , -DL*Ki]
% eig(AA.NominalValue)
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

%% Discretización del controlador
Tm = 0.03
Kid = Ki*Tm;
sysd = c2d(sys,Tm);
GL = sysd.a;
HL = sysd.b;
AA = [GL - HL*Kes, HL*Kid;
     -CL*GL + CL*HL*Kes , eye(r) - CL*HL*Kid]
eig(AA)
BB = [zeros(n,r);
      eye(r)]
CC = [CL, zeros(r,r)]
DD = zeros(r,r);
sys_newd = ss(AA,BB,CC,DD,Tm)
CU = [-Kes Kid]
DU = zeros(p,r)
sysUd = ss(AA,BB,CU,DU,Tm)
figure(3)
step(sys_newd,sys_new)
grid
figure(4)
step(sysUd,sys_u)
grid


%% MODELO v1 ESTADOS[I ThetaL WL]  SALIDA[WL]

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



%% Analisis de la RGA
s = tf('s')
G = zpk(sys)
G = tf(sys)
Kst = dcgain(G)
%  Kvt = dcgain(s*G)
Kt = [0.024173608876778;
    0]
% RGA = Kt.*(inv(Kt))' % PARA MATRIX SIMETRICA
RGA = Kt.*(pinv(Kst))'

%% CONTROLADOR 1DOF PIDs
G11 = G(1,1)

Cpids = (58.086*(s^2 + 2.637*s + 24.29))/(s*(s+9.924)) % PID con seudo
Tm = 30e-3; %tiempo muestreo
z = tf('z',Tm)
G1z = c2d(G11,Tm,'zoh')
Cz = c2d(Cpids,Tm,'matched')

format long
[nz,dz] = tfdata(Cz,'v')

% PARA CONFIRMAR LOS ARGUMENTOS DE LA MATRIZ
% syms Kp Ki Kd N z
% Cpids = Kp + Ki*z/(z-1) + Kd*(z-1)/(z-N);
% collect(Cpids)

K1 = 52.346730287862997;
K2 = 99.613623826288730;
K3 = 48.365148241217845;
N = 0.742509213223832;
A = [1, 1, 1; 
     N+1, N, 2; 
     N, 0, 1];

B = [K1; K2; K3];
X = linsolve(A, B);
disp(X);

% 2 SEGUNDOS
Kp = -1.101543753570772;
%Ki = 4.265219414420463;
Ki = 2.065219414420463
%Kd = 49.183054627013306;
Kd = 30.183054627013306;
N = 0.742509213223832;
 
Cz1 = Kp + Ki*z/(z-1) + Kd*(z-1)/(z-N)


Tz = feedback(Cpids*G11,1);
Uz = feedback(Cpids,G11);
figure(1)
subplot(2,1,1)
step(Tz)
grid
subplot(2,1,2)
step(Uz)
grid
figure(3)
step(sys)

%% CONTROLADOR 2DOF PIDs-PDs
G11 = G(1,1)
% 
C1s = (35.291*(s^2 + 3.286*s + 29.58))/(s*(s+8.192)) %PIDseudo
C2s = (3.6678*(s+0.5667))/(s+0.09451) %PDseudo

z = tf('z',Tm);
Tm = 30e-3; %tiempo muestreo
G1z = c2d(G11,Tm,'zoh')
C1z = c2d(C1s,Tm,'matched')
C2z = c2d(C2s,Tm,'matched')

format long
[nz,dz] = tfdata(C1z,'v')
[nz,dz] = tfdata(C2z,'v')

% C1Z PID SEUDO
K1 = 32.929841189993510;
K2 = 61.935351065558599;
K3 = 29.838493069912801;
N = 0.782109908780621;
A = [1, 1, 1; 
     N+1, N, 2; 
     N, 0, 1];

B = [K1; K2; K3];
X = linsolve(A, B);
disp(X);

Kp = -3.357670445385118;
Ki = 3.822951239710286;
Kd = 32.464560395668343;
N = 0.782109908780621;
 
Cz1 = Kp + Ki*z/(z-1) + Kd*(z-1)/(z-N)

% C2Z PD SEUDO
K1 = 3.693827447702085;
K2 = 3.631559495778081;
N1 = 0.997168715666942;
A = [1, 1; 
     N, 1];
B = [K1; K2];
X = linsolve(A, B);
disp(X);

Kpd = 0.285776886757601;
Kdp = 3.408050560944484;
N1 = 0.997168715666942;
 
Cz1 = Kp + Ki*z/(z-1) + Kd*(z-1)/(z-N)
Cz2 = Kpd + Kdp*(z-1)/(z-N1)

Tz = feedback(Cz1*G1z,1+Cz2/Cz1);
Uz = feedback(Cz1,G1z*(1+Cz2/Cz1));
figure(1)
subplot(2,1,1)
step(Tz)
grid
subplot(2,1,2)
step(Uz)
grid

%% CONTROLADOR CASCADA 

G11 = G(1,1) %Posicion 
G21 = G(2,1) %Velocidad

C21 =  1.5978*(s^2 + 3.612*s + 29.65)/(s*(s+0.001))
C11 = (22*(s^2 + 4.354*s + 4.779))/( s*(s+3.303))
T1 = feedback(C11*G11,1)
Gnew = T1*G21
T = feedback(C21*Gnew,1)
figure(1)
step(T)
grid


Tm = 0.03;
G11z = c2d(G11,Tm,'zoh');
G21z = c2d(G21,Tm,'zoh');
C21z = c2d(C21,Tm,'matched')
C11z = c2d(C11,Tm,'matched')
format long
[nz,dz] = tfdata(C21z,'v')
[nz,dz] = tfdata(C11z,'v')
T21z = feedback(C21z*G21z,1);
figure(1)
step(T21z,T21)
grid
figure(2)
Uz1 = feedback(C21z*G21z);
step(Uz1)
grid

Gnewz = T21z*G21z
Tz = feedback(C11z*Gnewz,1)
figure(3)
step(T,Tz)
grid
figure(4)
Uz = feedback(C11z,Gnewz);
step(Uz)
grid


%% MODELO v2 ESTADOS[ThetaL WL]  SALIDA[WL]

clc
clear all

% Variables simbolicas del sistema
syms u1 Vcc R Kv Ki n L m g Bm Jl Jm Bl x1 x2 d

% Ecuaciones dinámicas
F1 = x2; %d0l/dt = Wl
F2 = ((Ki*(((Vcc*u1/100)-(Kv*x2*n)/R))*n)-x2*(Bm*n^2+Bl)-(m*g*d*sin(x1)))/(Jl+Jm*n^2);  %dwl/dt

%  Matrices (estados, entradas y salidas)
X = [x1;x2];
Xp = [F1;F2];
U = [u1];
Y = [x1];

% Definicion de los Jacobianos
A = jacobian(Xp,X)
B = jacobian(Xp,U)
C = jacobian(Y,X)
D = jacobian(Y,U)

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
U10 = 0;
Vin0 = Ro*X10

% Linealización numérica.
A_par = subs(A,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,d,g,x1,x2,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,do,go,X10,X20,U10});
B_par = subs(B,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,d,g,x1,x2,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,do,go,X10,X20,U10});
C_par = subs(C,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,d,g,x1,x2,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,do,go,X10,X20,U10});
D_par = subs(D,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,d,g,x1,x2,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,do,go,X10,X20,U10});
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

%% Matriz de controlabilidad
n = 2 % Número de estados
p = 1 % Número de actuadores
r = 1 % Variables a controlar
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


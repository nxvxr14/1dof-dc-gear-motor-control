%% MODELO 2 ESTADOS I TL WL  SALIDA TL WL

clc
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


% % DATOS ESTIMADOS 0 PWM
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
AL = double(subs(A_par))
BL = double(subs(B_par))
CL = [0 1 0;
    0 0 1]
DL = [0;
    0]

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
Kt = [0.198768705461364;
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
Ki = 4.265219414420463;
Kd = 49.183054627013306;
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

%% CONTROLADOR CASCADA 1DOF
G11 = G(1,1)
G21 = G(2,1)
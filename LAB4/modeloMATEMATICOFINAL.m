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

n = 3 % Número de estados
p = 1 % Número de actuadores
r = 1 % Variables a controlar


%% DISCRETA
Tm = 0.03
sysd = c2d(sys,Tm)
GL = sysd.a
HL = sysd.b
figure(3)
step(sys,sysd)




%% MIXSYN DISCRETO
s = tf('s')
[n,d] = butter(1,0.01,'low','s')
W1 = 0.05*tf(n,d)
W1z = c2d(W1,Tm,'matched');
[n,d] = butter(2,0.8,'high','s')
W2 = 0.00001*tf(n,d)
W2z = c2d(W2,Tm,'matched');
[n,d] = butter(2,7,'high','s')
W3 = 0.005*tf(n,d)
W3z = c2d(W3,Tm,'matched');
figure(4)
subplot(3,1,1)
bodemag(W1,W1z)
grid
subplot(3,1,2)
bodemag(W2,W2z)
grid
subplot(3,1,3)
bodemag(W3,W3z)
grid
%%
[K,CLazo,gamma,info] = mixsyn(sysd,W1z,W2z,W3z);
T = feedback(sysd*K,eye(r));
U = feedback(K,sysd);
S = feedback(eye(r),sysd*K);
figure(5)
subplot(3,1,1)
sigma(S,'b',gamma/W1,'b-.')
grid
subplot(3,1,2)
sigma(U,'r',gamma/W2,'r-.')
grid
subplot(3,1,3)
sigma(T,'g',gamma/W3,'g-.')
grid

figure(6)
step(T)
grid
figure(7)
step(U)
grid

%%
Kred = reduce(K,4);
%Kdig = c2d(Kred,Tm);
T1 = feedback(sysd*Kred,eye(r));
U1 = feedback(Kred,sysd);
S1 = feedback(eye(r),sysd*Kred);

figure(8)
step(T,T1)
grid
figure(9)
step(U,U1)
grid




%% LOOPSING DISCRETO 1DOF
s = tf('s')
Gd = 1/s %1DOF
Gdz = c2d(Gd, Tm, 'matched')

%%
[K,CLazo,GAM,info] = loopsyn(sysd,Gdz)
figure(2)
sigma(sysd*K,'r',Gdz*GAM,'k-.',Gdz/GAM,'k-.')  % plot result
grid
legend('G*K','Gd*GAM','Gd/GAM')
T = feedback(sysd*K,eye(r));
U = feedback(K,sysd);
figure(3)
step(T)
grid
figure(4)
step(U)
grid
%%
Kred = reduce(K,3)
T1 = feedback(sysd*Kred,eye(r));
U1 = feedback(Kred,sysd);
S1 = feedback(eye(r),sysd*Kred);

figure(5)
step(T,T1)
grid
figure(6)
step(U,U1)
grid
%% VER INTEGRADORES
eig(Kred.a)
sysI = ss(Kred.a,Kred.b,eye(3),zeros(3,1))
zpk(sysI)





%% LOOPSING DISCRETO 2DOF

K2dof = info.K2dof
K2dof.InputName = {'r1','y1'};
K2dof.OutputName = {'u1'};
sysd.InputName = {'u1'};
sysd.OutputName = {'y1'};
inputs = {'r1'};
outputs = {'y1'};
T2dof = connect(K2dof,sysd,inputs,outputs);
figure(3)
step(T,T2dof)
grid
inputs = {'r1'};
outputs = {'u1'};
U2dof = connect(K2dof,sysd,inputs,outputs);
figure(4)
step(U,U2dof)
grid

%%
Kred2dof = reduce(K2dof,4)
Kred2dof.InputName = {'r1','y1'};
Kred2dof.OutputName = {'u1'};
sysd.InputName = {'u1'};
sysd.OutputName = {'y1'};
inputs = {'r1'};
outputs = {'y1'};
T2dof_r = connect(Kred2dof,sysd,inputs,outputs);
figure(5)
step(T2dof,T2dof_r)
grid
inputs = {'r1'};
outputs = {'u1'};
U2dof_r = connect(Kred2dof,sysd,inputs,outputs);
figure(6)
step(U2dof,U2dof_r)
grid
%% VER INTEGRADORES
eig(Kred2dof.a)
sysI2 = ss(Kred2dof.a,Kred2dof.b,eye(4),zeros(4,1))
zpk(sysI2)





%% NCFSYN DISCRETO
s = tf('s')
Gd = 40/s %1DOF
Gdz = c2d(Gd, Tm, 'matched')
%%
[Kpos,CLazo,GAM,info] =  ncfsyn(sysd,Gdz)
%[Kpos,CLazo,GAM,info] =  ncfsyn(sys,W1,W2)
figure(2)
sigma(sysd*Kpos,sysd*Gdz)
%sigma(sys*Kncf,'r',W2*sys*W1,'b')  % plot result
legend('achieved','target')
grid
K = -Kpos;
T = feedback(sysd*K,eye(r));
U = feedback(K,sysd);
figure(3)
step(T)
grid
figure(4)
step(U)
grid
%%
Kred = reduce(K,3)
T1 = feedback(sysd*Kred,eye(r));
U1 = feedback(Kred,sysd);
S1 = feedback(eye(r),sysd*Kred);

figure(5)
step(T,T1)
grid
figure(6)
step(U,U1)
grid

%% VER INTEGRADORES
eig(Kred.a)
sysI = ss(Kred.a,Kred.b,eye(3),zeros(3,1))
zpk(sysI)


%% DATA
t = NFC(:,1)/1000;
t = (t'-t(1));
SP = NFC(:,2)*0.0174533;
theta = NFC(:,3)*0.0174533;
pwm = NFC(:,4);

%PWM IDENTIFIACION
figure(1)
subplot(2,1,1);
plot(t,theta);
hold on;
plot(t,SP);
title("POSICION");
xlabel('Tiempo [s]') 
ylabel('Angulo [rad]') 
subplot(2,1,2);
plot(t,pwm);
title("PWM");
xlabel('Tiempo [s]') 
ylabel('PWM [%]') 


figure(2)
histogram(diff(t))
title("PERIODO DE MUESTREO");
xlabel('Tiempo [s]') 
ylabel('Muestras [#]') 





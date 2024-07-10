
syms x1 x2 x3 Vin    % x1 = corriente, x2 = wl, x3 = thetal
syms Jm Jl Bm Bl Ki Kv R L n m g d
Jt = Jm*n^2 +Jl
f1 = (1/L)*(Vin - R*x1 - Kv*x2*n)
f2 = (1/Jt)*(Ki*x1*n - (Bm*n^2 + Bl)*x2 - m*g*d*sin(x3))
f3 = x2

F = [f1; f2; f3]
X = [x1; x2; x3]
A = jacobian(F,X)
B = jacobian(F,Vin)

% thetaL0 = 0
% x30 = n*thetaL0
% x10 = (m*g*d*sin(x30/n))/ki
% Vin0 = R*x10

%PARAMETROS INICIALES
Vin = 7; %Volts
R0 = 3.9654; % [ohms]
L0= 0.51e-3;
Kv0 = 0.03212e-6;
Ki0 = 0.00712; % Constante electromotriz [V/rad/s] = constante de torque [V/rad/s]
n0 = 120; % Relación de engranajes
Jm0 = 7.83067e-12; % Momento de inercia del motor
Bm0 = 5.862e-9; % Fricción viscosa del motor
Bl0 = 0.347; % Fricción viscosa de la barra
Jl0 = 0.04364; % Momento de inercia de la barra
m0 = 0.17244; % Masa de la barra [kg]
d0 = 0.05862; % Distancia del eje al centro de gravedad [m]
g0 = 9.81; % gravedad [m/s^2]
thetaL0 = 0;

x30 = thetaL0
x10 = (m0*g0*d0*sin(x30))/Ki0
x20 = 0
Vin0 = R0*x10

AL = double(subs(A,{x1,x2,x3,Vin,R,L,Ki,Kv,Jm,Jl,Bm,Bl,n,g,d,m},{x10,x20,x30,Vin0,R0,L0,Ki0,Kv0,Jm0,Jl0,Bm0,Bl0,n0,g0,d0,m0}))
BL = double(subs(B,{x1,x2,x3,Vin,R,L,Ki,Kv,Jm,Jl,Bm,Bl,n,g,d,m},{x10,x20,x30,Vin0,R0,L0,Ki0,Kv0,Jm0,Jl0,Bm0,Bl0,n0,g0,d0,m0}))
CL = [0 1 0;
    0 0 1]
DL = [0;
    0]
sys = ss(AL,BL,CL,DL)
eig(AL)
step(sys)
bodemag(sys)

%% MODELO 2 ESTADOS I TL WL  SALIDA TL WL

syms u1  Vcc R Kv Ki n L m g  Bm Jl Jm Bl x1 x2 x3  d


F1 = (Vin-Kv*x3*n-R*x1)/(L); %di/dt
F2 = x3; %d0l/dt = Wl
F3 = ((Ki*x1*n)-x3*(Bm*n^2+Bl)-(m*g*d*sin(x2)))/(Jl+Jm*n^2);  %dwl/dt 

X = [x1;x2;x3];
nx = length(X);

Xp = [F1;F2;F3];

U = [u1];
nu = length(U);

Y = [x2;x3];
ny = length(Y);

% Definicion de los Jacobianos
A = jacobian(Xp,X)
B = jacobian(Xp,U)
C = jacobian(Y,X)
D = jacobian(Y,U)

% PUNTOS DE EQUILIBRIO

x30 = thetaL0
x10 = (m0*g0*d0*sin(x30))/Ki0
x20 = 0
Vin0 = R0*x10

R0 = 3.4;
L0 = 0.0013;
Ki0 = 0.005119;
Kv0 = 0.005119;
Jm0 = 7.83568e-12;
Bm0 = 5.873e-9;
Bl0 = 0.05;

n0 = 120; % Relación de engranajes
Jl0 = 0.00000220756; % Momento de inercia de la barra
Bl0 = 0.0647; % Fricción viscosa de la barra
m0 = 0.17244; % Masa de la barra [kg]
d0 = 0.065; % Distancia del eje al centro de gravedad [m]
g0 = 9.81; % gravedad [m/s^2]
thetaL0 = 0;
% Linealización numérica.

A_par = subs(A,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,d,g,x1,x2,x3,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,do,go,X10,X20,X30,U10});
B_par = subs(B,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,g,x1,x2,x3,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,go,X10,X20,X30,U10});
C_par = subs(C,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,g,x1,x2,x3,u1},{Ro,Lo,Kvo,Kio,no,Jmo,Jlo,Bmo,Blo,mo,go,X10,X20,X30,U10});

AL = double(subs(A_par))
BL = double(subs(B_par))
CL = double(subs(C_par))
DL = double(subs(D))
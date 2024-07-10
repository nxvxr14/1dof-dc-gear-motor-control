%% MODELO 2 ESTADOS I TL WL  SALIDA TL WL

syms u1 Vin R Kv Ki n L m g  Bm Jl Jm Bl x1 x2 x3 d


F1 = (Vin-Kv*x3*n-R*x1)/(L); %di/dt
F2 = x3; %d0l/dt = Wl
F3 = ((Ki*x1*n)-x3*(Bm*n^2+Bl)-(m*g*d*sin(x2)))/(Jl+Jm*n^2);  %dwl/dt 

F = [F1;F2;F3];
X = [x1;x2;x3];
Y = [x2;x3];

%DEFINICION DE LOS JACOBIANOS
A = jacobian(F,X)
B = jacobian(F,Vin)

% DATOS SIN ESTIMAR
R0 = 3; %[Ohms] Resistencia motor
L0 = 0.010; %[H] Inductancia motor 
Ki0 = 0.004119; %[Nm/A]  Cte par del motor 
Kv0 = 0.004119; %[V/rad s ] Cte contraelectromotriz motor 
Jm0 = 25.83568e-7; %[Kg.m2] Inercia motor 
Bm0 = 5.873e-9; %[Kg/s] Friccion viscosa motor
n0 = 120; % Relación caja de engranajes
Jl0 = 0.17244*((0.021)^2) + 0.00000798557; %[Kg.m2] Inercia barra
Bl0 = 0.00247; %[Kg/s] Friccion viscosa barra
m0 = 0.17244; %[kg] Masa de la barra 
d0 = 0.06862-0.021; %[m] Distancia del eje al centro de gravedad 
g0 = 9.81; %[m/s^2] Gravedad 


% DATOS ESTIMADOS
% R0 = 12; %ESTIMADA
% L0 = 0.171319600851342; %ESTIMADA
% Ki0 = 0.00160119572488985; %ESTIMADA
% Kv0 = 0.00313545752318492; %ESTIMADA
% Jm0 = 2.13190246911322e-07;%ESTIMADA
% Bm0 = 1.07240481196771e-08 %ESTIMADA;
% n0 = 120; % Relación de engranajes
% Jl0 = 0.000244979327240305; %ESTIMADA
% Bl0 = 1.33884099995416e-05 %ESTIMADA
% m0 = 0.17244; % Masa de la barra [kg]
% d0 = 0.06862-0.021; % Distancia del eje al centro de gravedad [m]
% g0 = 9.81; % gravedad [m/s^2]


%PUNTOS DE EQUILIBRIO
thetaL0 = 0;
x20 = thetaL0
x10 = (m0*g0*d0*sin(x20))/Ki0
x30 = 0
u10 = 0
Vin0 = R0*x10

%LINEALIZACION NUMERICA
AL = double(subs(A,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,d,g,x1,x2,x3,Vin},{R0,L0,Kv0,Ki0,n0,Jm0,Jl0,Bm0,Bl0,m0,d0,g0,x10,x20,x30,Vin0}));
BL = double(subs(B,{R,L,Kv,Ki,n,Jm,Jl,Bm,Bl,m,d,g,x1,x2,x3,Vin},{R0,L0,Kv0,Ki0,n0,Jm0,Jl0,Bm0,Bl0,m0,d0,g0,x10,x20,x30,Vin0}));
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

w3db = 7.58;
ws = 10*(2*w3db);
Tm = 2*pi/ws

%% INCERTIDUMBRES
% Modelo de incertidumbre
Tol = 10;
Lc = ureal('Lc',L0,'Percentage',[-Tol Tol]); 
Rc = ureal('Rc',R0,'Percentage',[-Tol Tol]); 
Jmc = ureal('Jmc',Jm0,'Percentage',[-Tol Tol]);
Kic = ureal('Kic',Ki0,'Percentage',[-Tol Tol]);
Kvc = ureal('Kvc',Kv0,'Percentage',[-Tol Tol]);
Bmc = ureal('Bmc',Bm0,'Percentage',[-Tol Tol]);
Jlc = ureal('Jlc',Jl0,'Percentage',[-Tol Tol]);
Blc = ureal('Blc',Bl0,'Percentage',[-Tol Tol]);
mc = ureal('mc',m0,'Percentage',[-Tol Tol]);



%ECUACIONES CAMBIAN DEPENDIENDO R L JL Jm
Ac = [-R0/L0,0 ,-(Kv0*n0)/L0;
    0, 0, 1;
    (Ki0*n0)/(Jm0*n0^2 + Jl0), -(d0*g0*mc*cos(x20))/(Jm0*n0^2 + Jl0), -(Bm0*n0^2 + Blc)/(Jm0*n0^2 + Jl0)]
Bc = [1/L0;
     0;
     0]

% Ac = [-Rc/Lc,0 ,-(Kvc*n0)/Lc;
%     0, 0, 1;
%     (Kic*n0)/(Jmc*n0^2 + Jlc), -(d0*g0*m0*cos(x20))/(Jmc*n0^2 + Jlc), -(Bmc*n0^2 + Blc)/(Jmc*n0^2 + Jlc)]
% Bc = [1/Lc;
%      0;
%      0]

%GRAFICAMOS
sysc = ss(Ac,Bc,CL,DL) 
figure(1)
bodemag(sysc)
grid
figure(2)
step(sysc)
grid

%% Analisis de la RGA
s = tf('s')
G = zpk(sys)
G = tf(sys)
Kst = dcgain(G)
Kt = [0.198768705461364;
    0]
% RGA = Kt.*(inv(Kt))' % PARA MATRIX SIMETRICA
RGA = Kt.*(pinv(Kst))'

%% Controladores
G11 = G(1,1)
Cpids = (0.12625*(s^2 + 7.555*s + 31.85))/( s*(s+1.329))

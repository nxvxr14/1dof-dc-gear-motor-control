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
%%
Tm = 0.05
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
figure(3)
impulse(sys,sysd)
grid
eig(GL)
%% Matriz de controlabilidad
n = 3 % Numero de estados
p = 3 % Número de actuadores
r = 2 % Número de salidas
Mcon = ctrb(GL,HL)
rank(Mcon)
%% Control óptimo
Q = diag([1 1 1]);
R = diag([10 10 10]);
[K,S,E] = dlqr(GL,HL,Q,R)
%% Modelo lineal en lazo cerrado para punto de equilibrio
xini = [0.2; -0.3; -0.4]
AA = GL-HL*K
eig(AA)
BB = zeros(n,p)
CC = CL
DD = zeros(r,p);
sys_new = ss(AA,BB,CC,DD,Tm)
figure(1)
initial(sys_new,xini)
grid

% Acción de control
CC1 = -K;
DD1 = zeros(p,p);
sys_u = ss(AA,BB,CC1,DD1,Tm)
figure(2)
initial(sys_u,xini)
grid

%% Obsevabor de Luenberger
Vob = obsv(GL,CL)
rank(Vob)
ts = 0.8
muestras = ts/Tm  % Mínimo 8 muestras
s1 = -4/ts
pol_obv_c = [s1 1.2*s1 2.5*s1]
pol_obv_d = exp(pol_obv_c*Tm)
L = place(GL',CL',pol_obv_d)'
eig(GL-L*CL)
%% Observador de Kalman
sys_new = ss (GL,[HL HL],CL,[DL DL],Tm)
Qn = diag([0.01^2 0.01^2 0.01^2])   % ruido del actuador
Rn = diag([0.01^2 0.01^2]) % ruido del sensor
[sys_obv,L,S] = kalman(sys_new,Qn,Rn)
eig(sys_obv.a)
%% Controlador y observador discretos unificados
Gc = GL-HL*K-L*CL+L*DL*K;
Hc = L;
Cc = -K;
Dc = zeros(p,r)

GLc = [GL, HL*Cc;
       Hc*CL, Gc+Hc*DL*Cc]
HLc = zeros(2*n,1)
CLc = [CL DL*Cc]
DLc = zeros(r,1)

CuLc = [zeros(p,n) Cc]
DuLc = zeros(p,1)
xininew = [xini; zeros(n,1)]
sysd_Lc = ss(GLc,HLc,CLc,DLc,Tm)
figure(3)
initial(sysd_Lc,xininew)
grid

sysdu_Lc = ss(GLc,HLc,CuLc,DuLc,Tm)
figure(4)
initial(sysdu_Lc,xininew)
grid

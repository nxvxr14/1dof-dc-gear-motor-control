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
M = ctrb(AL,BL)
rank(M)
%%
Q = diag([1 1 1]);
R = diag([1 1 1]);
[Kest,S,E] = lqr(AL,BL,Q,R)
%% Modelo lineal en lazo cerrado para punto de equilibrio
xini = [0.2; -0.3; -0.4]
Anew = AL-BL*Kest
eig(Anew)
Bnew = zeros(n,p)
Cnew = CL
Dnew = DL
sys_new = ss(Anew,Bnew,Cnew,Dnew)
figure(1)
initial(sys_new,xini)
grid
Cu = -Kest
Du = zeros(p,p)
sys_u = ss(Anew,Bnew,Cu,Du)
figure(2)
initial(sys_u,xini)
grid
%% Obsevabor de Luenberger
Vob = obsv(AL,CL)
rank(Vob) % rango del numero de estados
ts = 1
s1 = -4/ts
delta = [s1 1.2*s1 2*s1]
L = place(AL',CL',delta)'
eig(AL-L*CL) 
%% Obsevabor de Luenberger polos complejos
ts = 1.5
Mp = 0.1
zita = abs(log(Mp))/sqrt(pi^2+(log(Mp))^2)
wn = 4/(ts*zita)
wd = wn*sqrt(1-zita^2)
s1 = -zita*wn + wd*j
delta = [s1 conj(s1) 1.1*real(s1)]
L = (place(AL',CL',delta))'
eig(AL-L*CL) 
%% Observador de Kalman
sys_new = ss (AL,[BL BL],CL,[DL DL])
Qn = diag([1e-4,1e-5,1e-5])   % ruido del actuador  varianza = std^2
Rn = diag([1e-5,1e-5]) % ruido del sensor
[sys_obv,L,S] = kalman(sys_new,Qn,Rn)
eig(sys_obv.a)
%% Control unificado: controlador & Observador
Ac = AL - L*CL - (BL-L*DL)*Kest
Bc = L
Cc = -Kest
Dc = zeros(p,r)
sys_c = ss(Ac,Bc,Cc,Dc)
ALc = [AL, BL*Cc;
       Bc*CL, Ac+Bc*DL*Cc]
BLc = zeros(2*n,1)
CLc = [CL DL*Cc]
DLc = zeros(r,1)

CuLc = [zeros(p,n) Cc];
DuLc = zeros(p,1)
xininew = [xini; zeros(n,1)]
sys_new_Lc = ss(ALc,BLc,CLc,DLc)
figure(3)
initial(sys_new_Lc,xininew)
grid

sys_u_Lc = ss(ALc,BLc,CuLc,DuLc)
figure(4)
initial(sys_u_Lc,xininew)
grid
%%
Tm = 0.02;
sys_c_dig = c2d(sys_c,Tm)
Gc = sys_c_dig.a
Hc = sys_c_dig.b
Cc = sys_c_dig.c
Dc = sys_c_dig.d
sys_d = c2d(sys,Tm)
Gp = sys_d.a
Hp = sys_d.b
Cp = sys_d.c
Dp = sys_d.d
GLc = [Gp, Hp*Cc;
       Hc*Cp, Gc+Hc*Dp*Cc]
HLc = zeros(2*n,1)
CLc = [Cp Dp*Cc]
DLc = zeros(r,1)

CuLc = [zeros(p,n) Cc]
DuLc = zeros(p,1)
xininew = [xini; zeros(n,1)]
sysd_Lc = ss(GLc,HLc,CLc,DLc,Tm)
figure(5)
initial(sysd_Lc,xininew)
grid

sysdu_Lc = ss(GLc,HLc,CuLc,DuLc,Tm)
figure(6)
initial(sysdu_Lc,xininew)
grid
%%          DISEÑO FUNCIÓN LQG
n = 4 % Numero de estados
p = 4 % Número de actuadores
r = 2 % Número de salidas
M = ctrb(AL,BL)
rank(M)
Vob = obsv(AL,CL)
rank(Vob)

Q = diag([1 0.5 2 1]) % Penalidad de los estados
QI = diag([0.1 0.3]);         % Penalidad de las variables a controlar
Qn = diag([0.1^2,0.1^2,0.1^2,0.1^2]);  % Varianza de cada estado
Rn = diag([0.01^2,0.01^2]);                % Varianza de la variable de salida
QXU = blkdiag(Q,R)
QWV = blkdiag(Qn,Rn)
reg = lqg(sys,QXU,QWV)

Ac = reg.a
Bc = reg.b
Cc = reg.c
Dc = reg.d

ALc = [AL, BL*Cc;
       Bc*CL, Ac+Bc*DL*Cc]
BLc = zeros(2*n,1)
CLc = [CL DL*Cc]
DLc = zeros(r,1)

CuLc = [zeros(p,n) Cc];
DuLc = zeros(p,1)
xininew = [xini; zeros(n,1)]
sys_new_Lc = ss(ALc,BLc,CLc,DLc)
figure(3)
initial(sys_new_Lc,xininew)
grid

sys_u_Lc = ss(ALc,BLc,CuLc,DuLc)
figure(4)
initial(sys_u_Lc,xininew)
grid
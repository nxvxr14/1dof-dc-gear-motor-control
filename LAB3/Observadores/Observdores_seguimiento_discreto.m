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
Ghat = [GL zeros(n,r);
        -CL*GL eye(r)]
Hhat = [HL;
       -CL*HL]
Mcon = ctrb(Ghat,Hhat)
rank(Mcon)
%% Control por ubicación de polos reales
ts = 2;
muestras = ts/Tm
s1 = -4/ts;
delta = [s1 1.5*s1 2.3*s1 5*s1 10*s1]
pd = exp(Tm*delta)
Kt = place(Ghat,Hhat,pd)
Kest = Kt(:,1:n)
Ki = -Kt(:,n+1:end)
%% Control por ubicación de polos complejos
ts = 2;
muestras = ts/Tm
Mp = 0.1;
zita = abs(log(Mp))/sqrt(pi^2+(log(Mp))^2)
wn = 4/(ts*zita)
wd = wn*sqrt(1-zita^2)
s1 = -zita*wn + wd*j
delta = [s1 conj(s1) 1.2*real(s1) 1.5*real(s1) 6*real(s1)]
pd = exp(Tm*delta)
Kt = place(Ghat,Hhat,pd)
Kest = Kt(:,1:n)
Ki = -Kt(:,n+1:end)
%% Control óptimo
Q = diag([1 1 1 0.5 0.1]);
R = diag([1 1 1]);
[K,S,E] = dlqr(Ghat,Hhat,Q,R)
Kest = K(:,1:n)
Ki = -K(:,n+1:end)
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
ts = 0.5
muestras = ts/Tm  % Mínimo 8 muestras
s1 = -4/ts
pol_obv_c = [s1 2*s1 4*s1]
pol_obv_d = exp(pol_obv_c*Tm)
L = place(GL',CL',pol_obv_d)'
eig(GL-L*CL)
%% Obsevabor de Luenberger - Polos complejos
ts = 1
muestras = ts/Tm  % Mínimo 8 muestras
Mp = 0.1;
zita = abs(log(Mp))/sqrt(pi^2+(log(Mp))^2)
wn = 4/(ts*zita)
wd = wn*sqrt(1-zita^2)
s1 = -zita*wn + wd*j
pol_obv_c = [s1 conj(s1) 2*real(s1)]
pol_obv_d = exp(pol_obv_c*Tm)
L = place(GL',CL',pol_obv_d)'
eig(GL-L*CL)
%% Observador de Kalman
sys_new = ss (GL,[HL HL],CL,[DL DL],Tm)
Qn = diag([0.01^2 0.01^2 0.01^2])   % ruido del actuador
Rn = diag([0.01^2 0.01^2]) % ruido del sensor
[sys_obv,L,S] = kalman(sys_new,Qn,Rn)
eig(sys_obv.a)
%% Controlador y observador discretos unificados 2 DOF
Gcon = [GL-HL*Kest-L*CL+L*DL*Kest, HL*Ki - L*DL*Ki;
      -CL*GL+CL*HL*Kest, eye(2)-CL*HL*Ki]
Hcon = [zeros(n,r), L;
       eye(r) zeros(r,r)]
Ccon = [-Kest Ki]
Dcon = zeros(p,2*r)
sys_cond = ss(Gcon,Hcon,Ccon,Dcon,Tm)
Hsp = Hcon(:,1:r)
Hpv = Hcon(:,r+1:end)
GLc = [GL HL*Ccon;
       Hpv*CL Gcon]
HLc = [zeros(n,r);
       Hsp]   
CLc = [CL zeros(r,n+r)]
DLc = zeros(r,r)
sysLc = ss(GLc,HLc,CLc,DLc,Tm)
CuLc = [zeros(p,n) Ccon]
DuLc = zeros(p,r)
sysuLc = ss(GLc,HLc,CuLc,DuLc,Tm)
figure(3)
step(sysLc)
grid
figure(4)
step(sysuLc)
grid
clc
clear
close all
s = tf('s')
g = (s+1)/((s+10)*(s^2+s+1))
sys = ss(g)
AL = sys.a
BL = sys.b
CL = sys.c
DL = sys.d

n = 3 % Numero de estados
p = 1 % Número de actuadores
r = 1 % Número de salidas
%% Diseño sistema seguimiento 1 DOF
Q = diag([1 1 1]) % Penalidad de los estados
QI = [0.8];
R = diag([1])
Qn = diag([0.0032^2,0.0032^2,0.01^2]);  % Varianza de cada estado
Rn = diag([0.00032^2]);                % Varianza de la variable de salida
QXU = blkdiag(Q,R)
QWV = blkdiag(Qn,Rn)
reg = lqg(sys,QXU,QWV,QI,'1dof')
T = feedback(sys*reg,r)
U = feedback(reg,sys)
figure(1)
step(T)
grid
figure(2)
step(U)
grid
%% Diseño sistema seguimiento 2 DOF
reg = lqg(sys,QXU,QWV,QI,'2dof')

Ac_2 = reg.a;
Bc_2 = reg.b;
Cc_2 = reg.c;
Dc_2 = reg.d;

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
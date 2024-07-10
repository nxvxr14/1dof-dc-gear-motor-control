clc
clear
syms x1 x2 x3 x4 u1 u2 u3 u4
f1 = -x2 - x1^2 + u2^2 + u1*u3;
f2 = -x3*x2 + u2^2 ;
f3 = -x4 + u4*x3 + u1;
f4 = -x4^2 + u1 
F = [f1; f2; f3; f4]
X = [x1; x2; x3; x4]
U = [u1; u2; u3; u4]
A = jacobian(F,X)
B = jacobian(F,U)
x10 = 2;
x20 = 8;
x30 = 2;
x40 = 1;
u10 = x40^2
u20 = sqrt(x30*x20)
u40 = (x40 - u10)/x30
u30 = (x20 + x10^2 - u20^2)/u10
AL = double(subs(A,{x1,x2,x3,x4,u1,u2,u3,u4},{x10,x20,x30,x40,u10,u20,u30,u40}))
BL = double(subs(B,{x1,x2,x3,x4,u1,u2,u3,u4},{x10,x20,x30,x40,u10,u20,u30,u40}))
CL = [1 1 0 0;
      0 0 5 -2]
DL = [0 0 0 0;
      0 0 0 0]
sys = ss(AL,BL,CL,DL)
Gs = tf(sys)
eig(AL)
%% Matriz de controlabilidad y observabilidad
n = 4 % Numero de estados
p = 4 % Número de actuadores
r = 2 % Número de salidas
Ahat = [AL zeros(n,r);
        -CL zeros(r,r)];
Bhat = [BL;
        -DL]    
M = ctrb(Ahat,Bhat)
rank(M)

Vob = obsv(AL,CL)
rank(Vob) % rango del numero de estado
%% Diseño sistema seguimiento 1 DOF
Q = diag([1 1 1 1]) % Penalidad de los estados
QI = diag([0.8 1]);  % Penalidad de la componente integral
R = diag([1 1 1 1])   % Penalidad de la señal de control
Qn = diag([0.0032^2,0.0032^2,0.01^2,0.0032^2]);  % Varianza de cada estado
Rn = diag([0.00032^2,0.00032^2]);                % Varianza de la variable de salida
QXU = blkdiag(Q,R)
QWV = blkdiag(Qn,Rn)
reg = lqg(sys,QXU,QWV,QI,'1dof')
T = feedback(sys*reg,eye(r))
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
%% Diseño punto de equilibrio
Q = diag([1 1 1 1]) % Penalidad de los estados
R = diag([1 1 1 1])
Qn = diag([0.0032^2,0.0032^2,0.01^2,0.0032^2]);  % Varianza de cada estado
Rn = diag([0.00032^2,0.00032^2]);                % Varianza de la variable de salida
QXU = blkdiag(Q,R)
QWV = blkdiag(Qn,Rn)
reg = lqg(sys,QXU,QWV)

Ac = reg.a
Bc = reg.b
Cc = reg.c
Dc = reg.d
xini = [0.5; -0.3; -0.7; 0.9]
ALc = [AL, BL*Cc;
       Bc*CL, Ac+Bc*DL*Cc]
BLc = zeros(2*n,1)
CLc = [CL DL*Cc]
DLc = zeros(r,1)

CuLc = [zeros(p,n) Cc]
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

t50 = datosIDE(:,1)/1000;
t50 = (t50'-t50(1));
w50 = datosIDE(:,2)*0.104719755;
theta50 = datosIDE(:,3)*0.0174533;
pwm50 = datosIDE(:,4);

t30 = datosVALI(:,1)/1000;
t30 = (t30'-t30(1));
w30 = datosVALI(:,2)*0.104719755;
theta30 = datosVALI(:,3)*0.0174533;
pwm30 = datosVALI(:,4);

% t2 = datosPWMVAR(:,1)/1000;
% t2 = (t2'-t2(1));
% vm2 = datosPWMVAR(:,2)*0.104719755;
% theta2 = datosPWMVAR(:,3)*0.0174533;
% pwm2 = datosPWMVAR(:,4);


%PWM IDENTIFIACION
figure(1)
subplot(3,1,1);
plot(t50,theta50);
title("POSICION");
xlabel('Tiempo [s]') 
ylabel('Angulo [rad]') 
subplot(3,1,2);
plot(t50,w50);
title("VELOCIDAD");
xlabel('Tiempo [s]') 
ylabel('Velocidad [rad/s]') 
subplot(3,1,3);
stairs(t50,pwm50);
title("PWM");
xlabel('Tiempo [s]') 
ylabel('PWM [%]')
figure(2)
histogram(diff(t50))
title("PERIODO DE MUESTRE0");
xlabel('Tiempo [s]') 
ylabel('Muestras [#]') 


%PWM VALIDACION
figure(3)
subplot(3,1,1);
plot(t30,theta30);
title("POSICION");
xlabel('Tiempo [s]') 
ylabel('Angulo [rad]') 
subplot(3,1,2);
plot(t30,w30);
title("VELOCIDAD");
xlabel('Tiempo [s]') 
ylabel('Velocidad [rad/s]') 
subplot(3,1,3);
stairs(t30,pwm30);
title("PWM");
xlabel('Tiempo [s]') 
ylabel('PWM [%]')
figure(4)
histogram(diff(t30))
title("PERIODO DE MUESTRE0");
xlabel('Tiempo [s]') 
ylabel('Muestras [#]') 


%PWM VARIABLE
% subplot(3,1,1);
% plot(t2,theta2);
% title("POSICION");
% xlabel('Tiempo [s]') 
% ylabel('Angulo [rad]') 
% subplot(3,1,2);
% plot(t2,vm2);
% title("VELOCIDAD");
% xlabel('Tiempo [s]') 
% ylabel('Velocidad [rad/s]') 
% subplot(3,1,3);
% stairs(t2,pwm2);
% title("PWM");
% xlabel('Tiempo [s]') 
% ylabel('PWM [%]') 
% figure(4)
% histogram(diff(t2))
% title("PERIODO DE MUESTRE0");
% xlabel('Tiempo [s]') 
% ylabel('Muestras [#]') 

% w1 = [0;diff(theta)/0.030];
% plot(t',vm,t',w1)
% grid;
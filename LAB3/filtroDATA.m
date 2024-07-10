t = filkalmaV1(:,1)/1000;
t = (t'-t(1));
P = filkalmaV1(:,2)*0.0174533;
Pf = filkalmaV1(:,3)*0.0174533;
rpm = filkalmaV1(:,4)*0.1047197551;
rpmf = filkalmaV1(:,5)*0.1047197551;
pwm = filkalmaV1(:,6);

%PWM IDENTIFIACION
figure(1)
plot(t,P);
hold on;
plot(t,Pf);
title("POSICION");
xlabel('Tiempo [s]') 
ylabel('Angulo [rad]')
grid;
figure(2)
plot(t,rpm);
hold on;
plot(t,rpmf);
title("VELOCIDAD");
xlabel('Tiempo [s]') 
ylabel('Velocidad [rad/s]')
grid;
figure(3)
plot(t,pwm);
title("PWM");
xlabel('Tiempo [s]') 
ylabel('PWM [%]') 
grid;


figure(4)
histogram(diff(t))
title("PERIODO DE MUESTREO");
xlabel('Tiempo [s]') 
ylabel('Muestras [#]') 
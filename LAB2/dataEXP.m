t = prueba2(:,1)/1000;
t = (t'-t(1));
SP = prueba2(:,2)*0.0174533;
theta = prueba2(:,3)*0.0174533;
pwm = prueba2(:,5);

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
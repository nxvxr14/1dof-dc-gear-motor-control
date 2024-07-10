t = kalmaV1(:,1)/1000;
t = (t'-t(1));
SP = kalmaV1(:,2)*0.0174533;
theta = kalmaV1(:,3)*0.0174533;
i = kalmaV1(:,4);
theta1 = kalmaV1(:,5)*0.0174533;
vel = kalmaV1(:,6);
pwm = kalmaV1(:,7);

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
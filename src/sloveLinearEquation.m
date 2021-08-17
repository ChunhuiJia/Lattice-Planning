clc,clear;
s0=[0,10,2];
s1=[20,4,0];
t=5;
A=[1 ,0 ,0   ,0      ,0     ,0;
   0 ,1 ,0   ,0      ,0     ,0;
   0 ,0 ,2   ,0      ,0     ,0;
   1 ,t ,t^2 ,t^3   ,t^4    ,t^5;
   0 ,1 ,2*t ,3*t^2 ,4*t^3  ,5*t^4;
   0 ,0 ,2   ,6*t   ,12*t^2 ,20*t^3];
b=[s0(1),s0(2),s0(3),s1(1),s1(2),s1(3)]';
x = inv(A)*b;
a0=x(1);
a1=x(2);
a2=x(3);
a3=x(4);
a4=x(5);
a5=x(6);
plot_t = 0:0.1:t;
plot_y = a0+a1*plot_t+a2*plot_t.^2+a3*plot_t.^3+a4*plot_t.^4+a5*plot_t.^5;
plot_y_dot = a1+a2*2*plot_t+a3*3*plot_t.^2+a4*4*plot_t.^3+a5*5*plot_t.^4;
plot_y_dot_dot = a2*2+a3*6*plot_t+a4*12*plot_t.^2+a5*20*plot_t.^3;
subplot(1,3,1)
plot(plot_t,plot_y);
legend('s(t)');
subplot(1,3,2)
plot(plot_t,plot_y_dot);
legend("s'(t)");
subplot(1,3,3)
plot(plot_t,plot_y_dot_dot);
legend("s''(t)");

clc,clear;
s1_sample = [20,40,60,80];
l0_dot = 0;
l0_dot_dot = 0;
l0=[0,l0_dot,l0_dot_dot];
l1_sample = [-0.5,0,0.5];
for i=1:length(s1_sample)
    for j=1:length(l1_sample)
        s1=s1_sample(i);
        l1=[l1_sample(j),0,0];
        [plot_t,plot_y] = slovefunction(s1,l0,l1);
        plot(plot_t,plot_y);
        hold on;
        grid on;
    end
end

function [plot_t,plot_y] = slovefunction(dt,s0,s1)
    t=dt;
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
end
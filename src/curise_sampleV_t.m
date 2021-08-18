clc,clear;
v_u = 20;  % m/s
v_u_sample = [0,0.25*v_u,0.5*v_u,0.75*v_u,v_u];
s0_dot = 10;
s0_dot_dot = 2;
s0=[0,s0_dot,s0_dot_dot];
t_sample = [2,4,6,8];  % uint:s
for i=1:length(v_u_sample)
    for j=1:length(t_sample)
        t_s=t_sample(j);
        s1=[0,v_u_sample(i),0];
        [plot_t,plot_y,plot_y_dot,plot_y_dot_dot] = slovefunction(t_s,s0,s1);
        subplot(1,3,1)
        plot(plot_t,plot_y);
        hold on;
        grid on;
        subplot(1,3,2)
        plot(plot_t,plot_y_dot);
        hold on;
        grid on;
        subplot(1,3,3)
        plot(plot_t,plot_y_dot_dot);
        hold on;
        grid on;
    end
end

function [plot_t,plot_y,plot_y_dot,plot_y_dot_dot] = slovefunction(dt,s0,s1)
    t=dt;
    A=[1 ,0 ,0   ,0      ,0;
       0 ,1 ,0   ,0      ,0;
       0 ,0 ,2   ,0      ,0;
       0 ,1 ,2*t ,3*t^2 ,4*t^3;
       0 ,0 ,2   ,6*t   ,12*t^2];
    b=[s0(1),s0(2),s0(3),s1(2),s1(3)]';
    x = inv(A)*b;
    a0=x(1);
    a1=x(2);
    a2=x(3);
    a3=x(4);
    a4=x(5);
    plot_t = 0:0.1:t;
    plot_y = a0+a1*plot_t+a2*plot_t.^2+a3*plot_t.^3+a4*plot_t.^4;
    plot_y_dot = a1+a2*2*plot_t+a3*3*plot_t.^2+a4*4*plot_t.^3;
    plot_y_dot_dot = a2*2+a3*6*plot_t+a4*12*plot_t.^2; 
end
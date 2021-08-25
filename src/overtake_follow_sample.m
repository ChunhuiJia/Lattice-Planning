clc,clear;
ego_initV = 10; %ego vehicle speed,units:m/s
ego_initAccel = 0; % ego vehicle acceleration,units:m/s2
% step1: Create obstacles and draw obstacles
obstacle_in_t = 4;  % unit:s
obstacle_out_t = 8;   % unit:s
obstacle_v = 5;  %unit:m/s
obstacle_intercept = 20;  %unit:m
obstacle_length = 5;  %unit:m
half_obstacle_length = obstacle_length/2;
x = obstacle_in_t:0.1:obstacle_out_t;
y = obstacle_v .* x + obstacle_intercept;
fill_x = [obstacle_in_t,obstacle_in_t,obstacle_out_t,obstacle_out_t];
fill_y_temp = obstacle_v .* fill_x + obstacle_intercept;

fill_y_append = [-half_obstacle_length,half_obstacle_length,half_obstacle_length,-half_obstacle_length];
fill_y = fill_y_temp + fill_y_append;
fill(fill_x,fill_y,'b');
hold on;
plot(x,y,'r');
hold on;
axis([0,obstacle_out_t + 1,0,fill_y(3)+20])
% step2: sample t and the distance in front of and behind the obstacle
for sample_t = obstacle_in_t:1:obstacle_out_t
    obstacle_y = obstacle_v * sample_t + obstacle_intercept;
    sample_s = [obstacle_y+half_obstacle_length+10,obstacle_y+half_obstacle_length+5,obstacle_y-half_obstacle_length-5,obstacle_y-half_obstacle_length-10];
    for i=1:length(sample_s)
        s0=[0,ego_initV,ego_initAccel];
        s1=[sample_s(i),obstacle_v,0];
        [plot_t,plot_y] = slovefunction(sample_t,s0,s1);
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

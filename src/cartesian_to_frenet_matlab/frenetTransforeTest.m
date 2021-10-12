clc,clear;
close all;
% step1，生成一条参考线和四条车道线
x=0:0.1:50;
a0=0;
a1=0.3857;
a2=-0.018;
a3=0.00019;
y=a0+a1*x+a2*x.^2+a3*x.^3;

% 对s进行累加积分，并添加到参考线y中
s=zeros(1,length(y));
% ds = zeros(length(y),1);
for i=2:length(y)
    dx = x(i)-x(i-1);
    dy = y(i)-y(i-1);
    s(i) = s(i-1) + sqrt(dx^2+dy^2);
    % ds(i) = sqrt(dx^2+dy^2);
end

% 求航向角theta_r
theta_r = zeros(1,length(y));
for i=2:length(y)
    dy = y(i)-y(i-1);
    dx = x(i)-x(i-1);
    theta_r(i) = atan2(dy,dx);
end
theta_r(1)=theta_r(2);
% 车道宽度line_width=3.5m,利用frenetToCartesianSimple获取车道的边界线：
line_width=3.5;
left_line = zeros(2,length(y));
lleft_line = zeros(2,length(y));
right_line = zeros(2,length(y));
rright_line = zeros(2,length(y));
for i=1:length(y)
    x_r=x(i);
    y_r=y(i);
    theta_ref=theta_r(i);
    l=line_width/2;
    [x_x,y_x]=frenetToCartesianSimple(x_r,y_r,theta_ref,l);
    left_line(1,i)=x_x;
    left_line(2,i)=y_x;
    l=l+3.5;
    [x_x,y_x]=frenetToCartesianSimple(x_r,y_r,theta_ref,l);
    lleft_line(1,i)=x_x;
    lleft_line(2,i)=y_x;
    l=-line_width/2;
    [x_x,y_x]=frenetToCartesianSimple(x_r,y_r,theta_ref,l);
    right_line(1,i)=x_x;
    right_line(2,i)=y_x;
    l=l-3.5;
    [x_x,y_x]=frenetToCartesianSimple(x_r,y_r,theta_ref,l);
    rright_line(1,i)=x_x;
    rright_line(2,i)=y_x;
end
plot1=subplot(2,1,1);
grid on
plot(x,y,'--k');
hold on;
plot(left_line(1,:),left_line(2,:),'k');
plot(lleft_line(1,:),lleft_line(2,:),'k');
plot(right_line(1,:),right_line(2,:),'k');
plot(rright_line(1,:),rright_line(2,:),'k');
plot(10,3.5,'xr')
axis equal
xlabel('x');
ylabel('y');

% step2:已知道路中有一个障碍物,Cartesian坐标为(10,3.5),求它以道路中心线为reference line的Frenet坐标
left_frenet_line=zeros(2,length(y));
lleft_frenet_line=zeros(2,length(y));
right_frenet_line=zeros(2,length(y));
rright_frenet_line=zeros(2,length(y));
for i=1:length(y)
    %left_line
    x_p=left_line(1,i);
    y_p=left_line(2,i);
    ref_xline=x;
    ref_yline=y;
    index = findRefLineNearPoint(x_p,y_p,ref_xline,ref_yline);
    x_r=x(index);
    y_r=y(index);
    s_r=s(index);
    theta_r_in=theta_r(index);
    [l_out,s_out]=cartesianToFrenetSimple(x_r,y_r,s_r,theta_r_in,x_p,y_p);
    left_frenet_line(1,i)=s_out;
    left_frenet_line(2,i)=l_out;
    
    %lleft_line
    x_p=lleft_line(1,i);
    y_p=lleft_line(2,i);
    index = findRefLineNearPoint(x_p,y_p,ref_xline,ref_yline);
    x_r=x(index);
    y_r=y(index);
    s_r=s(index);
    theta_r_in=theta_r(index);
    [l_out,s_out]=cartesianToFrenetSimple(x_r,y_r,s_r,theta_r_in,x_p,y_p);
    lleft_frenet_line(1,i)=s_out;
    lleft_frenet_line(2,i)=l_out;
    
    %right_line
    x_p=right_line(1,i);
    y_p=right_line(2,i);
    index = findRefLineNearPoint(x_p,y_p,ref_xline,ref_yline);
    x_r=x(index);
    y_r=y(index);
    s_r=s(index);
    theta_r_in=theta_r(index);
    [l_out,s_out]=cartesianToFrenetSimple(x_r,y_r,s_r,theta_r_in,x_p,y_p);
    right_frenet_line(1,i)=s_out;
    right_frenet_line(2,i)=l_out;
    
    %rright_line
    x_p=rright_line(1,i);
    y_p=rright_line(2,i);
    index = findRefLineNearPoint(x_p,y_p,ref_xline,ref_yline);
    x_r=x(index);
    y_r=y(index);
    s_r=s(index);
    theta_r_in=theta_r(index);
    [l_out,s_out]=cartesianToFrenetSimple(x_r,y_r,s_r,theta_r_in,x_p,y_p);
    rright_frenet_line(1,i)=s_out;
    rright_frenet_line(2,i)=l_out;
end
%x点
x_p=10;
y_p=3.5;
index = findRefLineNearPoint(x_p,y_p,ref_xline,ref_yline);
x_r=x(index);
y_r=y(index);
s_r=s(index);
theta_r_in=theta_r(index);
[l_out,s_out]=cartesianToFrenetSimple(x_r,y_r,s_r,theta_r_in,x_p,y_p);
% 防止横坐标s不是单调递增，对二维数组对s进行升序排序
left_frenet_line=left_frenet_line';
sort(left_frenet_line);
left_frenet_line=left_frenet_line';

lleft_frenet_line=lleft_frenet_line';
sort(lleft_frenet_line);
lleft_frenet_line=lleft_frenet_line';

right_frenet_line=right_frenet_line';
sort(right_frenet_line);
right_frenet_line=right_frenet_line';

rright_frenet_line=rright_frenet_line';
sort(rright_frenet_line);
rright_frenet_line=rright_frenet_line';

ref_lline=zeros(1,length(y));
subplot(2,1,2);
grid on
plot(s,ref_lline,'--k');
hold on
plot(left_frenet_line(1,:),left_frenet_line(2,:),'k');
plot(lleft_frenet_line(1,:),lleft_frenet_line(2,:),'k');
plot(right_frenet_line(1,:),right_frenet_line(2,:),'k');
plot(rright_frenet_line(1,:),rright_frenet_line(2,:),'k');
plot(s_out,l_out,'xr')
xlabel('s');
ylabel('l');
axis equal

% step3:在frenet坐标系进行lattice采样
dt=40;
sample_l=[-1,1,2,-2,3,-3,4,-4];
s0=[0,0.3];  %初始状态
lattice_line_frenet=zeros(length(sample_l)+1,dt*10+1);
for i=1:length(sample_l)
    s1=[sample_l(i),0];
    [plot_s,plot_l,plot_l_dot,plot_l_ddot] = slove3ordFunction(dt,s0,s1);
    lattice_line_frenet(1,:)=plot_s;
    lattice_line_frenet(3*(i-1)+2,:)=plot_l;
    lattice_line_frenet(3*(i-1)+3,:)=plot_l_dot;
    lattice_line_frenet(3*(i-1)+4,:)=plot_l_ddot;
    plot(plot_s,plot_l,'b');
end
% 将Lattice采样结果，从frenet坐标系转换到Cartesian坐标系中
lattice_line_cartesian=zeros(length(sample_l)*2,dt*10+1);
lattice_line_catesian_temp=zeros(1,dt*10+1);
for i=1:length(sample_l)
    for j=1:length(plot_l)
        s_p = lattice_line_frenet(1,j);
        l_p = lattice_line_frenet(3*(i-1)+2,j);
        index = findRefLineNearPoint(s_p,l_p,s,ref_lline);
        x_r=x(index);
        y_r=y(index);
%         x_r=x(j);
%         y_r=y(j);
        theta_ref=theta_r(index);
        l=lattice_line_frenet(3*(i-1)+2,j);
        [x_x,y_x]=frenetToCartesianSimple(x_r,y_r,theta_ref,l);
        lattice_line_catesian_temp(1,j)=x_x;
        lattice_line_catesian_temp(2,j)=y_x;
    end
    % 防止横坐标s不是单调递增，对二维数组对s进行升序排序
    lattice_line_catesian_temp=lattice_line_catesian_temp';
    sort(lattice_line_catesian_temp);
    lattice_line_catesian_temp=lattice_line_catesian_temp';
    lattice_line_cartesian((i-1)+1,:)=lattice_line_catesian_temp(1,:);
    lattice_line_cartesian(2*i,:)=lattice_line_catesian_temp(2,:);
    plot(plot1,lattice_line_catesian_temp(1,:),lattice_line_catesian_temp(2,:),'b');
end

%step4:计算规划的Lattice路径的航向角，曲率，速度，加速度信息
% 计算曲率kx,kr和曲率的导数d_kr
figure;
latplot1=subplot(2,3,1);
hold on
grid on
title('y');
latplot2=subplot(2,3,2);
hold on
grid on
title('yaw-angle');
latplot3=subplot(2,3,3);
hold on
grid on
title('vx');
latplot4=subplot(2,3,4);
hold on
grid on
title('ax');
latplot5=subplot(2,3,5);
hold on
grid on
title('kx');
y_dot = a1+2*a2*x+3*a3*x.^2;
y_ddot = 2*a2+6*a3*x;
y_dddot = 6*a3;
kr=y_ddot./((1+y_dot.^2).^1.5);
d_kr=(y_dddot+y_dddot.*y_dot.^2-3.*y_dot.*y_ddot.^2)./((1+y_dot.^2).^3);
l_line = zeros(1,length(y));
lattice_line_cartesian_all=zeros(length(sample_l)*6,dt*10+1);
lattice_line_cartesian_all_temp=zeros(6,dt*10+1);
for i=1:length(sample_l)
    for j=1:length(plot_l)
        s_p = lattice_line_frenet(1,j);
        l_p = lattice_line_frenet(3*(i-1)+2,j);
        index = findRefLineNearPoint(s_p,l_p,s,ref_lline);
        x_r=x(index);
        y_r=y(index);
        l=lattice_line_frenet(3*(i-1)+2,j);
        l_dot=lattice_line_frenet(3*(i-1)+3,j);
        l_ddot=lattice_line_frenet(3*(i-1)+4,j);
        s_dot=10;
        s_ddot=0;
        theta_r_rad=theta_r(index);
        kr_input=kr(index);
        d_kr_input=d_kr(index);
        [x_x,y_x,theta_x,vx,ax,kx] = frenetToCartesian(x_r,y_r,l,l_dot,l_ddot,s_dot,s_ddot,theta_r_rad,kr_input,d_kr_input);
        lattice_line_cartesian_all_temp(1,j)=x_x;
        lattice_line_cartesian_all_temp(2,j)=y_x;
        lattice_line_cartesian_all_temp(3,j)=theta_x*57.3;
        lattice_line_cartesian_all_temp(4,j)=vx;
        lattice_line_cartesian_all_temp(5,j)=ax;
        lattice_line_cartesian_all_temp(6,j)=kx;
    end
    % 防止横坐标s不是单调递增，对二维数组对s进行升序排序
    lattice_line_cartesian_all_temp=lattice_line_cartesian_all_temp';
    sort(lattice_line_cartesian_all_temp);
    lattice_line_cartesian_all_temp=lattice_line_cartesian_all_temp';
    lattice_line_cartesian_all(6*(i-1)+1,:)=lattice_line_cartesian_all_temp(1,:);
    lattice_line_cartesian_all(6*(i-1)+2,:)=lattice_line_cartesian_all_temp(2,:);
    lattice_line_cartesian_all(6*(i-1)+3,:)=lattice_line_cartesian_all_temp(3,:);
    lattice_line_cartesian_all(6*(i-1)+4,:)=lattice_line_cartesian_all_temp(4,:);
    lattice_line_cartesian_all(6*(i-1)+5,:)=lattice_line_cartesian_all_temp(5,:);
    lattice_line_cartesian_all(6*(i-1)+6,:)=lattice_line_cartesian_all_temp(6,:);
    plot(latplot1,lattice_line_cartesian_all_temp(1,:),lattice_line_cartesian_all_temp(2,:),'b');
    hold on
    plot(latplot2,lattice_line_cartesian_all_temp(1,:),lattice_line_cartesian_all_temp(3,:),'b');
    hold on
    plot(latplot3,lattice_line_cartesian_all_temp(1,:),lattice_line_cartesian_all_temp(4,:),'b');
    hold on
    plot(latplot4,lattice_line_cartesian_all_temp(1,:),lattice_line_cartesian_all_temp(5,:),'b');
    hold on
    plot(latplot5,lattice_line_cartesian_all_temp(1,:),lattice_line_cartesian_all_temp(6,:),'b');
    hold on
end
plot(latplot1,x,y,'--k');
plot(latplot1,left_line(1,:),left_line(2,:),'k');
plot(latplot1,lleft_line(1,:),lleft_line(2,:),'k');
plot(latplot1,right_line(1,:),right_line(2,:),'k');
plot(latplot1,rright_line(1,:),rright_line(2,:),'k');
plot(latplot1,10,3.5,'xr')
xlabel('x');
ylabel('y');

function [plot_t,plot_y,plot_y_dot,plot_y_ddot] = slove3ordFunction(dt,s0,s1)
    t=dt;
    A=[1 ,0 ,0   ,0;
       0 ,1 ,0   ,0;
       1 ,t ,t^2 ,t^3;
       0 ,1 ,2*t ,3*t^2];
    b=[s0(1),s0(2),s1(1),s1(2)]';
    x = A\b;
    a0=x(1);
    a1=x(2);
    a2=x(3);
    a3=x(4);
    plot_t = 0:0.1:t;
    plot_y = a0+a1*plot_t+a2*plot_t.^2+a3*plot_t.^3;
    plot_y_dot = a1+2*a2.*plot_t+3*a3.*plot_t.*plot_t;
    plot_y_ddot = 2*a2+6*a3.*plot_t;
end
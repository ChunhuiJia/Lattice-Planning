function [l,l_dot,l_ddot,s,s_dot,s_ddot] = cartesianToFrenet(x_x,y_x,x_r,y_r,s_r,vx,ax,theta_x_rad,theta_r_rad,kx,kr,d_kr)
% caculate l
cos_theta_r = cos(theta_r_rad);
sin_theta_r = sin(theta_r_rad);
dx = x_x-x_r;
dy = y_x-y_r;
value_judge_sign = dy*cos_theta_r-dx*sin_theta_r;
if value_judge_sign > 0
    l_sign = 1;
else
    l_sign = -1;
end
l = l_sign*sqrt(dx*dx+dy*dy);

% caculate l_dot
delta_theta_rad = theta_x_rad - theta_r_rad;
tan_delta_theta = tan(delta_theta_rad);
cos_delta_theta = cos(delta_theta_rad);
l_dot = (1-kr*l)*tan_delta_theta;

% caculate l_ddot
kr_l_dot = d_kr*l+kr*l_dot;
l_ddot = -kr_l_dot*tan_delta_theta+(1-kr*l)/(cos_delta_theta^2)*(kx*(1-kr*l)/cos_delta_theta-kr);

% caculate s
s = s_r;

% caculate s_dot
s_dot = vx*cos_delta_theta/(1-kr*l);

% caculate s_ddot
s_ddot = (ax*cos_delta_theta-s_dot^2*(l_dot*(kx*(1-kr*l)/cos_delta_theta-kr)-kr_l_dot))/(1-kr*l);
end
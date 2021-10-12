function [x_x,y_x,theta_x,vx,ax,kx] = frenetToCartesian(x_r,y_r,l,l_dot,l_ddot,s_dot,s_ddot,theta_r,kr,d_kr)
sin_theta_r = sin(theta_r);
cos_theta_r = cos(theta_r);
% caculate x_x,y_x
x_x = x_r - l*sin_theta_r;
y_x = y_r + l*cos_theta_r;

% caculate theta_x
theta_x = atan2(l_dot,1-kr*l) + theta_r;
theta_x = normalize(theta_x);

% caculate vx
vx = sqrt((s_dot*(1-kr*l))^2+(s_dot*l_dot)^2);

% caculate kx
kr_l_dot = d_kr*l+kr*l_dot;
delta_theta = theta_x - theta_r;
tan_delta_theta = tan(delta_theta);
cos_delta_theta = cos(delta_theta);
kx = ((l_ddot+kr_l_dot*tan_delta_theta)*cos_delta_theta^2/(1-kr*l)+kr)*cos_delta_theta/(1-kr*l);

% caculate ax
ax = s_ddot*(1-kr*l)/cos_delta_theta+s_dot^2/cos_delta_theta*(l_dot*(kx*(1-kr*l)/cos_delta_theta-kr)-kr_l_dot);
end

function normal_var = normalize(var)

if var > pi
    normal_var = var - 2*pi;
elseif var < -pi
    normal_var = var + 2*pi;
else
    normal_var = var;
end
end

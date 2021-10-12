function [l,s]=cartesianToFrenetSimple(x_r,y_r,s_r,theta_r,x_x,y_x)
    % caculate l
    cos_theta_r = cos(theta_r);
    sin_theta_r = sin(theta_r);
    dx = x_x-x_r;
    dy = y_x-y_r;
    value_judge_sign = dy*cos_theta_r-dx*sin_theta_r;
    if value_judge_sign > 0
        l_sign = 1;
    else
        l_sign = -1;
    end
    l = l_sign*sqrt(dx*dx+dy*dy);
    % caculate s
    s=s_r;
end
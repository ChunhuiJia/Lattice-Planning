function [x_x,y_x]=frenetToCartesianSimple(x_r,y_r,theta_r,l)
    sin_theta_r = sin(theta_r);
    cos_theta_r = cos(theta_r);
    % caculate x_x,y_x
    x_x = x_r - l*sin_theta_r;
    y_x = y_r + l*cos_theta_r;
end
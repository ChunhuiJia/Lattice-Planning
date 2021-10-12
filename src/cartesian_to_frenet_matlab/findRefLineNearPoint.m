function index = findRefLineNearPoint(x_p,y_p,ref_xline,ref_yline)
    index = 1;
    square_d_min=10000;
    for i=1:length(ref_xline)
        dx = x_p-ref_xline(i);
        dy = y_p-ref_yline(i);
        square_d = dx*dx+dy*dy;
        if square_d < square_d_min
            square_d_min = square_d;
            index = i;
        end
    end
end
% File name "Calculate_z_c.m"

% Solve equation 3.10

function Z = Calculate_z_c(Target_pixel_Camera1,Target_pixel_Camera2,a,b)
    d = Target_pixel_Camera1(1,:)-Target_pixel_Camera2(1,:);
    f = a(1,1);
    Z = (f*b)./d;
end
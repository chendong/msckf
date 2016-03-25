function  axang = quat2axang(quat)
quat = quat/norm(quat);
angle = 2*acos(quat(1));
s = sqrt(1 - quat(1)*quat(1));
if s < 0.001
    x = quat(2);
    y = quat(3);
    z = quat(4);
    
else
    x = quat(2)/s;
    y = quat(3)/s;
    z = quat(4)/s;
end
axang = [x,y,z,angle];


end
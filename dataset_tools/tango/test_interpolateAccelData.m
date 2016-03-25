interpolate_accel= interpolateAccelData(acc_timestamp,gyro_timestamp,accel);
figure(1);
plot([1:n],interpolate_accel(2,:),[1:n],interpolate_accel(3,:),[1:n],interpolate_accel(4,:));
title('Interpolated accel x y z');
hold on;
figure(2);

n2= 16918;
plot([1:n2],accel(:,2),[1:n2],accel(:,3),[1:n2],accel(:,4));
title('original accel x y z');

groudtruth = state_groudtruth_estimate;
num_sample = size(groudtruth,1);

SubsampleFactor = 100;

p = zeros(3,num_sample);
q = zeros(4,num_sample);
for i = 1:num_sample
    p(:,i)=groudtruth(i,2:4)';
    q(:,i)=groudtruth(i,5:8)';    
end

figure(1);
q_plotPose(p(:, 1), q(:, 1), '', 0.5);
hold on;

for i = SubsampleFactor+1:SubsampleFactor:num_sample
    q_plotPose(p(:, i), q(:, i), '', 0.05);
end

plot3(p(1, :), p(2, :), p(3, :), 'k');

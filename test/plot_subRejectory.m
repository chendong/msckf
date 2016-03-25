function plot_subRejectory(sub_groud_truth,sample_factor)

% example:
%       plot_subRejectory(groudtruth(1:1000,:),100)

num_sample = size(sub_groud_truth,1);

SubsampleFactor = sample_factor;

p = zeros(3,num_sample);
q = zeros(4,num_sample);
for i = 1:num_sample
    p(:,i)=sub_groud_truth(i,2:4)';
    q(:,i)=sub_groud_truth(i,5:8)';    
end


q_plotPose(p(:, 1), q(:, 1), '', 0.5);
hold on;

for i = SubsampleFactor+1:SubsampleFactor:num_sample
    q_plotPose(p(:, i), q(:, i), '', 0.05);
end

plot3(p(1, :), p(2, :), p(3, :), 'k');
end

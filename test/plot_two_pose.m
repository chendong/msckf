function plot_two_pose(sub_groud_truth)

% example:
%       plot_two_pose(groudtruth(1:2,:))

num_sample = size(sub_groud_truth,1);
p = zeros(3,num_sample);
q = zeros(4,num_sample);
for i = 1:num_sample
    p(:,i)=sub_groud_truth(i,2:4)';
    q(:,i)=sub_groud_truth(i,5:8)';    
end

figure(1);
q_plotPose(p(:, 1), q(:, 1), '', 0.07);
hold on;
q_plotPose(p(:, 2), q(:, 2), '', 0.05);


plot3(p(1, :), p(2, :), p(3, :), 'k');
end

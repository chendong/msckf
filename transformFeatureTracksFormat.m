function featuretrack_3_m_max = transformFeatureTracksFormat(old_format)

feature_index = old_format(3:3:end,:);
max_index = max(feature_index(:));

% image num
image_num = size(old_format,1)/3;
featuretrack_3_m_max = zeros(3,image_num,max_index);
% initial all to -1
featuretrack_3_m_max(:) = -1;

for i = 1:image_num
    for j = 1:1000
        feature = old_format(i*3-2:i*3,j);
        index = feature(3,1);
        if index ~= -1
            
            featuretrack_3_m_max(:,i,index) = feature;
        end
        
    end
    
    
end

end
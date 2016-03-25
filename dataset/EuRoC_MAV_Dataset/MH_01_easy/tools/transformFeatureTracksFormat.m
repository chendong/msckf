function featuretrack_3_m_max = transformFeatureTracksFormat(old_format)


old_format(3:3:end,:) = old_format(3:3:end,:)+1;
feature_index = old_format(3:3:end,:);
max_index = max(feature_index(:));

% image num
image_num = size(old_format,1)/3;
features_per_line = size(old_format,2);
featuretrack_3_m_max = zeros(3,image_num,max_index);
% initial all to -1
featuretrack_3_m_max(:) = -1;

for i = 1:image_num
    for j = 1:features_per_line
        feature = old_format(i*3-2:i*3,j);
        index = feature(3,1);
        if index == ~isnan(index)
            featuretrack_3_m_max(:,i,index) = feature;
        end
       
        
    end
    
    
end

end
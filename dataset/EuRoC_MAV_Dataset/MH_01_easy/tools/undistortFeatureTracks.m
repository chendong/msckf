function undistorted_feature = undistortFeatureTracks(old_format,cu,cv,fu,fv,k1,k2,p1,p2)
% Usage : undistored_featuretracks = undistortFeatureTracks(featuretracks,cu,cv,fu,fv,w);
    image_num = size(old_format,1)/3;
    feature_num = size(old_format,2);
    for i = 1:image_num
        for j = 1:feature_num
            if old_format(3*i,j) ~= -1
                % x
                x = (old_format(3*i-2,j)- cu)/fu;
                % y
                y = (old_format(3*i-1,j)- cv)/fv;
                
                r2 = x*x + y*y;
                x_correct = x*(1+k1*r2+k2*r2*r2) + 2*p1*x*y + p2*(r2+2*x*x);
                y_correct = y*(1+k1*r2+k2*r2*r2) + 2*p2*x*y + p1*(r2+2*y*y);
                
                old_format(3*i-2,j) = fu*x_correct + cu;
                old_format(3*i-1,j) = fv*y_correct + cv;
                
                
                
            end
        end
    end

    undistorted_feature = old_format;
end
function undistorted_feature = undistortFeatureTracks(old_format,cu,cv,fu,fv,w)
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


                distR = sqrt(x*x + y*y);
                dOneOver2Tan = 1.0 / (2.0 * tan(w / 2.0));
                R = tan(distR * w) * dOneOver2Tan;
                if distR > 0.01
                    dFactor =  R / distR;
                else
                    dFactor = 1.0;
                end

                old_format(3*i-2,j) = x*dFactor;
                old_format(3*i-1,j) = y*dFactor;



            end
        end
    end

    undistorted_feature = old_format;



end
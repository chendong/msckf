function interpolated_v = interpolate(t_current,t1,t2,state1,state2)
ratio = (t_current-t1)/(t2-t1);
interpolated_v = ratio*(state2-state1)+state1;

end
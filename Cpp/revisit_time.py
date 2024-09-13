def revisit_time_(linear_length, cumulative_rotation, linear_speed = 0.3, rotational_speed= 0.52):
    
    t_l = linear_length / linear_speed
    
    
    t_r = cumulative_rotation / rotational_speed
    
    
    t_t = t_l + t_r
    
    
    
    return t_t



function [alpha_v] = arm_controller(ef_numeric, arm_ref, alpha, pos_rot, Kp_arm, jacob, grasp_id_rope)

err_ef = arm_ref - ef_numeric;
a1 = alpha(1);   a2 = alpha(2);   a3 = alpha(3);   a4 = alpha(4);   a5 = alpha(5);
px = pos_rot(1); py = pos_rot(2); rt = pos_rot(3);
J  = eval(jacob);

% iJ = pinv(J); alpha_v = iJ*Kp_arm*err_ef;
% iJ =  inv(J); alpha_v = iJ*Kp_arm*err_ef;
alpha_v = J\(Kp_arm*err_ef);

if sum(grasp_id_rope(:)) > 0 
   alpha_v_lim = 0.1;
else
   alpha_v_lim = 1.2;
end   
    
% alpha_v_lim = 0.5;
alpha_v(alpha_v >  alpha_v_lim) =  alpha_v_lim;
alpha_v(alpha_v < -alpha_v_lim) = -alpha_v_lim;

end


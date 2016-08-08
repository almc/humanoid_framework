function [grad, end_eff_ref] = biot_savart_run2(end_effector, loop_pos, loop_dir, biot_savart, world_to_loop_4x4)

%% calculation of the biot savart law (power 3 because not normalized r)
p  = end_effector;
grad = [0; 0; 0];
for i = 1:length(loop_pos(1,:))-1
    pl = loop_pos(:,i  );
    dl = loop_pos(:,i+1) - loop_pos(:,i);
    r  = p - pl;
    cp = cross(dl,r) ./ (norm(r)^3);
    grad = grad + cp;
end
%% virtual loop closing
pl = [loop_pos(1,end); loop_pos(2,end); loop_pos(3,end)];
dl = [loop_pos(1,1) - loop_pos(1,end); ...
      loop_pos(2,1) - loop_pos(2,end); ...
      loop_pos(3,1) - loop_pos(3,end)];
r  = p - pl;
cp = cross(dl,r) ./ (norm(r)^3);
grad = grad + cp;

grad = grad ./ length(loop_pos(1,:));
attraction_force = 0.1;
grad = attraction_force * grad ./ norm(grad);

% end_eff_ref = zeros(3,1);

% %% transformation C is between the world coordinate and the loop coordinate
% a1 = [1,0,0]'; a2 = [0,1,0]'; a3 = [0,0,1]';
% b1 = loop.parl_vector1; b2 = loop.parl_vector2; b3 = loop.perp_vector;
% M = [b1, b2, b3];
% O = [a1, a2, a3];
% C = linsolve(M, O);

grad_in_loop      = world_to_loop_4x4 * [grad;1];
grad_in_loop(1:2) = biot_savart.alfa * grad_in_loop(1:2);
grad_in_loop(3)   = biot_savart.beta * grad_in_loop(3);
grad              = world_to_loop_4x4 \ grad_in_loop;

end_eff_ref = end_effector + grad(1:3);


end
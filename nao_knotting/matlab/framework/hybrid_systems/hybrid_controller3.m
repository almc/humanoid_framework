switch state
   case 0
        arm1.ref(:,n) = [  1.5 + loop.center(1), 1.0 + loop.center(2), 2.5]';
        arm2.ref(:,n) = [ -1.5 + loop.center(1), 1.0 + loop.center(2), 2.5]';
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        
        d_arm1_err = norm(arm1.ref(:,n) - arm1.ef, 2);
        d_arm2_err = norm(arm2.ref(:,n) - arm2.ef, 2);
        d_base_err = norm(base.ref(:,n) - base.pos_rot', 2);
        
        if d_arm1_err < 0.1 && d_arm2_err < 0.1 && d_base_err < 0.2
           state = 1;
        end
   case 1
      %test_arm_reachability()
      base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
%       disp('moving towards rope end to grasp it');
%       delta_vec2 = rope.pos(:,end) - arm2.ef;
      delta_nrm2 = norm(rope.pos(:,end) - arm2.ef,2);
%       delta_pos2 = 0.2* delta_vec2 / delta_nrm2;
%       arm2.ref(:,n) = arm2.ef + delta_pos2;
      arm2.ref(:,n) = rope.pos(:,end);
      if delta_nrm2 < 0.1
          % grasp rope at position end, with arm 2.
          grasp_id_rope(2,end) = 1;
          state = 2;
      end
   case 2
%       disp('moving rope through the loop using biot_savart controller');
      base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
      arm1ef_in_loop           = world_to_loop_4x4 * [arm1.ef;1];
      arm2ef_in_loop           = world_to_loop_4x4 * [arm2.ef;1];
      loop_perp_vector_in_loop = world_to_loop_3x3 * [loop.perp_vector];
      d_arm1_loop = abs(dot(arm1ef_in_loop(1:3), loop_perp_vector_in_loop ));
      d_arm2_loop = abs(dot(arm2ef_in_loop(1:3), loop_perp_vector_in_loop ));
      d_arm1_cent = norm(arm1.ef - loop.center, 2);
      d_arm2_cent = norm(arm2.ef - loop.center, 2);
      if d_arm1_loop < 0.2 && d_arm1_cent < loop.radius
          arm1.ref(:,n) = arm1.ef;
      else
%           [arm1.grad, arm1.ref(:,n)] = biot_savart_run (arm1.ef, loop.pos,     loop, biot_savart, world_to_loop_3x3);
          [arm1.grad, arm1.ref(:,n)] = biot_savart_run3(arm1.ef, loop.pos, 0, biot_savart, world_to_loop_4x4);
          debug.point1(:,1) = arm1.ref(:,n);
          for idx = 2:10
             [debug.grad1, debug.point1(:,idx)] = biot_savart_run3(debug.point1(:,idx-1), loop.pos, 0, biot_savart, world_to_loop_4x4);
          end
      end
      if  d_arm2_loop < 0.2 && d_arm2_cent < loop.radius
          arm2.ref(:,n) = arm2.ef;
      else
%           [arm2.grad, arm2.ref(:,n)] = biot_savart_run (arm2.ef, loop.pos_rev, loop, biot_savart, world_to_loop_3x3);
          [arm2.grad, arm2.ref(:,n)] = biot_savart_run3(arm2.ef, loop.pos, 1, biot_savart, world_to_loop_4x4);
          debug.point2(:,1) = arm2.ref(:,n);
          for idx = 2:10
             [debug.grad2, debug.point2(:,idx)] = biot_savart_run3(debug.point2(:,idx-1), loop.pos, 1, biot_savart, world_to_loop_4x4);
          end
      end
      if d_arm1_loop < 0.2 && d_arm2_loop < 0.2  && d_arm1_cent < loop.radius && d_arm2_cent < loop.radius 
          state = 3; 
      end
   case 3
%        disp('passing rope through loop');
       base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
       arm1.ref(:,n) = (arm1.ef + arm2.ef) / 2;
       arm2.ref(:,n) = arm1.ref(:,n);
       if norm(arm1.ef - arm2.ef, 2) < 0.1
           grasp_id_rope(1,end) = 1;
           grasp_id_rope(2,end) = 0;
           state = 4;
       end
    case 4
%         disp('regrasping other end of the rope');
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        delta_nrm1 = norm(rope.pos(:,1) - arm2.ef, 2);
        if delta_nrm1 < 0.1
          grasp_id_rope(2,1) = 1;
          delta_nrm2 = norm(arm1.ref(:,n) + [loop.center(1); loop.center(2); 0] - arm1.ef, 2);
          delta_nrm3 = norm(arm2.ref(:,n) + [loop.center(1); loop.center(2); 0] - arm2.ef, 2);
          if delta_nrm2 < 0.1 && delta_nrm3 < 0.1
              % rope grasped from both sides, efs at stable positions
              state = 5;
          else
              arm1.ref(:,n) = arm1.ref(:,n) + [loop.center(1); loop.center(2); 0];
              arm2.ref(:,n) = arm2.ref(:,n) + [loop.center(1); loop.center(2); 0];
          end
        else
            arm2.ref(:,n) = rope.pos(:,1);
        end
    case 5
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        n_stabilization = 360; % 360
        if n < n_stabilization % 260 wait for rope to stabilize
            n_state_5 = n_stabilization; %260
            d_state_5 = 200;
            arm1.ref(:,n) = arm1.ef;
            arm2.ref(:,n) = arm2.ef;
            base.ref(:,n) = base.pos_rot;
            base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        else
            arm1.ref(:,n)   = [  0.0 + 0.5*cos( (n - n_state_5)*pi / d_state_5 ) + loop.center(1) ;
                                 1.5 + 0.0*sin( (n - n_state_5)*pi / d_state_5 ) + loop.center(2) ;
                                 3.0 + 0.5*sin( (n - n_state_5)*pi / d_state_5 )];

            arm2.ref(:,n)   = [  0.0 + 0.5*cos( (n - n_state_5)*pi / d_state_5 + pi ) + loop.center(1) ;
                                 1.5 + 0.0*sin( (n - n_state_5)*pi / d_state_5 + pi ) + loop.center(2) ;
                                 3.0 + 0.5*sin( (n - n_state_5)*pi / d_state_5 + pi )];
            base.ref(:,n)   = [  0.0 + loop.center(1)             ;
                                 3.2 + loop.center(2)             ;
                                 0*(n - n_state_5)*pi / d_state_5];
        end
        if (n == n_state_5 + d_state_5)
           state = 5.5;
        end
    case 5.5
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        grasp_id_rope(1,end) = 0;
        grasp_id_rope(2,1)   = 0;
        
        delta_nrm1 = norm(rope.pos(:,1) - arm1.ef,2);
        arm1.ref(:,n) = rope.pos(:,1);
        if delta_nrm1 < 0.1
          grasp_id_rope(1,1) = 1;
        end
        
        delta_nrm2 = norm(rope.pos(:,end) - arm2.ef,2);
        arm2.ref(:,n) = rope.pos(:,end);
        if delta_nrm2 < 0.1
          grasp_id_rope(2,end) = 1;
        end
        
        if delta_nrm1 < 0.1 && delta_nrm2 < 0.1
            state = 6;
        end
    case 6
        arm1.ref(:,n) = [  1.0 + loop.center(1), 1.0 + loop.center(2), 2.5]';
        arm2.ref(:,n) = [ -1.0 + loop.center(1), 1.0 + loop.center(2), 2.5]';
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        
        d_arm1_err = norm(arm1.ref(:,n) - arm1.ef, 2);
        d_arm2_err = norm(arm2.ref(:,n) - arm2.ef, 2);
        d_base_err = norm(base.ref(:,n) - base.pos_rot', 2);
        
        if d_arm1_err < 0.1 && d_arm2_err < 0.1 && d_base_err < 0.1
           state = 7;
        end
    case 7
%         disp('starting biot savart controller on the rope loop');
        if ~loop_strangled
            disp('something went wrong, loop not strangled')
        end
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        arm1.ref(:,n) = [  1.0 + loop.center(1), 1.0 + loop.center(2), 3.5]';
        arm2.ref(:,n) = [  -1.0, 1.0, 2.5 + 0.5]';
        arm1ef_in_rope_loop                = world_to_rope_loop_4x4 * [arm1.ef;1];
        arm2ef_in_rope_loop                = world_to_rope_loop_4x4 * [arm2.ef;1];
        rope_loop_perp_vector_in_rope_loop = world_to_rope_loop_3x3 * [rope_loop.perp_vector];
        d_arm1_rope_loop = abs(dot(arm1ef_in_rope_loop(1:3), rope_loop_perp_vector_in_rope_loop ));
        d_arm2_rope_loop = abs(dot(arm2ef_in_rope_loop(1:3), rope_loop_perp_vector_in_rope_loop ));
        d_arm1_cent = norm(arm1.ef - rope_loop.center, 2);
        d_arm2_cent = norm(arm2.ef - rope_loop.center, 2);
        if d_arm2_rope_loop < 0.1 && d_arm2_cent < rope_loop.radius
            arm2.ref(:,n) = arm2.ef;
        else
%             [arm2.grad, arm2.ref(:,n)] = biot_savart_run (arm2.ef, rope_loop.pos,     rope_loop, biot_savart, world_to_rope_loop_3x3);
            [arm2.grad, arm2.ref(:,n)] = biot_savart_run3(arm2.ef, rope_loop.pos, 0, biot_savart, world_to_rope_loop_4x4);
            debug.point3(:,1) = arm2.ref(:,n);
            for idx = 2:10
                [debug.grad3, debug.point3(:,idx)] = biot_savart_run3(debug.point3(:,idx-1), rope_loop.pos, 0, biot_savart, world_to_loop_4x4);
            end
        end

        if d_arm2_rope_loop < 0.1 && d_arm2_cent < rope_loop.radius 
            state = 8; 
        end
    case 8
%         disp('picking up with arm 1 to pass the knot the knot')
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        arm2.ref(:,n) = arm2.ef;
        grasp_id_rope(1,1) = 0;
        arm1.ref(:,n) = rope.pos(:,end);
        delta_nrm1 = norm(rope.pos(:,end) - arm1.ef,2);
        if delta_nrm1 < 0.1
          grasp_id_rope(1,end) = 1;
          grasp_id_rope(2,end) = 0;
          state = 9;
        end
    case 9
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        arm1.ref(:,n) = arm1.ef;
        arm2.ref(:,n) = rope_loop.center + [ -0.5, 0, -0.5]';
        delta_nrm3 = norm(arm2.ref(:,n) - arm2.ef,2);
        if delta_nrm3 < 0.1
            state = 10;
        end        
    case 10
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        arm1.ref(:,n) = arm1.ef;
        arm2.ref(:,n) = rope_loop.center + [ -0.8, 0, +0.5]';
        delta_nrm3 = norm(arm2.ref(:,n) - arm2.ef,2);
        if delta_nrm3 < 0.1
            state = 11;
            if rope.pos(3,1) - rope_loop.center(3) < -0.1
                state = 11;
            end
        end
    case 11
%         disp('regrasping the other end of the rope to secure the knot')
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        arm1.ref(:,n) = arm1.ef;
        arm2.ref(:,n) = rope.pos(:,1);
        delta_nrm1 = norm(rope.pos(:,1) - arm2.ef,2);
        if delta_nrm1 < 0.1
          grasp_id_rope(2,1) = 1;
          state = 12;
        end
    case 12
%         disp('tightening knot')
        base.ref(:,n) = [  0.0 + loop.center(1), 2.0 + loop.center(2), 0]';
        arm1.ref(:,n) = arm1.ef;
        arm2.ref(:,n) = arm2.ef; %rope_loop.center + [0, 0, 0]';
   otherwise
      disp('hybrid controller error');
end
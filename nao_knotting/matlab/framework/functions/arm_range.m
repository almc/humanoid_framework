function [inrange] = arm_range(arm_ref, base_ref, arm)

D = sqrt( (arm_ref(1)-base_ref(1))^2 + (arm_ref(2)-base_ref(2))^2 + (arm_ref(3)-arm.l1)^2);

if D >= arm.l2 + arm.l3 + arm.l4 + arm.l5
    inrange = 0;
else
    inrange = 1;
end

end
%% calculate loop properties
gx = gradient(x);
gy = gradient(y);
gz = gradient(z);
loop_center = [sum(x)./length(x);
               sum(y)./length(y);
               sum(z)./length(z)];

[n_1,n_23,p_1] = affine_fit([x',y',z']);
perp_vector  = n_1;
parl_vector1 = n_23(:,1);
parl_vector2 = n_23(:,2);
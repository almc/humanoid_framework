function [jacob, J1, J2, J3, J4, J5, EF] = arm_jacobian(arm, arm_id)

a1 = sym('a1'); a2 = sym('a2'); a3 = sym('a3'); a4 = sym('a4'); a5 = sym('a5');
px = sym('px'); py = sym('py'); rt = sym('rt');

L1 = arm.l1; L2 = arm.l2; L3 = arm.l3; L4 = arm.l4; L5 = arm.l5;

o   = [0 0 0 1]';

% tx0 = px;         ty0 = py; tz0 = 0;
if     arm_id == 1
    txd = +arm.d; tyd =  0; tzd = 0;
elseif arm_id ==2
    txd = -arm.d; tyd =  0; tzd = 0;
end
tx1 = 0;          ty1 =  0; tz1 = L1;
tx2 = L2*sin(a2); ty2 =  0; tz2 = L2*cos(a2);
tx3 = L3*sin(a3); ty3 =  0; tz3 = L3*cos(a3);
tx4 = L4*sin(a4); ty4 =  0; tz4 = L4*cos(a4);
tx5 = 0;          ty5 =  0; tz5 = L5;


R0 = [cos(rt)  -sin(rt)  0  px;
      sin(rt)   cos(rt)  0  py;
      0         0        1   0;
      0         0        0   1];

Rd = [1  0  0 txd;
      0  1  0 tyd;
      0  0  1 tzd;
      0  0  0  1];

R0 = R0*Rd;

R1 = [cos(a1) -sin(a1) 0 tx1;
      sin(a1)  cos(a1) 0 ty1;
      0        0       1 tz1;
      0        0       0  1];

R2 = [cos(a2) 0  sin(a2) tx2;
      0       1    0     ty2;
     -sin(a2) 0  cos(a2) tz2;
      0       0    0      1];

R3 = [cos(a3) 0  sin(a3) tx3;
      0       1    0     ty3;
     -sin(a3) 0  cos(a3) tz3;
      0       0    0      1];
  
R4 = [cos(a4) 0  sin(a4) tx4;
      0       1    0     ty4;
     -sin(a4) 0  cos(a4) tz4;
      0       0    0      1];

R5 = [cos(a5) -sin(a5) 0 tx5;
      sin(a5)  cos(a5) 0 ty5;
      0        0       1 tz5;
      0        0       0  1];


J1 = R0*o;                 J1 = J1(1:3);
J2 = R0*R1*o;              J2 = J2(1:3);
J3 = R0*R1*R2*o;           J3 = J3(1:3);
J4 = R0*R1*R2*R3*o;        J4 = J4(1:3);
J5 = R0*R1*R2*R3*R4*o;     J5 = J5(1:3);
EF = R0*R1*R2*R3*R4*R5*o;  EF = EF(1:3);

jacob = jacobian(EF, [a1 a2 a3 a4 a5]);

end
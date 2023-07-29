clearvars
syms xb zb th th1 th2 ...
     dxb dzb dth dth1 dth2 ...
     ddxb ddzb ddth ddth1 ddth2 ...
     tw t1 t2 real

%%% use a symbolic value for the mass of the ball
%%% to distinguish the dynamic model before and after throwing
syms mball real

%% vector of generalized coordinates and derivatives
q=[xb zb th th1 th2]';
dq=[dxb dzb dth dth1 dth2]';
ddq=[ddxb ddzb ddth ddth1 ddth2]';

%% kinematics and dynamics parameters
hb=0.193; ht=0.797; h1=0.32; h2=0.59;
r1=0.06; r2=0.06; d1=h1/2; d2=h2/2; db=0.54; %d1,d2 CoM of 2R links
rt1=0.062; rt2=0.159; rt3=0.02435; rt4=0.155;
rw=0.0985; g0=9.81;
mb=30; mt=27; m1=6.15; m2=3.96;

d23=[rt4-rt1;rw+hb+ht];
a3=norm(d23);
gamma=atan2(d23(2),d23(1));
%the inertia values given in the sketch correspond to frames having axes
%parallel to the axes of the world frame and origin at the associated CoM
%so we have axes x-z on the plane and y going inside
Ib=diag([0.465 0.4832 0.5509]);
It=diag([0.6234 0.6457 0.13769]);
%anyway, we need to express them considering the DH frame 3
%with z outgoing and x-y on the plane, hence we need the following rot mat
R=elem_rot_mat('z',gamma)*[1 0 0; 0 0 1;0 -1 0];
Ib=R*Ib*R';
It=R*It*R';
%compute diagonal I1,I2 considering cylinder links with radius r1,r2 respectively
i1x=0.5*m1*r1^2;
i1y=(1/12)*m1*(3*r1^2+h1^2);
i1z=i1y;
I1=diag([i1x i1y i1z]);
i2x=0.5*m2*r2^2;
i2y=(1/12)*m2*(3*r2^2+h2^2);
i2z=i2y;
I2=diag([i2x i2y i2z]);
%% useful stuff to compute energy
%matrix A_w0 from the world frame to the frame 0
A_w0=[0     0   1   0;
      0     -1  0   0;
      1     0   0   0;
      0     0   0   1];
%compute dh matrices from frame 0 to frame 5
dh_table=[-pi/2     0   xb  -pi/2;
          pi/2      0   zb  -pi/2;
          0         a3  0   th+gamma;
          0         h1  0   th1-gamma;
          0         h2  0   th2];

[T,A]=DHMatrix(dh_table);

A1 = A_w0*A{1};
A2 = A_w0*A{1}*A{2};
A3 = A_w0*A{1}*A{2}*A{3};
A4 = A_w0*A{1}*A{2}*A{3}*A{4};
A5 = A_w0*A{1}*A{2}*A{3}*A{4}*A{5};

R1=A1(1:3,1:3);
R2=A2(1:3,1:3);
R3=A3(1:3,1:3);
R4=A4(1:3,1:3);
R5=A5(1:3,1:3);

z0=A_w0(1:3,1:3)*[0 0 1]';
z1=R1*[0 0 1]';
z2=R2*[0 0 1]';
z3=R3*[0 0 1]';
z4=R4*[0 0 1]';

%% Positions of joints and CoMs in the world frame
p1 = A1(1:3,4);
p2 = A2(1:3,4);
p3 = A3(1:3,4);
p4 = A4(1:3,4);

pcb = p2+elem_rot_mat('y',-th)*[0;0;rw];
pct = p2+elem_rot_mat('y',-th)*[-rt1+rt3;0;rw+hb+rt2];

pcbt = (1/(mb+mt))*(pcb*mb + pct*mt);

pc1 = [p3(1)+d1*cos(th+th1);
       0;
       p3(3)+d1*sin(th+th1)];

%compute the new parameters of link 2 
%influenced by the ball payload
m2l = m2 + mball;
d2l = (m2*d2 + mball*h2)/m2l;
diff2=[d2l-d2;0;0];
I2l=I2 + m2*(diff2'*diff2*eye(3)-diff2*diff2') + diag([0 0 mball*(h2-d2l)^2]);

pc2l = [p4(1)+d2l*cos(th+th1+th2);
        0;
        p4(3)+d2l*sin(th+th1+th2)];

%% Geometric Jacobians
%(linear (p), angular (o) part)
zero_31=zeros(3,1);
Jpbt=[z0     z1      cross(z2,pcbt-p2)    zero_31    zero_31];
Jobt=[zero_31   zero_31     z2   zero_31    zero_31];
Jbt=[Jpbt(1:2:3,:);
     Jobt(2,:)];

Jp1=[z0 z1 cross(z2,pc1-p2) cross(z3,pc1-p3) zero_31];
Jo1=[zero_31  zero_31  z2               z3               zero_31];
J1=[Jp1(1:2:3,:);
    Jo1(2,:)];

Jp2=[z0 z1 cross(z2,pc2l-p2) cross(z3,pc2l-p3) cross(z4,pc2l-p4)];
Jo2=[zero_31  zero_31  z2               z3               z4];
J2=[Jp2(1:2:3,:);
    Jo2(2,:)];

%% Kinetic energy for each link
mbt=mb+mt;
%computation for the inertia of base+torso around pcbt
%express pcb,pct,pcbt in frame3 of dh, where Ib and It are defined
rt_3=R3'*(pcbt-pct);
rb_3=R3'*(pcbt-pcb);
It_cbt=simplify(It+mt*(rt_3'*rt_3*eye(3)-rt_3*rt_3'));
Ib_cbt=simplify(Ib+mb*(rb_3'*rb_3*eye(3)-rb_3*rb_3'));
Ibt=It_cbt+Ib_cbt;
Tbt=simplify(0.5*(mbt*dq'*Jpbt'*Jpbt*dq + dq'*Jobt'*R3*Ibt*R3'*Jobt*dq));
T1=simplify(0.5*(m1*dq'*Jp1'*Jp1*dq + dq'*Jo1'*R4*I1*R4'*Jo1*dq));
T2=simplify(0.5*(m2l*dq'*Jp2'*Jp2*dq + dq'*Jo2'*R5*I2l*R5'*Jo2*dq));
T=simplify(Tbt+T1+T2);

%% Inertia matrix B(q)
B=simplify(mbt*Jpbt'*Jpbt+Jobt'*R3*Ibt*R3'*Jobt+ ...
           m1*Jp1'*Jp1+Jo1'*R4*I1*R4'*Jo1+ ...
           m2l*Jp2'*Jp2+Jo2'*R5*I2l*R5'*Jo2);

%% Coriolis and centrifugal terms c(q,dq)
C=sym(zeros(5,5,5));
c=sym(zeros(5,1));
for k=1:size(q)
    C(:,:,k)=0.5*(jacobian(B(:,k),q) + jacobian(B(:,k),q)' - diff(B,q(k)));
    c(k)=simplify(dq'*C(:,:,k)*dq);
end

%% gravitational terms
g = [0 0 -g0]';

Ubt=-mbt*g'*pcbt;
U1=-m1*g'*pc1;
U2=-m2l*g'*pc2l;
U=simplify(Ubt+U1+U2);

gradU=jacobian(U,q)';

%% S(q) matrix [5 x 3]
%actuator forces
u = [tw t1 t2]';
S = [1/rw 0 0;
     0    0 0;
     0    0 0;
     0    1 0;
     0    0 1];

%% A(q) matrix [5 x 2]
h = [zb; th];
A = jacobian(h,q)';
syms l1 l2 real
lam=[l1;l2];

%% set to constant values the fictitious joint coordinates (and the derivatives to null)
zb=0; th=0; dzb=0; dth=0;
B=eval(B);
c=eval(c);
gradU=eval(gradU);
A=eval(A);
%% robot dynamics in Lagrange formulation
B*ddq + c + gradU == S*u + A*lam;

%% Selection matrices Qf and Qr
Qf=[0 1 0 0 0;
    0 0 1 0 0];

Qr=[1 0 0 0 0;
    0 0 0 1 0;
    0 0 0 0 1];

G=null(A'*Qr');
q_r=Qr*q;
dq_r=Qr*dq;
ddq_r=Qr*ddq;

%% Reduced dynamic model
x=[q_r dq_r]';
M = simplify(G'*Qr*B*Qr'*G);
m = simplify(G'*Qr*(c+gradU));
E = G'*Qr*S;
%Lagrange equation
red_dyn_mod = M*ddq_r + m == E*u;
%state space form
iM=inv(M);
dx = [dq_r; iM*(E*u-m)];

%% Contact forces
contact_forces=-Qf*B*Qr'*G*iM*E*u+ ...
               +Qf*B*Qr'*G*iM*m+ ...
               -Qf*(c+gradU); %neglect the term with G_dot since it is a null matrix
contact_forces=simplify(contact_forces);

% consider the contact forces applied in the origin of the a new frame Fv
% Fv frame is frame2 of dh in our case
% so we need to consider R from F2 to W, i.e. R2'
% and also 
fv=R2'*R1*[0;0;1]*contact_forces(1);
mv=R2'*R2*[0;0;1]*contact_forces(2);

% define the two unit vectors along the edges
% in our 2d case the edges will be only points
e1=[0;0;-1]; e2=[0;0;1];

% calculate the moment the robot applies around edges of support polygon
p1=[-db/2;0;0];
p2=[db/2;0;0];
edge_torques=[-cross(p1,fv)+mv -cross(p2,fv)+mv];
mu = [e1'*edge_torques(:,1);
      e2'*edge_torques(:,2)];
D=simplify(-jacobian(mu,u));
b=simplify(mu+D*u);

%% create 2 files with [iM,m,E] and [D,b] respectively
matlabFunction(iM,m,E,G,'File','get_dyn_terms.m','Vars',[mball,th1,th2,dth1,dth2],'Optimize',false);
matlabFunction(D,b,'File','get_balance_terms.m','Vars',[mball,th1,th2,dth1,dth2],'Optimize',false);

%% create a file with [mv,fv]
matlabFunction(mv,fv,'File','get_gen_forces.m','Vars',[mball,th1,th2,dth1,dth2,tw,t1,t2]);
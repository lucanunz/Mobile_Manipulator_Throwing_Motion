function plot_robot(x, options)
    
arguments
    x (:,1) double
    options.th double = 0
    options.ball logical = true
end

th=options.th;
ball=options.ball;
%% kinematic parameters
global h1 h2 rt1 rt4 ht hb rw db;

%% forward kinematic to find position of the robot points to plot (homogeneous)
p0=[x(1);
    rw;
    0;
    1];
p1=[x(1)-rt1+rt4;
    rw+hb+ht;
    0;
    1];
p2=[p1(1)+h1*cos(x(2));
    p1(2)+h1*sin(x(2));
    0;
    1];
p3=[p2(1)+h2*cos(x(2)+x(3));
    p2(2)+h2*sin(x(2)+x(3));
    0;
    1];
p=[p0 p1 p2 p3];

%position of the caster wheels
r_caster=rw/5;
p_rearwheel=[p0(1)-db/2;
             r_caster;
             0;
             1];  
p_frontwheel=[p0(1)+db/2;
              r_caster;
              0;
              1];

% write the lines to plot in the format [x_init x_fin; y_init y_fin; 0 0; 1 1] (homogeneous)
torso=[p0(1)-rt1 p0(1)-rt1;
       rw+hb     rw+hb+ht;
       0         0;
       1         1];
link1=[p1(1) p2(1);
       p1(2) p2(2);
       0     0;
       1     1];
link2=[p2(1) p3(1);
       p2(2) p3(2);
       0     0;
       1     1];

rearwheel_ax=[p0(1)-db/2   p0(1)-db/2;
              2*r_caster  rw;
              0           0;
              1           1];
frontwheel_ax=[p0(1)+db/2   p0(1)+db/2;
               2*r_caster   rw;
               0            0;
               1            1];

torso_appendix=[p0(1)-rt1 p1(1);
                rw+hb+ht  p1(2);
                0         0;
                1         1];

length_ee=0.06;
%length_ee1=0.034;

end_effector1=[p3(1)-length_ee*(cos(x(2)+x(3))) p3(1)+length_ee*(cos(x(2)+x(3)));
              p3(2)-length_ee*(sin(x(2)+x(3))) p3(2)+length_ee*(sin(x(2)+x(3)));
              0                               0;
              1                               1];
% end_effector1=[p3(1)-length_ee*sin(x(2)+x(3)) p3(1)+length_ee*sin(x(2)+x(3));
%                p3(2)+length_ee*cos(x(2)+x(3)) p3(2)-length_ee*cos(x(2)+x(3));
%                0                               0;
%                1                               1];
% end_effector2=[p3(1)-length_ee1*sin(x(2)+x(3)) p3(1)-length_ee1*sin(x(2)+x(3))+length_ee*cos(x(2)+x(3));
%                p3(2)+length_ee1*cos(x(2)+x(3)) p3(2)+length_ee1*cos(x(2)+x(3))+length_ee*sin(x(2)+x(3));
%                0                               0;
%                1                               1];
% end_effector3=[p3(1)+length_ee1*sin(x(2)+x(3)) p3(1)+length_ee1*sin(x(2)+x(3))+length_ee*cos(x(2)+x(3));
%                p3(2)-length_ee1*cos(x(2)+x(3)) p3(2)-length_ee1*cos(x(2)+x(3))+length_ee*sin(x(2)+x(3));
%                0                               0;
%                1                               1];

% represent the rectangle considering only the 4 vertices as homogeneous
% points [x_i; y_i; 0; 1] for i=1,2,3,4
% 4x4 matrix with [bottom_rear bottom_front top_rear top_front] columns
base=[p0(1)-db/2 p0(1)+db/2 p0(1)-db/2 p0(1)+db/2;
           rw         rw         rw+hb      rw+hb;
           0          0          0          0;
           1          1          1          1];

%% find the transformation to rotate the robot
% rotation about rear/front caster wheel when th >/< 0
T=eye(4);
iT=eye(4);
R=eye(4);
R(1:3,1:3)=elem_rot_mat('z',th);
if (th > 0)
    T(1:3,4)=p_rearwheel(1:3);
    iT(1:3,4)=-p_rearwheel(1:3);
elseif (th < 0)
    T(1:3,4)=p_frontwheel(1:3);
    iT(1:3,4)=-p_frontwheel(1:3);
end
%% apply the rotation to the robot
p=T*R*(iT*p);
p_rearwheel=T*R*(iT*p_rearwheel);
p_frontwheel=T*R*(iT*p_frontwheel);
rearwheel_ax=T*R*(iT*rearwheel_ax);
frontwheel_ax=T*R*(iT*frontwheel_ax);
torso=T*R*(iT*torso);
link1=T*R*(iT*link1);
link2=T*R*(iT*link2);
torso_appendix=T*R*(iT*torso_appendix);
base=T*R*(iT*base);
end_effector1=T*R*(iT*end_effector1);
% end_effector2=T*R*(iT*end_effector2);
% end_effector3=T*R*(iT*end_effector3);

%% plot
hold on
yline(0,'Color','k','LineStyle','-','LineWidth',0.01)
plot(base(1,1:2),base(2,1:2),'LineWidth',0.5,'Color','k')
plot(base(1,2:2:4),base(2,2:2:4),'LineWidth',0.5,'Color','k')
plot(base(1,3:4),base(2,3:4),'LineWidth',0.5,'Color','k')
plot(base(1,1:2:3),base(2,1:2:3),'LineWidth',0.5,'Color','k')
plot(rearwheel_ax(1,:),rearwheel_ax(2,:),'LineWidth',0.5,'Color','k')
plot(frontwheel_ax(1,:),frontwheel_ax(2,:),'LineWidth',0.5,'Color','k')
plot(torso(1,:),torso(2,:),'LineWidth',0.5,'Color','k')
plot(torso_appendix(1,:),torso_appendix(2,:),'LineWidth',0.5,'Color','k')
plot(link1(1,:),link1(2,:),'LineWidth',0.5,'Color','k')
plot(link2(1,:),link2(2,:),'LineWidth',0.5,'Color','k')
for k=1:4
    if k==1
        rectangle('Position',[p(1,k)-rw p(2,k)-rw 2*rw 2*rw],'Curvature',[1 1],'FaceColor','w')
    elseif k==4
        if (ball)
            plot(p(1,k),p(2,k),'.','MarkerSize',25,'Color','r')
        end
        plot(end_effector1(1,1:2),end_effector1(2,1:2),'Color','b','LineWidth',3)
%         plot(end_effector2(1,1:2),end_effector2(2,1:2),'Color','b','LineWidth',3)
%         plot(end_effector3(1,1:2),end_effector3(2,1:2),'Color','b','LineWidth',3)
    else
        plot(p(1,k),p(2,k),'.','MarkerSize',14,'Color','b')
    end
end
viscircles(p_rearwheel(1:2)',r_caster,'Color','k','LineWidth',0.5);
viscircles(p_frontwheel(1:2)',r_caster,'Color','k','LineWidth',0.5);
xlabel('x'),ylabel('z'),grid on

end
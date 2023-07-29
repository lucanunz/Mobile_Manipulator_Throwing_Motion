clear variables
close all
clc

file_name="1";
load("data/data_oneph"+file_name+".mat")

mball=3;
px=[];
state=[];
control=[];
x_i=sol(1:6);
T=T_t+T_s;
for k=1:9:size(sol,1)-6
    % --------- using only the optimized control torques of the solution with rho=5---------
    if(k==throw_idx)
        mball=0;
    end
    u_i=sol(k+6:k+8);
    [mu_v,fv]=get_gen_forces(mball,x_i(2),x_i(3),x_i(5),x_i(6),...
                                    u_i(1),u_i(2),u_i(3));
    px=[px; 
        mu_v(3)/fv(2)];
    state=[state;
           x_i'];
    control=[control;
             u_i'];
    x_i=dt_dynamics(@ct_dynamics,x_i,u_i,delta);
end
state=[state;
        x_i'];

t=0:size(px,1);
t=t.*delta;
f=figure;
f.Position=[276.2,77,988.8,658.4];
colororder(["blue","green"])
subplot(5,3,[10 11 13 14])
plot(t(1:end-1),px);
hold on,
xlabel("s","Interpreter","Latex"),ylabel("m","Interpreter","Latex")
yline(db/2,'r',{'limit'},'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom'), 
yline(-db/2,'r',{'limit'},'LabelHorizontalAlignment','right','LabelVerticalAlignment','top')
hxl=xline(T_t,'--k');%,{'throw'},'LabelHorizontalAlignment','left','LabelOrientation','horizontal');
xlim([0,T]), ylim("padded"), title('Zero moment point','Interpreter','Latex')
grid on
%% joint1
subplot(5,3,1)
plot(t,state(:,1)),grid on,hold on,
if q_max(1)~=inf
yline(q_max(1),'r--')
end
hxl=xline(T_t,'--k');%,{'throw'},'LabelHorizontalAlignment','left','LabelOrientation','horizontal');
xlabel("s","Interpreter","Latex"),ylabel("m","Interpreter","Latex")
xlim([0,T]), ylim("padded")
title("$x_b$",'Interpreter','Latex')

subplot(532),plot(t,state(:,4)),grid on,hold on,
if w_max(1)~=inf && -w_max(1)~=inf
    yline(w_max(1),'r--'),yline(-w_max(1),'r--')
end
hxl=xline(T_t,'--k');%,{'throw'},'LabelHorizontalAlignment','left','LabelOrientation','horizontal');
hxl.FontSize=8;
xlabel("s","Interpreter","Latex"),ylabel("m/s","Interpreter","Latex")
xlim([0,T]), ylim("padded")
title("$\dot{x}_b$","Interpreter","Latex")

subplot(533),plot(t(1:end-1),control(:,1)),grid on,hold on,
if t_max(1)~=inf
    yline(t_max(1),'r--'),yline(-t_max(1),'r--')
end
hxl=xline(T_t,'--k');%,{'throw'},'LabelHorizontalAlignment','left','LabelOrientation','horizontal');
hxl.FontSize=8;
xlabel("s","Interpreter","Latex"),ylabel("Nm","Interpreter","Latex")
xlim([0,T]), ylim("padded")
title("$u_w$","Interpreter","Latex")
%% joint 2
subplot(534),plot(t,state(:,2)),grid on,hold on,
if q_min(2)~=inf && q_max(2)~=inf
yline(q_max(2),'r--'),yline(q_min(2),'r--'),
end
hxl=xline(T_t,'--k');%,{'throw'},'LabelHorizontalAlignment','left','LabelOrientation','horizontal');
hxl.FontSize=8;
xlabel("s","Interpreter","Latex"),ylabel("rad","Interpreter","Latex")
xlim([0,T]), ylim("padded")
title("$q_1$","Interpreter","Latex")

subplot(535),plot(t,state(:,5)),grid on,hold on,
if -w_max(2)~=inf && w_max(2)~=inf
yline(w_max(2),'r--'),yline(-w_max(2),'r--')
end
hxl=xline(T_t,'--k');%,{'throw'},'LabelHorizontalAlignment','left','LabelOrientation','horizontal');
hxl.FontSize=8;
xlabel("s","Interpreter","Latex"),ylabel("rad/s","Interpreter","Latex")
xlim([0,T]), ylim("padded")
title("$\dot{q}_1$","Interpreter","Latex")

subplot(536),plot(t(1:end-1),control(:,2)), hold on,
if t_max(2)~=inf
yline(t_max(2),'r--'),yline(-t_max(2),'r--')
end
hxl=xline(T_t,'--k');%,{'throw'},'LabelHorizontalAlignment','left','LabelOrientation','horizontal');
hxl.FontSize=8;
grid on,xlabel("s","Interpreter","Latex"),ylabel("Nm","Interpreter","Latex")
xlim([0,T]), ylim("padded")
title("$u_1$","Interpreter","Latex")
%% joint 3
subplot(537),plot(t,state(:,3)),grid on,hold on,
if q_min(3)~=inf && q_max(3)~=inf
yline(q_max(3),'r--'),yline(q_min(3),'r--')
end
hxl=xline(T_t,'--k');%,{'throw'},'LabelHorizontalAlignment','left','LabelOrientation','horizontal');
hxl.FontSize=8;
xlabel("s","Interpreter","Latex"),ylabel("rad","Interpreter","Latex")
xlim([0,T]), ylim("padded")
title("$q_2$",'Interpreter','Latex')

subplot(538),plot(t,state(:,6)),grid on,hold on,
if -w_max(3)~=inf && w_max(3)~=inf
yline(w_max(3),'r--'),yline(-w_max(3),'r--')
end
hxl=xline(T_t,'--k');%,{'throw'},'LabelHorizontalAlignment','left','LabelOrientation','horizontal');
hxl.FontSize=8;
xlabel("s","Interpreter","Latex"),ylabel("rad/s","Interpreter","Latex")
xlim([0,T]), ylim("padded")
title("$\dot{q}_2$","Interpreter","Latex")

subplot(539),plot(t(1:end-1),control(:,3)),hold on,
if t_max(3)~=inf
yline(t_max(3),'r--'),yline(-t_max(3),'r--')
end
hxl=xline(T_t,'--k');%,{'throw'},'LabelHorizontalAlignment','left','LabelOrientation','horizontal');
hxl.FontSize=8;
grid on,xlabel("s","Interpreter","Latex"),ylabel("Nm","Interpreter","Latex")
xlim([0,T]), ylim("padded")
title("$u_2$","Interpreter","Latex")
exportgraphics(gcf,"video/figure_"+file_name+".png")
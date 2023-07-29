clear variables
close all
%%% The plot is related to the actual dynamics of the robot
%%% given the optimized control (not just taking optimized states)
%%% anyway, if the optimization works the result is the same

% set parameters
global h1 h2 rt1 rt4 ht hb rw db g0;
hb=0.193; ht=0.797; h1=0.32; h2=0.59;
rt1=0.062; rt4=0.155; rw=0.0985; 
db=0.54; g0=9.81;

file_number="1";
tol=1e-6;
fall=false;

load("data/data_oneph" + file_number + ".mat");

factor=delta*1000;
n_throw=T_t/delta;
loops=(size(sol,1)-6)/9;
vid(loops) = struct('cdata',[],'colormap',[]);
v=VideoWriter("video/video_" + file_number + ".avi");
v.Quality=100;
v.FrameRate=1/Ts;
open(v);
f = @ct_dynamics;
cnt=1;
x=sol(1:6);
for k=1:9:loops*9
    fall=false;
    u=sol(k+6:k+8);

    % compute the actual zmp
    [mu_v,fv]=get_gen_forces(mball,x(2),x(3),x(5),x(6),...
                                     u(1),u(2),u(3));
    zmp=mu_v(3)/fv(2);
    
    %plot the actual zmp
    if (k==1 || k==9*n_throw+1)
        gcf=figure('visible','on');
    else
        gcf=figure('visible','off');
    end
    axis([sol(1)-0.8 x_goal+0.5 -0.1 3]);
    daspect([1 1 1]);
    hold on
    xline(x_goal,'Color','r','LineWidth',0.03,'LineStyle','--','Label','goal');
    if (k<9*n_throw+1)
        if k~=1
            title("Initial configuration",'Interpreter','latex');
        else
            title('Throwing phase','Interpreter','latex');
        end
        if (abs(zmp) > db/2+tol)
            gcf.Visible='on';
            title('Falling','Interpreter','latex');
            plot_robot(x,'th',-zmp);
            plot(zmp+x(1),0,'.','MarkerSize',10,'Color','c','LineWidth',0.5);
            fall=true;
            fig=getframe(gcf);
            vid(cnt:cnt+150)=repmat(fig,1,151);
            exportgraphics(gcf,"video/fall_" + file_number+ ".png");
            break;
        else
            plot_robot(x);
            plot(zmp+x(1),0,'.','MarkerSize',10,'Color','c','LineWidth',0.5);
            vid(cnt)=getframe(gcf);
            if k==1
                exportgraphics(gcf,"video/init_" + file_number+ ".png");
            end
        end
    elseif (k==9*n_throw+1)
        mball=0;
        [x_traj,z_traj]=plot_balistic(x);
        plot(x_traj(1),z_traj(1),'.','MarkerSize',25,'Color','r');
        plot_robot(x,'ball',false);
        plot(zmp+x(1),0,'.','MarkerSize',10,'Color','c','LineWidth',0.5);
        title("Throwing configuration",'Interpreter','latex');
        vid(cnt)=getframe(gcf);
        exportgraphics(gcf,"video/throw_" + file_number+ ".png");
    else
        title('Stopping phase','Interpreter','latex');
        idx=(k-9*n_throw-1)*factor/9+1;
        if (abs(zmp) > db/2+tol)
            gcf.Visible='on';
            title('Falling','Interpreter','latex');
            plot_robot(x,'th',-zmp,'ball',false);
            plot(zmp+x(1),0,'.','MarkerSize',10,'Color','c','LineWidth',0.5);
            fall=true;
            fig=getframe(gcf);
            vid(cnt:cnt+150)=repmat(fig,1,151);
            exportgraphics(gcf,"video/fall_" + file_number+ ".png");
            break;
        else
            if (idx < size(x_traj,2))
                plot(x_traj(idx),z_traj(idx),'.','MarkerSize',25,'Color','r')
                plot(x_traj(idx:end),z_traj(idx:end),'Color','b')
            else
                plot(x_traj(end),z_traj(end),'.','MarkerSize',25,'Color','r')
            end
            plot_robot(x,'ball',false);
            plot(zmp+x(1),0,'.','MarkerSize',10,'Color','c','LineWidth',0.5);
            vid(cnt)=getframe(gcf);
        end
    end
    x=dt_dynamics(f,x,u,Ts);
    cnt=cnt+1;
end
if (~fall)
    gcf=figure('Visible','on');
    title("Stopping configuration",'Interpreter','latex');
    axis([sol(1)-0.8 x_goal+0.5 -0.1 3]);
    daspect([1 1 1]);
    hold on;
    plot(x_traj(end),z_traj(end),'.','MarkerSize',25,'Color','r');
    xline(x_goal,'Color','r','LineWidth',0.03,'LineStyle','--','Label','goal');
    plot_robot(x,'ball',false);
    exportgraphics(gcf,"video/stop_" + file_number+ ".png");
    vid(cnt)=getframe(gcf);
end

writeVideo(v,vid);
close(v);
fig = figure;
movie(fig,vid,1,1/delta);
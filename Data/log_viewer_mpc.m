clear; close all; clc;

%% load data
%type = 'SLUNGLOAD';
type = 'MULTIROTOR';
animation = false;
if(strcmp(type,'SLUNGLOAD'))
    n=18;
    m=4;
elseif(strcmp(type,'MULTIROTOR'))
    n=12;
    m=4;
end
logFile = '/home/youngdong/ROS/MPC_Multirotor_ROS/Data/DATA_MULTIROTOR_EXPERIMENT_ANALYTIC_2017_5_16_15_37_59.txt';
logData = importdata(logFile, '\t');
%% parse data
t_span = logData(logData(:,1)==10, 2).';
N = size(logData,2)-4;

t_compute = zeros(size(t_span,2),1);
t_compute_real = zeros(size(t_span,2),1);
cost = zeros(size(t_span,2),1);
x_w = zeros(size(t_span,2),n);
x_f = zeros(size(t_span,2),n);
x_o = zeros(size(t_span,2),3);
t_w = zeros(size(t_span,2),1);
x = zeros(size(t_span,2),N+1);
y = zeros(size(t_span,2),N+1);
z = zeros(size(t_span,2),N+1);
roll = zeros(size(t_span,2),N+1);
pitch = zeros(size(t_span,2),N+1);
yaw = zeros(size(t_span,2),N+1);
px = zeros(size(t_span,2),N+1);
py = zeros(size(t_span,2),N+1);
pz = zeros(size(t_span,2),N+1);
vx = zeros(size(t_span,2),N+1);
vy = zeros(size(t_span,2),N+1);
vz = zeros(size(t_span,2),N+1);
p = zeros(size(t_span,2),N+1);
q = zeros(size(t_span,2),N+1);
r = zeros(size(t_span,2),N+1);
px_dot = zeros(size(t_span,2),N+1);
py_dot = zeros(size(t_span,2),N+1);
pz_dot = zeros(size(t_span,2),N+1);
thrust = zeros(size(t_span,2),N);
Mx = zeros(size(t_span,2),N);
My = zeros(size(t_span,2),N);
Mz = zeros(size(t_span,2),N);
eig_old = zeros(size(t_span,2),N);
eig_new = zeros(size(t_span,2),N);
tension = zeros(size(t_span,2),1);
load_position_x = zeros(size(t_span,2),1);
load_position_y = zeros(size(t_span,2),1);
load_position_z = zeros(size(t_span,2),1);
for i=1:size(t_span,2)
    x_f = logData(logData(:,1) == 2, 4:4+17);
    x_o = logData(logData(:,1) == 3, 4:4+2);
    t_compute = logData(logData(:,1) == 4, 4); 
    t_compute_real = logData(logData(:,1) == 12, 4); 
    cost = logData(logData(:,1) == 5, 4);
    t_w = logData(logData(:,1) ==6, 4);
    eig_old = logData(logData(:,1) ==7, 4:4+N-1);
    eig_new = logData(logData(:,1) ==8, 4:4+N-1);
    tension = logData(logData(:,1) ==9, 4);
    load_position_x = logData(logData(:,1) ==11, 4);
    load_position_y = logData(logData(:,1) ==11, 5);
    load_position_z = logData(logData(:,1) ==11, 6);
    if(strcmp(type,'SLUNGLOAD'))
        x_w = logData(logData(:,1) == 1, 4:4+17);
        x = logData(logData(:,1) == 10, 4:end);
        y = logData(logData(:,1) == 20, 4:end);
        z = logData(logData(:,1) == 30, 4:end);
        roll = logData(logData(:,1) == 40, 4:end);
        pitch = logData(logData(:,1) == 50, 4:end);
        yaw = logData(logData(:,1) == 60, 4:end);
        px = logData(logData(:,1) == 70, 4:end);
        py = logData(logData(:,1) == 80, 4:end);
        pz = logData(logData(:,1) == 90, 4:end);
        vx = logData(logData(:,1) == 100, 4:end);
        vy = logData(logData(:,1) == 110, 4:end);
        vz = logData(logData(:,1) == 120, 4:end);
        p = logData(logData(:,1) == 130, 4:end);
        q = logData(logData(:,1) == 140, 4:end);
        r = logData(logData(:,1) == 150, 4:end);
        px_dot = logData(logData(:,1) == 160, 4:end);
        py_dot = logData(logData(:,1) == 170, 4:end);
        pz_dot = logData(logData(:,1) == 180, 4:end);
        thrust = logData(logData(:,1) == 1000, 4:end-1);
        Mx = logData(logData(:,1) == 2000, 4:end-1);
        My = logData(logData(:,1) == 3000, 4:end-1);
        Mz = logData(logData(:,1) == 4000, 4:end-1);
    elseif(strcmp(type,'MULTIROTOR'))
        x_w = logData(logData(:,1) == 1, 4:4+11);
        x = logData(logData(:,1) == 10, 4:end);
        y = logData(logData(:,1) == 20, 4:end);
        z = logData(logData(:,1) == 30, 4:end);
        roll = logData(logData(:,1) == 40, 4:end);
        pitch = logData(logData(:,1) == 50, 4:end);
        yaw = logData(logData(:,1) == 60, 4:end);
        
        vx = logData(logData(:,1) == 70, 4:end);
        vy = logData(logData(:,1) == 80, 4:end);
        vz = logData(logData(:,1) == 90, 4:end);
        p = logData(logData(:,1) == 100, 4:end);
        q = logData(logData(:,1) == 110, 4:end);
        r = logData(logData(:,1) == 120, 4:end);

        thrust = logData(logData(:,1) == 1000, 4:end-1);
        Mx = logData(logData(:,1) == 2000, 4:end-1);
        My = logData(logData(:,1) == 3000, 4:end-1);
        Mz = logData(logData(:,1) == 4000, 4:end-1);
    end
end

x_real = x(:,1);
y_real = y(:,1);
z_real = z(:,1);
roll_real = roll(:,1);
pitch_real = pitch(:,1);
yaw_real = yaw(:,1);
px_real = px(:,1);
py_real = py(:,1);
pz_real = pz(:,1);
vx_real = vx(:,1);
vy_real = vy(:,1);
vz_real = vz(:,1);
p_real = p(:,1);
q_real = q(:,1);
r_real = r(:,1);
px_dot_real = px_dot(:,1);
py_dot_real = py_dot(:,1);
pz_dot_real = pz_dot(:,1);
thrust_real = thrust(:,1);
Mx_real = Mx(:,1);
My_real = My(:,1);
Mz_real = Mz(:,1);
%% plot trajectories
if(animation)
   vid1 = VideoWriter('animation.avi');
   vid1.Quality = 80;
   vid1.FrameRate = 1/0.04;
   open(vid1);
end
figure(1);
t_span = round(t_span,2);
for i=1:1:size(t_span,2)
%for i=722:1:723
   clf;
   subplot(1,2,1);
   p1 = plot3(x(i,:),y(i,:),z(i,:),'r','LineWidth',5); hold on; grid on; axis equal; axis([-3 3 -3 3 -1 3]);
   %p2 = plot3(x_w(i,1),x_w(i,2),x_w(i,3),'o','MarkerFaceColor','b','MarkerSize',6,'MarkerEdgeColor','b');
   p3 = plot3(x_f(i,1),x_f(i,2),x_f(i,3),'o','MarkerFaceColor','r','MarkerSize',6,'MarkerEdgeColor','r');
   %p4 = plot3(x_o(i,1),x_o(i,2),x_o(i,3),'o','MarkerFaceColor','k','MarkerSize',10,'MarkerEdgeColor','k');
   p1.Color(4) = 0.4;
   if(strcmp(type,'MULTIROTOR'))
       drawRotors([x(i,1) y(i,1) z(i,1) roll(i,1) pitch(i,1) yaw(i,1)],0.3);
       quiver3(x(i,1),y(i,1),z(i,1),vx(i,1)*2,vy(i,1)*2,vz(i,1)*2,'LineWidth',2,'MaxHeadSize',8);
   end
   text(0.5,0,0,['Time : ' num2str(t_span(i))]);
   %plot3([x(i,1) load_position_x(i,1)],[y(i,1) load_position_y(i,1)],[z(i,1) load_position_z(i,1)],'-b','LineWidth',2);
   %drawLoad([load_position_x(i,1) load_position_y(i,1) load_position_z(i,1)]);
   view([126 37]);
   set(gca,'FontSize',15);
   xlabel('x(m)','FontSize',15); ylabel('y(m)','FontSize',15); zlabel('z(m)','FontSize',15);

   subplot(1,2,2);
   p1 = plot3(x(i,:),y(i,:),z(i,:),'r','LineWidth',5); hold on; grid on; axis equal; axis([-3 3 -3 3 -1 3]);
   %p2 = plot3(x_w(i,1),x_w(i,2),x_w(i,3),'o','MarkerFaceColor','b','MarkerSize',6,'MarkerEdgeColor','b');
   p3 = plot3(x_f(i,1),x_f(i,2),x_f(i,3),'o','MarkerFaceColor','r','MarkerSize',6,'MarkerEdgeColor','r');
   %p4 = plot3(x_o(i,1),x_o(i,2),x_o(i,3),'o','MarkerFaceColor','k','MarkerSize',10,'MarkerEdgeColor','k');
   p1.Color(4) = 0.4;
   text(0.5,0,0,['Time : ' num2str(t_span(i))]);
   view([0 90]);
   set(gcf,'Position',[66 1 1855 1001]);
   if(strcmp(type,'MULTIROTOR'))
       drawRotors([x(i,1) y(i,1) z(i,1) roll(i,1) pitch(i,1) yaw(i,1)],0.3);
       quiver3(x(i,1),y(i,1),z(i,1),vx(i,1)*2,vy(i,1)*2,vz(i,1)*2,'LineWidth',2,'MaxHeadSize',8);
   end
   %plot3([x(i,1) load_position_x(i,1)],[y(i,1) load_position_y(i,1)],[z(i,1) load_position_z(i,1)],'-b','LineWidth',2);
   %drawLoad([load_position_x(i,1) load_position_y(i,1) load_position_z(i,1)]);
   set(gca,'FontSize',15);
   xlabel('x(m)','FontSize',15); ylabel('y(m)','FontSize',15); zlabel('z(m)','FontSize',15);
   drawnow;
   if(animation)
        writeVideo(vid1,getframe(gcf));
   end
end
if(animation)
   close(vid1); 
end
%% Compute load velocity & acceleration
load_position = [load_position_x load_position_y load_position_z];
load_velocity = zeros(size(load_position_x,1),3);
load_acceleration = zeros(size(load_position_x,1),3);

load_velocity(1:end-1,:) = (load_position(2:end,:) - load_position(1:end-1,:))/0.04;
load_velocity(end,:) = load_velocity(end-1,:);
load_acceleration(1:end-2,:) = (load_velocity(2:end-1,:) - load_velocity(1:end-2,:))/0.04;
load_acceleration(end-1,:) = load_acceleration(end-2,:);
load_acceleration(end,:) = load_acceleration(end-1,:);
figure(123);
subplot(331); plot(t_span(1:end),load_position(:,1),'-','LineWidth',1); grid on;
subplot(332); plot(t_span(1:end),load_position(:,2),'-','LineWidth',1); grid on;
subplot(333); plot(t_span(1:end),load_position(:,3),'-','LineWidth',1); grid on;
subplot(334); plot(t_span(1:end),load_velocity(:,1),'-','LineWidth',1); grid on;
subplot(335); plot(t_span(1:end),load_velocity(:,2),'-','LineWidth',1); grid on;
subplot(336); plot(t_span(1:end),load_velocity(:,3),'-','LineWidth',1); grid on;
subplot(337); plot(t_span(1:end),load_acceleration(:,1),'-','LineWidth',1); grid on;
subplot(338); plot(t_span(1:end),load_acceleration(:,2),'-','LineWidth',1); grid on;
subplot(339); plot(t_span(1:end),load_acceleration(:,3),'-','LineWidth',1); grid on;

%% tension estimate
tension_estimate = zeros(size(tension));
m_L = 0.58;
for i=1:size(t_span,2)
    tension_estimate(i) = m_L*norm(9.81*[0 0 1].' + load_acceleration(i,:).');
end
%% plot tension
figure(2);
for i=1:length(tension)
    if(tension(i)>=100)
       tension(i) = tension(i-1); 
    end
end
plot(t_span(1:end),tension(1:end),'-','LineWidth',1); grid on; hold on;
plot(t_span(1:end),tension_estimate(1:end),'-','LineWidth',1);
set(gca,'FontSize',15);
xlabel('time(s)', 'FontSize',20);
ylabel('tension(N)','FontSize',20);
legend('measurement','estimation');
%% plot compute time
figure(2);
subplot(1,2,1); plot(t_span(3:end),t_compute(3:end),'-o','LineWidth',1); grid on;
set(gca,'FontSize',15);
xlabel('time(s)', 'FontSize',15);
ylabel('algorithm time(s)', 'FontSize',15);
ylim([0 0.1]);
subplot(1,2,2); plot(t_span(3:end),t_compute_real(3:end),'-o','LineWidth',1); grid on;
ylim([0 0.1]);
set(gca,'FontSize',15);
xlabel('time(s)', 'FontSize',15);
ylabel('compute time(s)', 'FontSize',15);
%% plot cost history
figure(3);
plot(t_span,cost,'-','LineWidth',2); grid on;
set(gca,'FontSize',15);
xlabel('time(s)', 'FontSize',15);
%% plot eigen values
figure(22);
subplot(121); plot(t_span,min(eig_old.'),'-k','LineWidth',2); hold on;
subplot(122); plot(t_span,min(eig_new.'),'-r','LineWidth',2);

%% plot state & input history with an assumption of static waypoint, obstacle, final states
if(strcmp(type,'MULTIROTOR'))
    X = [x_real y_real z_real roll_real pitch_real yaw_real vx_real vy_real vz_real p_real q_real r_real].';
    U = [thrust_real Mx_real My_real Mz_real].';
    title_state = {'x_L','y_L','z_L','\phi','\theta','\psi','\dot{x}_L','\dot{y}_L','\dot{z}_L','p','q','r'};
    title_input = {'T', 'M_x', 'M_y', 'M_z'};
    units_state = {'m','m','m','deg','deg','deg','m/s','m/s','m/s','deg/s','deg/s','deg/s'};
    units_input = {'N','N \cdot m','N \cdot m','N \cdot m'};
    figure(4);
    for i=1:n
        subplot(4,3,i);
        plot(t_span, X(i,:),'k','LineWidth',4); hold on;
% plot(0:0.04:(0.04*(size(X(i,:),2)-1)), X(i,:),'k','LineWidth',4); hold on;
        %plot(t_w, x_w(1,i),'o','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',10);
        %plot(t_span(end), x_f(1,i),'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10);
% plot((0.04*(size(X(i,:),2)-1)), x_f(1,i),'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10);
        if(i==1)
            %legend({'history','final','waypoint'},'Location','Best','FontSize',9);
            %legend({'history','final'},'Location','Best','FontSize',9);
        end
        xlabel('time(s)','FontSize',14); title(['$$' title_state{i} '$$'],'Interpreter','latex');
        ylabel(['$$' units_state{i} '$$'],'Interpreter','latex');
        set(gca,'FontSize',14);
    end
    figure(5);
    for i=1:m
        subplot(4,1,i);
        plot(t_span, U(i,:),'k','LineWidth',4); hold on;
% plot(0:0.04:(0.04*(size(X(i,:),2)-1)), U(i,:),'k','LineWidth',4); hold on;
        if(i==1)
            legend({'history'},'Location','Best','FontSize',9);
        end
        xlabel('time(s)','FontSize',14); title(['$$' title_input{i} '$$'],'Interpreter','latex');
        ylabel(['$$' units_input{i} '$$'], 'Interpreter','latex');
        set(gca,'FontSize',14);
    end
elseif(strcmp(type,'SLUNGLOAD'))
    X = [x_real y_real z_real roll_real pitch_real yaw_real px_real py_real pz_real vx_real vy_real vz_real p_real q_real r_real px_dot_real py_dot_real pz_dot_real].';
    U = [thrust_real Mx_real My_real Mz_real].';
    title_state = {'x_L','y_L','z_L','\phi','\theta','\psi','p_x','p_y','p_z','\dot{x}_L','\dot{y}_L','\dot{z}_L','p','q','r','\omega_{px}','\omega_{py}','\omega_{pz}'};
    title_input = {'T', 'M_x', 'M_y', 'M_z'};
    units_state = {'m','m','m','deg','deg','deg','','','','m/s','m/s','m/s','deg/s','deg/s','deg/s','/s','/s','/s'};
    units_input = {'N','N \cdot m','N \cdot m','N \cdot m'};
    figure(4);
    for i=1:n
        subplot(6,3,i);
        plot(t_span, X(i,:),'k','LineWidth',4); hold on;
        plot(t_w, x_w(1,i),'o','MarkerFaceColor','b','MarkerSize',12);
        plot(t_span(end), x_f(1,i),'o','MarkerFaceColor','r','MarkerSize',12);
        if(i==1)
            legend({'history','final','waypoint'},'Location','Best','FontSize',9);
        end
        xlabel('time(s)','FontSize',14); title(['$$' title_state{i} '$$'],'Interpreter','latex');
        ylabel(['$$' units_state{i} '$$'],'Interpreter','latex');
        set(gca,'FontSize',14);
    end
    figure(5);
    for i=1:m
        subplot(4,1,i);
        plot(t_span, U(i,:),'k','LineWidth',4); hold on;
        if(i==1)
            legend({'history'},'Location','Best','FontSize',9);
        end
        xlabel('time(s)','FontSize',14); title(['$$' title_input{i} '$$'],'Interpreter','latex');
        ylabel(['$$' units_input{i} '$$'], 'Interpreter','latex');
        set(gca,'FontSize',14);
    end
end
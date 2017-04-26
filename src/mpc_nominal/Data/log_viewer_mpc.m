clear; close all; clc;

%% load data
%type = 'SLUNGLOAD';
type = 'MULTIROTOR';
if(strcmp(type,'SLUNGLOAD'))
    n=18;
    m=4;
elseif(strcmp(type,'MULTIROTOR'))
    n=12;
    m=4;
end
logFile = '~/ROS/MPC_Multirotor_ROS/src/mpc_nominal/Data/DATA_MULTIROTOR_SIMULATION_ANALYTIC_2017_4_26_18_40_35.txt';
logData = importdata(logFile, '\t');
%% parse data
t_span = logData(logData(:,1)==10, 2).';
N = size(logData,2)-4;

t_compute = zeros(size(t_span,2),1);
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

for i=1:size(t_span,2)
    x_f = logData(logData(:,1) == 2, 4:4+17);
    x_o = logData(logData(:,1) == 3, 4:4+2);
    t_compute = logData(logData(:,1) == 4, 4);    
    cost = logData(logData(:,1) == 5, 4);
    t_w = logData(logData(:,1) ==6, 4);
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
figure(1);
for i=1:size(t_span,2)
   clf;
   subplot(1,2,1);
   p1 = plot3(x(i,:),y(i,:),z(i,:),'r','LineWidth',2); hold on; grid on; axis equal; axis([-2 4 -1 7 -1 1]);
   p2 = plot3(x_w(i,1),x_w(i,2),x_w(i,3),'o','MarkerFaceColor','b','MarkerSize',6,'MarkerEdgeColor','b');
   p3 = plot3(x_f(i,1),x_f(i,2),x_f(i,3),'o','MarkerFaceColor','r','MarkerSize',6,'MarkerEdgeColor','r');
   p4 = plot3(x_o(i,1),x_o(i,2),x_o(i,3),'o','MarkerFaceColor','k','MarkerSize',10,'MarkerEdgeColor','k');
   view([0 90]);
   subplot(1,2,2);
   p1 = plot3(x(i,:),y(i,:),z(i,:),'r','LineWidth',2); hold on; grid on; axis equal; axis([-2 4 -1 7 -1 1]);
   p2 = plot3(x_w(i,1),x_w(i,2),x_w(i,3),'o','MarkerFaceColor','b','MarkerSize',6,'MarkerEdgeColor','b');
   p3 = plot3(x_f(i,1),x_f(i,2),x_f(i,3),'o','MarkerFaceColor','r','MarkerSize',6,'MarkerEdgeColor','r');
   p4 = plot3(x_o(i,1),x_o(i,2),x_o(i,3),'o','MarkerFaceColor','k','MarkerSize',10,'MarkerEdgeColor','k');
   view([-53 6]);
   set(gcf,'Position',[66 1 1855 1001]);
   pause(0.01);
end
%% plot compute time
figure(2);
plot(t_span,t_compute,'-o','LineWidth',1);
%% plot cost history
figure(3);
plot(t_span,cost,'-','LineWidth',2);
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
        subplot(6,2,i);
        plot(t_span, X(i,:),'k','LineWidth',4); hold on;
        plot(t_w, x_w(1,i),'o','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',10);
        plot(t_span(end), x_f(1,i),'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10);
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
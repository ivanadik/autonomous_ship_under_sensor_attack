% This code implements the sensor attack of autonomous ship

clear
close all

% sampling time of the simulation
dt = 0.05;      % sampling time

% load waypoint for path generator
load('way_point.mat')

% applied time to generate path
time_s = [0 10 20 30 40 50 60 70 80 90 100];
time_array = 0:1:max(time_s);      

% generate path
way_px = spline(time_s,way_p_1(:,1),time_array);  
way_py = spline(time_s,way_p_1(:,2),time_array);

% feedback control options
feedback_mode = 3;      % 1 = ideal measurement,
                        % 2 = spoofed measurement, 
                        % 3 = estimation from adaptive observer
                        
% parameter for control path following
vt = 2;         % max speed
k_c = 0.5;      % koef. controller
k_a = 1;        % koef. controller


% initial state of the autonomous ship
xs(1) = -10;
ys(1) = 4;
vs(1) = 0;
yaws(1) = 1.484;

% initial speed actual and orientation rate actual
v_act = 0;
r_act = 0;

% matrix for observer
A = zeros(4,4);
C = [1 0 0 0; 0 1 0 0];
D = eye(2);
B = [0 0; 0 0; 1 0; 0 1];
Af = 10*eye(2);

% augmented matrix
Ab = [A zeros(4,2); Af*C -Af];
Bb = [B; zeros(2,2)];
Cb = [zeros(2,4) eye(2)];
Dd = [zeros(4,2); Af*D];

Ab = eye(6)+Ab*dt;

% initial variables
z           = [xs(1) ys(1) 0 yaws(1) 0 0]';     % initial ideal state
zbar        = [xs(1) ys(1) 0 yaws(1) 0 0]';     % initial estimated state
zreal       = [xs(1) ys(1) 0 yaws(1)]';         % initial actual state
zArray      = [];                               % storage of ideal state data
zbarArray   = [];                               % storage of estimated state data
zrealArray  = [];                               % storage of actual state data

yreal       = [xs(1) ys(1)]';                   % spoofed measurement
yrealArray  = [];                               % storage of spoofed measurement data

theta           = [0;0];                        % initial of actual sensor attack
thetabar        = [0;0];                        % initial of estimated sensor attack
thetaArray      = [];                           % storage of actual sensor attack data
thetabarArray   = [];                           % storage of estimated sensor attack data

% parameter observer
lambdaz = 0.999;
lambdat = 0.8;

% initial matrix for observer
Rz = 10000*diag([100 100]);
Rt = 0.1*diag([10 10]);
Pz = 0.001*eye(6);
Pt = 0.001*eye(2);
Gamma = [zeros(2,6)]';

% initial iteration
i = 1;

while true
    
    %  injecting sensor attack (after 59 seconds of simulation)
    if i*dt < 59
        theta = zeros(2,1);
    else
        if mod(i,1) == 0
            theta(1) = theta(1)-0.02;
        end
        if mod(i,1) == 0
            theta(2) = theta(2)-0.01;
        end
    end
    
    % updating storage
    zArray  = [zArray z];
    zbarArray  = [zbarArray zbar];
    zrealArray  = [zrealArray zreal];
    
    yrealArray  = [yrealArray yreal];
    
    thetaArray    = [thetaArray theta];
    thetabarArray = [thetabarArray thetabar];
    
    % selecting feedback control mode
    if feedback_mode == 1
        statex = xs(i);
        statey = ys(i);
    elseif feedback_mode == 2
        statex = yreal(1);
        statey = yreal(2);
    elseif feedback_mode == 3
        statex = zbar(1);
        statey = zbar(2);
    end
    
    % calculating error position and orientation
    [valx,idx] = min(abs(sqrt((way_px-statex).^2+(way_py-statey).^2)));
    
    if idx == 1
        idx = 2;
    end
    
    way_yaw = atan2(way_py(idx)-way_py(idx-1),way_px(idx)-way_px(idx-1));
    er(i)   = way_yaw-yaws(i);
    
    dxw     = way_px(idx) - way_px(idx-1);
    dyw     = way_py(idx) - way_py(idx-1);
    curv    = dxw * way_py(idx-1) -  dyw * way_px(idx-1);
    ep(i)   = ((statex*dyw)+ curv - (statey*dxw))/sqrt(dxw^2+dyw^2) + 10^(-32);
    
    % calculate control signal
    v_cd    = vt/(2+sqrt(abs(ep(i))+abs(er(i))));
    
    if abs(er(i)) < 0.0001 
        r_cd = k_c*er(i) + ep(i)*v_cd;
    else
        r_cd = k_c*er(i) + ep(i)*v_cd*(sin(er(i))/er(i));
    end
    
    % PID control for actuator
    v_act = cs_long_control(v_act, v_cd);
    r_act = cs_lat_control(r_act, r_cd);
    
    % Implementing control input
    a(i) = k_a*(v_act - vs(i))/dt;
    r(i) = r_act;
    
    u = [a(i); r(i)];
    
    % simulating model of autonomous ship in ideal case
    xs(i+1)     = xs(i)+dt*vs(i)*cos(yaws(i));
    ys(i+1)     = ys(i)+dt*vs(i)*sin(yaws(i));
    vs(i+1)     = vs(i)+dt*a(i);
    yaws(i+1)   = yaws(i)+dt*r(i);
    
    zreal = [xs(i+1) ys(i+1) vs(i+1) yaws(i+1)]';
    yreal = C*zreal+D*theta;
    
    % simulating autonomous ship in augmented state and adding sensor
    % attack
    fx = [z(3)*cos(z(4)); z(3)*sin(z(4)); a(i); r(i);...
         0; 0];
    
    z = Ab*z + fx*dt + Dd*theta*dt;
    y = Cb*z;
    
        
    %------------ ADAPTIVE OBSERVER ---------------%
    
    % calculating obserever gain
    Kx      = Pz*Cb'*inv(Cb*Pz*Cb'+Rz);
    Kt      = Pt*Gamma'*Cb'*inv(Cb*Gamma*Pt*Gamma'*Cb'+Rt);
    Gamma   = (eye(6)-Kx*Cb)*Gamma;
    
    % correcting estimated state
    zbar    = zbar+(Kx+Gamma*Kt)*(y-Cb*zbar);
    thetabar = thetabar-Kt*(y-Cb*zbar);
    
    fxbar = [zbar(3)*cos(zbar(4)); zbar(3)*sin(zbar(4)); a(i); r(i);...
         0; 0];
    
    % updating estimated state and parameter observer
    zbar = Ab*zbar+fxbar*dt+Dd*thetabar*dt;
    thetabar = thetabar;

    Fxbar = [0 0 cos(zbar(4)) -zbar(3)*sin(zbar(4));
        0 0 sin(zbar(4)) zbar(3)*cos(zbar(4));
        0 0 0 0;
        0 0 0 0];
    
    Fxb = [Fxbar zeros(4,2); zeros(2,6)];
    
    Pz      = (1/lambdaz)*(Ab+Fxb*dt)*(eye(6)-Kx*Cb)*Pz*(Ab+Fxb*dt)';
    Pt      = (1/lambdat)*(eye(2)-Kt*Cb*Gamma)*Pt;
    Gamma   = (Ab+Fxb*dt)*Gamma-Dd*dt;
    
    
    % stopping the simulation
    if idx == length(way_px)
        stop = 1;
    else
        stop = 0;
    end
    
    i = i+1;
    
    if stop == 1 || i > 5000
        a(i) = a(i-1);
        r(i) = r(i-1);
        ep(i) = ep(i-1);
        er(i) = er(i-1);
        zrealArray  = [zrealArray zreal];
        zbarArray  = [zbarArray zbar];
        zArray  = [zArray z];
        yrealArray  = [yrealArray yreal];
        thetaArray    = [thetaArray theta];
        thetabarArray = [thetabarArray thetabar];
        break
    end
    
end


% plotting x-y coordinate of simulation
figure(3)
fh = figure(3);
fh.WindowState = 'maximized';
img = imread('background.png');
image('CData',img,'XData',[-20 80],'YData',[40 -10])
hold on
plot(way_px,way_py,'LineWidth',2,'Color','g')
plot(yrealArray(1,:),yrealArray(2,:),'r','LineWidth',4)
plot(zbarArray(1,:),zbarArray(2,:),':','LineWidth',4,'Color','y')
plot(way_px(1),way_py(1),'.b','MarkerSize',40)
grid on
grid minor
axis equal
axis([-20 80 -10 40])
legend('Desired path','Path under cyber-attacks','Estimate','Initial point')
xlabel('x (m)')
ylabel('y (m)')
set(gca,'color','white','FontSize',20)


% plotting sensor attack and its estimation vs time
time_sim = [1:length(thetaArray(1,:))]*0.05;
figure(4)
fh = figure(4);
fh.Position = [0 50 950 550];
subplot(2,1,1)
plot(time_sim,thetaArray(1,:),'k','LineWidth',4)
hold on
plot(time_sim,thetabarArray(1,:),':b','LineWidth',4)
grid on
grid minor
xlim([0 90])
set(gca,'color','white','FontSize',20)
legend('\theta_{Actual}','\theta_{Estimate}','location','northwest')
ylabel({'$\theta_1$'},'Interpreter','latex')
subplot(2,1,2)
plot(time_sim,thetaArray(2,:),'k','LineWidth',4)
hold on
plot(time_sim,thetabarArray(2,:),':b','LineWidth',4)
grid on
grid minor
xlim([0 90])
set(gca,'color','white','FontSize',20)
ylabel({'$\theta_2$'},'Interpreter','latex')
xlabel('Time (s)')

% animating simulation of autonomous ship
figure(2); 
fh = figure(2);
fh.WindowState = 'maximized';
img = imread('background.png');
image('CData',img,'XData',[-20 80],'YData',[40 -10])

hold on

plot(way_px,way_py,'LineWidth',2,'Color','g')
plot(way_px(1),way_py(1),'.b','MarkerSize',40)
axis equal
axis([-20 80 -10 40])
grid on

for j = 1:size(zbarArray,2)-1 

    X   = zbarArray(1,j);
    Y   = zbarArray(2,j);
    Yaw = zbarArray(4,j);
    Xa  = yrealArray(1,j);
    Ya  = yrealArray(2,j);
    Yawa = atan2(yrealArray(2,j+1)-yrealArray(2,j),yrealArray(1,j+1)-yrealArray(1,j));
    
    if j < 10
        Yawa = Yaw;
    end

    % plot x-y
    plot(zbarArray(1,1:j),zbarArray(2,1:j),':','LineWidth',4,'Color','y')
    plot(yrealArray(1,1:j),yrealArray(2,1:j),'r','LineWidth',4)

    % draw the ship
    [X0,Y0,X1,Y1,X2,Y2,X3,Y3,X4,Y4] = draw_2D_ship(X,Y,Yaw);
    [X0a,Y0a,X1a,Y1a,X2a,Y2a,X3a,Y3a,X4a,Y4a] = draw_2D_ship(Xa,Ya,Yawa);

    if j==1

        D1a = patch('Faces',1:32,'Vertices',[X0a; Y0a]','Facecolor',[0.4 0 0.4],'FaceAlpha',0.5);
        D2a = patch('Faces',1:21,'Vertices',[X1a; Y1a]','Facecolor',[0.6 0 0.6],'FaceAlpha',0.5);
        D3a = patch('Faces',1:21,'Vertices',[X2a; Y2a]','Facecolor',[0.8 0 0.8],'FaceAlpha',0.5);
        D4a = patch('Faces',1:12,'Vertices',[X3a; Y3a]','Facecolor',[0.8 0 0.8],'FaceAlpha',0.5,'Edgecolor',[0.8 0 0.8]);
        D5a = patch('Faces',1:13,'Vertices',[X4a; Y4a]','Facecolor',[0.6 0 0.6],'FaceAlpha',0.5);

        D1 = patch('Faces',1:32,'Vertices',[X0; Y0]','Facecolor',[0 0.4 0.4]);
        D2 = patch('Faces',1:21,'Vertices',[X1; Y1]','Facecolor',[0 0.6 0.6]);
        D3 = patch('Faces',1:21,'Vertices',[X2; Y2]','Facecolor',[0 0.8 0.8]);
        D4 = patch('Faces',1:12,'Vertices',[X3; Y3]','Facecolor',[0 0.8 0.8],'Edgecolor',[0 0.8 0.8]);
        D5 = patch('Faces',1:13,'Vertices',[X4; Y4]','Facecolor',[0 0.6 0.6]);

    elseif j==2

        D1a = patch('Faces',1:32,'Vertices',[X0a; Y0a]','Facecolor',[0.4 0 0.4],'FaceAlpha',0.5);
        D2a = patch('Faces',1:21,'Vertices',[X1a; Y1a]','Facecolor',[0.6 0 0.6],'FaceAlpha',0.5);
        D3a = patch('Faces',1:21,'Vertices',[X2a; Y2a]','Facecolor',[0.8 0 0.8],'FaceAlpha',0.5);
        D4a = patch('Faces',1:12,'Vertices',[X3a; Y3a]','Facecolor',[0.8 0 0.8],'FaceAlpha',0.5,'Edgecolor',[0.8 0 0.8]);
        D5a = patch('Faces',1:13,'Vertices',[X4a; Y4a]','Facecolor',[0.6 0 0.6],'FaceAlpha',0.5);

        D1 = patch('Faces',1:32,'Vertices',[X0; Y0]','Facecolor',[0 0.4 0.4]);
        D2 = patch('Faces',1:21,'Vertices',[X1; Y1]','Facecolor',[0 0.6 0.6]);
        D3 = patch('Faces',1:21,'Vertices',[X2; Y2]','Facecolor',[0 0.8 0.8]);
        D4 = patch('Faces',1:12,'Vertices',[X3; Y3]','Facecolor',[0 0.8 0.8],'Edgecolor',[0 0.8 0.8]);
        D5 = patch('Faces',1:13,'Vertices',[X4; Y4]','Facecolor',[0 0.6 0.6]);

        pause(1);

    else

        set(D1a,'Faces',1:32,'Vertices',[X0a; Y0a]','Facecolor',[0.4 0 0.4],'FaceAlpha',1);
        set(D2a,'Faces',1:21,'Vertices',[X1a; Y1a]','Facecolor',[0.6 0 0.6],'FaceAlpha',1);
        set(D3a,'Faces',1:21,'Vertices',[X2a; Y2a]','Facecolor',[0.8 0 0.8],'FaceAlpha',1);
        set(D4a,'Faces',1:12,'Vertices',[X3a; Y3a]','Facecolor',[0.8 0 0.8],'FaceAlpha',1,'Edgecolor',[0.8 0 0.8]);
        set(D5a,'Faces',1:13,'Vertices',[X4a; Y4a]','Facecolor',[0.6 0 0.6],'FaceAlpha',1);

        set(D1,'Faces',1:32,'Vertices',[X0; Y0]','Facecolor',[0 0.4 0.4]);
        set(D2,'Faces',1:21,'Vertices',[X1; Y1]','Facecolor',[0 0.6 0.6]);
        set(D3,'Faces',1:21,'Vertices',[X2; Y2]','Facecolor',[0 0.8 0.8]);
        set(D4,'Faces',1:12,'Vertices',[X3; Y3]','Facecolor',[0 0.8 0.8],'Edgecolor',[0 0.8 0.8]);
        set(D5,'Faces',1:13,'Vertices',[X4; Y4]','Facecolor',[0 0.6 0.6]);


    end

    pause(0.01)   

end

legend('Desired path','Initial point','Path under cyber-attacks','Estimate')
xlabel('x (m)')
ylabel('y (m)')
set(gca,'color','white','FontSize',20)
    

% PID controller for actuator
function cs_long = cs_long_control(cs_long_act, cs_long_cd)
    er_cs_long = cs_long_cd-cs_long_act;
    cs_long = cs_long_act + 0.02 * er_cs_long;
end

function cs_lat = cs_lat_control(cs_lat_act, cs_lat_cd)
    er_cs_lat = cs_lat_cd-cs_lat_act;
    cs_lat = cs_lat_act + 0.2 * er_cs_lat;
end
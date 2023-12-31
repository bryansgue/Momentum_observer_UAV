%******************************************************************************************************************
%************************************ SEGUIMIENTO DE TRAYECTORIA **************************************************
%************************************* ROBOT MANIPULADOR AÉREO *****************************************************
%******************************************************************************************************************
clc; clear all; close all; warning off % Inicializacion

ts = 1/30;       % Tiempo de muestreo
tfin = 60;      % Tiempo de simulación
t = 0:ts:tfin;
load("chi_values.mat");
a=0;
b=0;
L=[a;b];
chi_values(:,1) = chi';
chi_real(:,1) = chi';

%% Configuracion ROS - ARM Sub
active = true;
Master = 'http://192.168.88.223:11311';
Local = '192.168.88.243';

Uav_Topic_Sub ='/dji_sdk/odometry';
Uav_Topic_Pub ='/m100/velocityControl';
Uav_Msg_Pub = 'geometry_msgs/Twist';

ROS_Options(Master,Local,active);
[UAV_Sub,Uav_Pub,Uav_Pub_Msg] = UAV_Options(Uav_Topic_Sub,Uav_Topic_Pub,Uav_Msg_Pub);

%% 1) PUBLISHER TOPICS & MSG ROS
u_ref = [0.0, 0, 0, 0];
send_velocities(Uav_Pub, Uav_Pub_Msg, u_ref);

%% 2) Suscriber TOPICS & MSG ROS
[odo] = odometry(UAV_Sub);
h(:,1) = odo(1:4);
h_p(:,1) = odo(5:8);

%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[xd, yd, zd, psid, xdp, ydp, zdp, psidp] = Trayectorias(3,t);
%% GENERALIZED DESIRED SIGNALS
hd = [xd; yd; zd; 0*psid];
hd_p = [xdp;ydp;zdp;0*psidp];

%% a) Posiciones iniciales del UAV
xu(1) = 0; 
yu(1) = 0; 
zu(1) = 1; 
psi(1)= 0;
h=[xu(1);yu(1);zu(1);psi(1)]
%% Velocidad inicial real del UAV
h_p = [0;0;0;0];
u_realchi = [0;0;0;0];
%% Ganancia Compensacion Dinamica
K = 1;
G = 1;


vcp(:,1) = [0;0;0;0];
%******************************************************************************************************************
%***************************************** CONTROLADOR ***********************************************************
%*****************************************************************************************************************
disp('Empieza el programa')


for k=1:length(t)
tic

%% 1) LEY DE CONTROL
%vc(:,k) = Vc_UAV(hdp(:,k),hd(:,k),xu(k),yu(k),zu(k),psi(k)); 

[vc(:,k),he(:,k)] = Solo_UAV(hd_p(1,k),hd_p(2,k),hd_p(3,k),0*hd_p(4,k),hd(:,k),h(:,k)); 

ul(k)=vc(1,k); 
um(k)=vc(2,k); 
un(k)=vc(3,k); 
w(k)=vc(4,k);

%% DYNAMIC COMPENSATION
vref(:,k) = dynamic_compensation_UAV(0*vcp(:,k), vc(:,k), h_p(:,k), chi_values, K, G, L, ts);

%% 2) Envia velocidades al UAV    
send_velocities(Uav_Pub, Uav_Pub_Msg, vref(:,k));

%% 2) DINAMICA DEL UAV (VELOCIDAD Y POSICION)
%u_real(:, k+1) = system_dynamic(chi_real(:,k), u_real(:,k), vref(:,k), psi(k), L,ts);

%% 3) Recibir datos - UAV          
    try
    [odo] = odometry(UAV_Sub);
    h(:,k+1) = odo(1:4);
    h(4,k+1) = Angulo(h(4,k+1)); 
    h_p(:,k+1) = odo(5:8);        
    catch
    h(:,k+1) = h(:,k);
    h_p(:,k+1) = h_p(:,k);    
    end
    
        J = [cos(psi(k)) -sin(psi(k)) 0 0;
         sin(psi(k)) cos(psi(k)) 0 0;
         0 0 1 0;
         0 0 0 1];
    
    v(:,k+1) = pinv(J)*h_p(:,k);

xu(k+1) = h(1,k+1);
yu(k+1) = h(2,k+1);
zu(k+1) = h(3,k+1);      
psi(k+1) = Angulo(h(4,k));

%% 2) ACELERATIONS VCP 
if k>1
ulp(k+1)=(v(1,k+1)- v(1,k))/ts;
ump(k+1)=(v(2,k+1)- v(2,k))/ts;
unp(k+1)=(v(3,k+1)- v(3,k))/ts;
wp(k+1) =(v(1,k+1)- v(4,k))/ts;
else
ulp(k+1)=ul/ts;   
ump(k+1)=um/ts; 
unp(k+1)=un/ts; 
wp(k+1) =w/ts; 
end
vcp(:,k+1) = [ulp(k+1);ump(k+1);unp(k+1);wp(k+1)];

% Integracion numerica metodo Runge-Kutta 
%h(:,k+1) = h(:,k)+ UAV_RK4(h(:,k),u_real(:,k+1),ts);

%% 4) Tiempo de muestreo
    while toc < ts
    end

%% 3) Tiempo de máquina   
dt(k) = toc;
end
disp('Fin de los calculos')
%% 1) PUBLISHER TOPICS & MSG ROS
u_ref = [0.0, 0, 0, 0];
send_velocities(Uav_Pub, Uav_Pub_Msg, u_ref);
%%
%*************************************************************************%
%**************ANIMACION SEGUIMIENTO DE TRAYECTORIA **********************%
%% ***********************************************************************%
save("T_MinNorma_UAV_DinInv_Real.mat","hd","hd_p","h","h_p","v","vref","dt","t");

%%

disp('Animacion RUN')

% 1) Parámetros del cuadro de animacion
figure(1)
axis equal
view(0,0) % Angulo de vista
cameratoolbar
title ("Simulacion")

% 2) Configura escala y color del UAV
Drone_Parameters(0.02);
H1 = Drone_Plot_3D(xu(1),yu(1),zu(1),0,0,psi(1));hold on


% c) Gráfica de la trayectoria deseada
plot3(xd,yd,zd,'--')
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

% 5) Simulación de movimiento del manipulador aéreo
for k=1:50:length(t)  
% a) Eliminas los dibujos anteriores del manipulador aéreo
delete(H1);
H1 = Drone_Plot_3D(xu(k),yu(k),zu(k),0,0,psi(k)); hold on
% b) Gráfica la posición deseada vs actual en cada frame
plot3(xu(1:k),yu(1:k),zu(1:k),'r')
hold on
plot3(xd(1:k),yd(1:k),zd(1:k),'b')

pause(0.1)
end

disp('FIN Simulación RUN')  

%%
%******************************************************************************************************************
%********************************************* GR�?FICAS ***********************************************************
%% ****************************************************************************************************************


% 2) Cálculos del Error
figure(2)
hxe= xd - xu(1:end-1);
hye= yd - yu(1:end-1);
hze= zd - zu(1:end-1);
%psie= Angulo(psid-psi(1:end-1));
plot(hxe), hold on, grid on
plot(hye)
plot(hze)
%plot(psie)
legend("hxe","hye","hze","psie")
title ("Errores de posición")

% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aéreo
figure(3)

subplot(4,1,1)
plot(xd)
hold on
plot(xu)
legend("xd","hx")
ylabel('x [m]'); xlabel('s [ms]');
title ("Posiciones deseadas y reales del extremo operativo del manipulador aéreo")

subplot(4,1,2)
plot(yd)
hold on
plot(yu)
legend("yd","hy")
ylabel('y [m]'); xlabel('s [ms]');

subplot(4,1,3)
plot(zd)
hold on
plot(zu)
grid on
legend("zd","hz")
ylabel('z [m]'); xlabel('s [ms]');

subplot(4,1,4)
plot(Angulo(psid))
hold on
plot(psi)
legend("psid","psi")
ylabel('psi [rad]'); xlabel('s [ms]');

% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aéreo
figure(4)


plot(vc(1,20:end))
hold on
plot(h_p(1,20:end))
hold on
plot(vref(1,20:end))
legend("ulc","ul","ul_{ref}")
ylabel('x [m/s]'); xlabel('s [ms]');
title ("Posiciones deseadas y reales del extremo operativo del manipulador aéreo")

figure(5)
plot(vc(2,20:end))
hold on
plot(h_p(2,20:end))
hold on
plot(vref(2,20:end))
legend("umc","um","um_{ref}")
ylabel('y [m/s]'); xlabel('s [ms]');

figure(6)
plot(vc(3,20:end))
hold on
plot(h_p(3,20:end))
hold on
plot(vref(3,20:end))
legend("unc","un","un_{ref}")
ylabel('z [m/ms]'); xlabel('s [ms]');

figure(7)
plot(vc(4,20:end))
hold on
plot(h_p(4,20:end))
hold on
plot(vref(4,20:end))
legend("wc","w","w_{ref}")
ylabel('psi [rad/s]'); xlabel('s [ms]');
%%
figure(8)
  ul_e= vref(1,1:end) - v(1,1:size(vref(1,1:end),2));
  um_e= vref(2,1:end) - v(2,1:size(vref(1,1:end),2));
  un_e= vref(3,1:end) - v(3,1:size(vref(1,1:end),2));
  
  plot(ul_e), hold on, grid on
  plot(um_e)
  plot(un_e)
  legend("hxe","hye","hze","psie")
  title ("Errores de posición")
  
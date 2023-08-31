%% *************************
%% CONTROL PREDICTIVO DE UAV
%% *************************
clc; clear all; close all; warning off % Inicializacion

ts = 0.05;       % Tiempo de muestreo
tfin = 100;      % Tiempo de simulación
t = 0:ts:tfin;

load("chi_simple.mat");
chi_uav = chi';
chi_est(:,1) = chi';

a=0;
b=0;
c=0;
L=[a;b;c];
% load("chi_values.mat");


%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[xd, yd, zd, psid, xdp, ydp, zdp, psidp] = Trayectorias(3,t);
%% GENERALIZED DESIRED SIGNALS
hd = [xd; yd; zd; psid];
hdp = [xdp;ydp;zdp;psidp];

%% Velocidad inicial real del UAV
u = [0;0;0;0];

%% Ganancia Compensacion Dinamica
K1 = 1;
K2 = 1;

disp('Empieza el programa')
h = [0;0;1;0];
v = [0; 0;0;0];

x = [h;v];

for k=1:length(t)
    tic
    
    %% 1) LEY DE CONTROL
    uc(:,k) = Vc_UAV(hdp(:,k),hd(:,k),h(1,k),h(2,k),h(3,k),h(4,k));
    
    %% 2) ACELERATIONS VCP
    if k>1
        uc_p(:,k) = (uc(:,k)- uc(:,k-1))/ts ;
    else
        uc_p(:,k) = [0;0;0;0];
    end
    
    uc_p(:,k) = [0;0;0;0];
    %% DYNAMIC COMPENSATION
    
    [u_ref(:,k),chi_est(:,k+1)] = adaptive_OPTI(uc_p(:,k), uc(:,k), u(:,k), chi_est(:,k), K1, K2, L, ts);
    
    %% Dinamica del sistema
    x(:,k+1) = UAV_Dinamica_RK4(chi_uav,x(:,k),u_ref(:,k),L,ts);
    
    h(:,k+1) = x(1:4,k+1);
    u(:,k+1) = x(5:8,k+1);
    
    
    % % %% CHANGE DYNAMIC PARAMETETS
    minimo = -0.032;
    maximo =  0.0355;
    noise(:,k)  =  (maximo-minimo) .* rand(19,1) + minimo;
    chi_uav = chi_uav  + noise(:,k);
    
    %% 3) Tiempo de máquina
    dt(k) = toc;
    
end
disp('Fin de los calculos')

%*************************************************************************%
%**************ANIMACION SEGUIMIENTO DE TRAYECTORIA **********************%
%% ***********************************************************************%
disp('Animacion RUN')

% 1) Parámetros del cuadro de animacion
figure(1)
axis equal
view(-15,15) % Angulo de vista
cameratoolbar
title ("Simulacion")

% 2) Configura escala y color del UAV
Drone_Parameters(0.02);
H1 = Drone_Plot_3D(h(1,1),h(2,1),h(3,1),0,0,h(4,1));hold on


% c) Gráfica de la trayectoria deseada
plot3(xd,yd,zd,'--')
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

% 5) Simulación de movimiento del manipulador aéreo
for k=1:100:length(t)
    % a) Eliminas los dibujos anteriores del manipulador aéreo
    delete(H1);
    H1 = Drone_Plot_3D(h(1,k),h(2,k),h(3,k),0,0,h(4,k)); hold on
    % b) Gráfica la posición deseada vs actual en cada frame
    plot3(h(1,1:k),h(2,1:k),h(3,1:k),'r')
    hold on
    plot3(xd(1:k),yd(1:k),zd(1:k),'b')
    
    pause(0.1)
end

disp('FIN SIMULACION')

%%
%******************************************************************************************************************
%********************************************* GR�?FICAS ***********************************************************
%% ****************************************************************************************************************


% 2) Cálculos del Error
figure(2)
hxe= xd - h(1,1:end-1);
hye= yd - h(2,1:end-1);
hze= zd - h(3,1:end-1);
psie= Angulo(psid-h(4,1:end-1));
plot(hxe), hold on, grid on
plot(hye)
plot(hze)
plot(psie)
legend("hxe","hye","hze","psie")
title ("Errores de posición")



% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aéreo
figure(4)


plot(uc(1,1:end))
hold on
plot(u(1,1:end))
hold on
plot(u_ref(1,1:end))
legend("ulc","ul","ul_{ref}")
ylabel('x [m/s]'); xlabel('s [ms]');
title ("Posiciones deseadas y reales del extremo operativo del manipulador aéreo")

figure(5)
plot(uc(2,1:end))
hold on
plot(u(2,1:end))
hold on
plot(u_ref(2,1:end))
legend("umc","um","um_{ref}")
ylabel('y [m/s]'); xlabel('s [ms]');

figure(6)
plot(uc(3,1:end))
hold on
plot(u(3,1:end))
hold on
plot(u_ref(3,1:end))
legend("unc","un","un_{ref}")
ylabel('z [m/ms]'); xlabel('s [ms]');

figure(7)
plot(uc(4,1:end))
hold on
plot(u(4,1:end))
hold on
plot(u_ref(4,1:end))
legend("wc","w","w_{ref}")
ylabel('psi [rad/s]'); xlabel('s [ms]');

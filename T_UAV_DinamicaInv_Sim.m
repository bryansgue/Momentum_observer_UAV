%%******************************************************************************************************************
%************************************ SEGUIMIENTO DE TRAYECTORIA **************************************************
%************************************* ROBOT MANIPULADOR AÉREO *****************************************************
%******************************************************************************************************************
clc; clear all; close all; warning off % Inicializacion

ts = 1/30;       % Tiempo de muestreo
tfin = 150;      % Tiempo de simulación
t = 0:ts:tfin;
load("chi_simple.mat");
a=0;
b=0;
c=0;
L=[a;b;c];

chi_uav(:,1) = chi';
%% Variables definidas por la TRAYECTORIA y VELOCIDADES deseadas
[xd, yd, zd, psid, xdp, ydp, zdp, psidp] = Trayectorias(4,t);
%% GENERALIZED DESIRED SIGNALS
hd = [xd; yd; zd; psid];
hdp = [xdp;ydp;zdp;psidp];

%% a) Posiciones iniciales del UAV



%% Ganancia Compensacion Dinamica
K1 = 1;
K2 = 1;
%******************************************************************************************************************
%***************************************** CONTROLADOR ***********************************************************
%*****************************************************************************************************************
disp('Empieza el programa')
h = [0;0;1;0];
u = [0; 0;0;0];

x = [h;u];

for k=1:length(t)
    tic
    
    %% 1) LEY DE CONTROL
    %vc(:,k) = Vc_UAV(hdp(:,k),hd(:,k),xu(k),yu(k),zu(k),psi(k));
    
    [uc(:,k),he(:,k)] = Solo_UAV(hdp(1,k),hdp(2,k),hdp(3,k),0*hdp(4,k),hd(:,k),h(:,k));
    
    ul(k)=uc(1,k);
    um(k)=uc(2,k);
    un(k)=uc(3,k);
    w(k)=uc(4,k);
    %% 2) ACELERATIONS VCP
    if k>1
        uc_p(:,k) = (uc(:,k)- uc(:,k-1))/ts ;
    else
        uc_p(:,k) = [0;0;0;0];
    end
    
    %% DYNAMIC COMPENSATION
    u_ref(:,k) = dynamic_compensation_UAV(uc_p(:,k), uc(:,k), u(:,k), chi_uav, K1, K2, L, ts);
    
    %% 2) DINAMICA DEL UAV (VELOCIDAD Y POSICION)
    Tu(:,k) = zeros(4,1);
    Tu(1,k) = 2*(sign(0.2*sin(0.05*k)+3*cos(-0.1*k)));
    Tu(2,k) = 2*rand()*(sign(0.2*sin(0.05*k)+3*cos(-0.05*k)));
    Tu(3,k) = 1*sin(0.05*k)+1*cos(-0.025*k);
    Tu(4,k) = 1.5*(sign(0.2*sin(0.04*k)+3*cos(-0.04*k)));
    
    Tu(:,k) = 0.2*Tu(:,k);
    
    x(:,k+1) = UAV_Dinamica_RK4_T(chi_uav,x(:,k),u_ref(:,k),L,ts,Tu(:,k));
    
    h(:,k+1) = x(1:4,k+1);
    u(:,k+1) = x(5:8,k+1);
    
    xu(k+1) = h(1,k+1);
    yu(k+1) = h(2,k+1);
    zu(k+1) = h(3,k+1);
    psi(k+1) = Angulo(h(4,k));
    
    u_t(:,k) = u_ref(:,k) - Tu(:,k);
    
    %% Observador
%     M = function_M(chi_uav,L);
%     p(:,k) = M*u(:,k);
    if k>1
        u_p(:,k) = (u(:,k)- u(:,k-1))/ts ;
    else
        u_p(:,k) = [0;0;0;0];
    end
    u_p(:,k) = [0;0;0;0];
    
    u_total(:,k) = Model_um(u_p(:,k), u(:,k), chi_uav, L);
%     
%     Tau = u_m(:,k) + u_ext
%     
%     
% 
%     G = function_G();
%     C = function_C(chi_uav,u(:,k), L);
%     B(:,k) = G-C'*u(:,k);
%     
%     
%     
%     p_est_p = u_m(:,k) - B(:,k) + Tu_est(:,k);
%     p_p     = u_m(:,k) + C'*u(:,k)
%     
%     Tu_est_p(:,k) = 
    
    % Perturvacion
    % minimo = -0.00;
    % maximo =  0.00;
    % noise(:,k)  =  (maximo-minimo) .* rand(19,1) + minimo;
    % chi_real(:,k+1) = chi_real(:,k) + noise(:,k);
    
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
view(15,45) % Angulo de vista
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
psie= Angulo(psid-psi(1:end-1));
plot(hxe), hold on, grid on
plot(hye)
plot(hze)
plot(psie)
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


plot(uc(1,1:end))
hold on
plot(u(1,1:end))
hold on
plot(vref(1,1:end))
legend("ulc","ul","ul_{ref}")
ylabel('x [m/s]'); xlabel('s [ms]');
title ("Posiciones deseadas y reales del extremo operativo del manipulador aéreo")

figure(5)
plot(uc(2,1:end))
hold on
plot(u(2,1:end))
hold on
plot(vref(2,1:end))
legend("umc","um","um_{ref}")
ylabel('y [m/s]'); xlabel('s [ms]');

figure(6)
plot(uc(3,1:end))
hold on
plot(u(3,1:end))
hold on
plot(vref(3,1:end))
legend("unc","un","un_{ref}")
ylabel('z [m/ms]'); xlabel('s [ms]');

figure(7)
plot(uc(4,1:end))
hold on
plot(u(4,1:end))
hold on
plot(vref(4,1:end))
legend("wc","w","w_{ref}")
ylabel('psi [rad/s]'); xlabel('s [ms]');

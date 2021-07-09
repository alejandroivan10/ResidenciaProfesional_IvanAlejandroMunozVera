%{
Entrada: Arduino ax,ay,az,gx,gy,gz,mx,my,mz
Salida: Graficación de la orientación del sensor por medio del 
    filtro Madgwick
%}

clear all
close all
clc
%dbstop if naninf; %Instruccion DEBUG: "Si encuentras un NaN detente"
%dbclear all; %Borrar instrucciones para DEBUG

%borrar datos previos
delete(instrfind({'Port'},{'COM3'}));
%crear objeto serie y determinar salto de linea como final de dato
s = serial('COM3','BaudRate',115200,'Terminator','CR/LF');
warning('off','MATLAB:serial:fscanf:unsuccessfulRead');
fopen(s); %abrir puerto
pause(3);
xlabel('Eje x')
ylabel('Eje y')
zlabel('Eje z')
title('Acelerometro en tiempo real con Arduino')
ro = 1;
xlim ([-1.1*ro 1.1*ro])
ylim ([-1.1*ro 1.1*ro])
zlim ([-1.1*ro 1.1*ro])
grid on
hold on
set(gca, 'CameraPosition', [1.1*ro -1.1*ro 1.1*ro]);

%% System constants
deltat = 0.001; % sampling period in seconds (shown as 1 ms)
gyroMeasError = 3.14159265358979 * (5.0 / 180.0); % gyroscope measurement error in rad/s (shown as 5 deg/s)
gyroMeasDrift = 3.14159265358979 * (0.2 / 180.0); % gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
beta = sqrt(3.0 / 4.0) * gyroMeasError; % compute beta
zeta = sqrt(3.0 / 4.0) * gyroMeasDrift; % compute zeta
%  Global system variables
% a_x, a_y, a_z; % accelerometer measurements
% w_x, w_y, w_z; % gyroscope measurements in rad/s
% m_x, m_y, m_z; % magnetometer measurements
% SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; % estimated orientation quaternion elements with initial conditions
b_x = 19.47; b_z = -40; % reference direction of flux in earth frame
w_bx = 0;  w_by = 0; w_bz = 0; % estimate gyroscope biases error

%% Local system variables
%norm; % vector norm
%SEqDot_omega = zeros(1,4); % quaternion rate from gyroscopes elements
f = zeros(1,3); % objective function elements
%J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, % objective function Jacobian elements
%J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; %
SEqHatDot = zeros(1,4); % estimated direction of the gyroscope error
%w_err_x; w_err_y; w_err_z; % estimated direction of the gyroscope error (angular)
%h_x, h_y, h_z; % computed flux in the earth frame
% axulirary variables to avoid reapeated calcualtions
SEq = [1 0 0 0];


% inicializar
i = 1; j = 0; t = 0;

%%
% ejecutar bucle cronometrado
tic;  t1 = toc;
while i <= 2000
    % leer del puerto serie
    val = fscanf(s,'%f %f %f %f %f %f %f %f %f')';
    Ac(1)= val(1);  Ac(2)= val(2);  Ac(3)= val(3);
    Gy(1)= val(4);  Gy(2)= val(5);  Gy(3)= val(6);    
    Mg(1)= val(7);  Mg(2)= val(8);  Mg(3)= val(9);

    Mg_escale = [1.0434  1.0584  1.0125]; %BMX
    Mg_offset = [4.299 -2.627   3.177];   %BMX
    %Mg(1) = (Mg(1) - Mg_offset(1)) * Mg_escale(1);
    %Mg(2) = (Mg(2) - Mg_offset(2)) * Mg_escale(2);
    %Mg(3) = (Mg(3) - Mg_offset(3)) * Mg_escale(3);
    
    halfSEq(1) = 0.5 * SEq(1);
    halfSEq(2) = 0.5 * SEq(2);
    halfSEq(3) = 0.5 * SEq(3);
    halfSEq(4) = 0.5 * SEq(4);
    twoSEq(1) = 2.0 * SEq(1);
    twoSEq(2) = 2.0 * SEq(2);
    twoSEq(3) = 2.0 * SEq(3);
    twoSEq(4) = 2.0 * SEq(4);
    twob_x = 2.0 * b_x;
    twob_z = 2.0 * b_z;
    twob_xSEq(1) = 2.0 * b_x * SEq(1);
    twob_xSEq(2) = 2.0 * b_x * SEq(2);
    twob_xSEq(3) = 2.0 * b_x * SEq(3);
    twob_xSEq(4) = 2.0 * b_x * SEq(4);
    twob_zSEq(1) = 2.0 * b_z * SEq(1);
    twob_zSEq(2) = 2.0 * b_z * SEq(2);
    twob_zSEq(3) = 2.0 * b_z * SEq(3);
    twob_zSEq(4) = 2.0 * b_z * SEq(4);
    %SEq_1SEq_2;
    SEq_1SEq_3 = SEq(1) * SEq(3);
    %SEq_1SEq_4;
    %SEq_2SEq_3;
    SEq_2SEq_4 = SEq(2) * SEq(4);
    %SEq_3SEq_4;
    twom_x = 2.0 * Mg(1);
    twom_y = 2.0 * Mg(2);
    twom_z = 2.0 * Mg(3);

    % normalise the accelerometer measurement
    norm = sqrt(Ac(1)*Ac(1) + Ac(2)*Ac(2) + Ac(3)*Ac(3));
    Ac(1) = Ac(1)/norm;
    Ac(2) = Ac(2)/norm;
    Ac(3) = Ac(3)/norm;
    % normalise the magnetometer measurement
    norm = sqrt(Mg(1)*Mg(1) + Mg(2)*Mg(2) + Mg(3)*Mg(3));
    Mg(1) = Mg(1)/norm;
    Mg(2) = Mg(2)/norm;
    Mg(3) = Mg(3)/norm;
    fprintf('i=%d  ax=%.3f ay=%.3f az=%.3f   mx=%.3f my=%.3f mz=%.3f \n',...
        i,Ac(1),Ac(2),Ac(3),Mg(1),Mg(2),Mg(3));
    % compute the objective function and Jacobian
    f(1) = twoSEq(2) * SEq(4) - twoSEq(1) * SEq(3) - Ac(1);
    f(2) = twoSEq(1) * SEq(2) + twoSEq(3) * SEq(4) - Ac(2);
    f(3) = 1.0 - twoSEq(2) * SEq(2) - twoSEq(3) * SEq(3) - Ac(3);
    f(4) = twob_x * (0.5 - SEq(3) * SEq(3) - SEq(4) * SEq(4)) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - Mg(1);
    f(5) = twob_x * (SEq(2) * SEq(3) - SEq(1) * SEq(4)) + twob_z * (SEq(1) * SEq(2) + SEq(3) * SEq(4)) - Mg(2);
    f(6) = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5 - SEq(2) * SEq(2) - SEq(3) * SEq(3)) - Mg(3);
    f(4) = 0;  f(5) = 0;  f(6) = 0;
    J_11or24 = twoSEq(3); % J_11 negated in matrix multiplication
    J_12or23 = 2.0 * SEq(4);
    J_13or22 = twoSEq(1); % J_12 negated in matrix multiplication
    J_14or21 = twoSEq(2);
    J_32 = 2.0 * J_14or21; % negated in matrix multiplication
    J_33 = 2.0 * J_11or24; % negated in matrix multiplication
    J_41 = twob_zSEq(3); % negated in matrix multiplication
    J_42 = twob_zSEq(4);
    J_43 = 2.0 * twob_xSEq(3) + twob_zSEq(1); % negated in matrix multiplication
    J_44 = 2.0 * twob_xSEq(4) - twob_zSEq(2); % negated in matrix multiplication
    J_51 = twob_xSEq(4) - twob_zSEq(2); % negated in matrix multiplication
    J_52 = twob_xSEq(3) + twob_zSEq(1);
    J_53 = twob_xSEq(2) + twob_zSEq(4);
    J_54 = twob_xSEq(1) - twob_zSEq(3); % negated in matrix multiplication
    J_61 = twob_xSEq(3);
    J_62 = twob_xSEq(4) - 2.0 * twob_zSEq(2);
    J_63 = twob_xSEq(1) - 2.0 * twob_zSEq(3);
    J_64 = twob_xSEq(2);
    % compute the gradient (matrix multiplication)
    SEqHatDot(1) = J_14or21 * f(2) - J_11or24 * f(1) - J_41 * f(4) - J_51 * f(5) + J_61 * f(6);
    SEqHatDot(2) = J_12or23 * f(1) + J_13or22 * f(2) - J_32 * f(3) + J_42 * f(4) + J_52 * f(5) + J_62 * f(6);
    SEqHatDot(3) = J_12or23 * f(2) - J_33 * f(3) - J_13or22 * f(1) - J_43 * f(4) + J_53 * f(5) + J_63 * f(6);
    SEqHatDot(4) = J_14or21 * f(1) + J_11or24 * f(2) - J_44 * f(4) - J_54 * f(5) + J_64 * f(6);
    % normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot(1) * SEqHatDot(1) + SEqHatDot(2) * SEqHatDot(2) + SEqHatDot(3) * SEqHatDot(3) + SEqHatDot(4) * SEqHatDot(4));
    SEqHatDot(1) = SEqHatDot(1) / norm;
    SEqHatDot(2) = SEqHatDot(2) / norm;
    SEqHatDot(3) = SEqHatDot(3) / norm;
    SEqHatDot(4) = SEqHatDot(4) / norm;
    % compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq(1) * SEqHatDot(2) - twoSEq(2) * SEqHatDot(1) - twoSEq(3) * SEqHatDot(4) + twoSEq(4) * SEqHatDot(3);
    w_err_y = twoSEq(1) * SEqHatDot(3) + twoSEq(2) * SEqHatDot(4) - twoSEq(3) * SEqHatDot(1) - twoSEq(4) * SEqHatDot(2);
    w_err_z = twoSEq(1) * SEqHatDot(4) - twoSEq(2) * SEqHatDot(3) + twoSEq(3) * SEqHatDot(2) - twoSEq(4) * SEqHatDot(1);
    % compute and remove the gyroscope baises
    deltat = toc - t1; %0.150 seg aprox.
    t1 = toc;
    w_bx = w_bx + w_err_x * deltat * zeta;
    w_by = w_by + w_err_y * deltat * zeta;
    w_bz = w_bz + w_err_z * deltat * zeta;
    w_bx=0;  w_by=0;  w_bz=0;
    Gy(1) = Gy(1) - w_bx;
    Gy(2) = Gy(2) - w_by;
    Gy(3) = Gy(3) - w_bz;
    % compute the quaternion rate measured by gyroscopes
    SEqDot_omega(1) = -halfSEq(2) * Gy(1) - halfSEq(3) * Gy(2) - halfSEq(4) * Gy(3);
    SEqDot_omega(2) = halfSEq(1)  * Gy(1) + halfSEq(3) * Gy(3) - halfSEq(4) * Gy(2);
    SEqDot_omega(3) = halfSEq(1)  * Gy(2) - halfSEq(2) * Gy(3) + halfSEq(4) * Gy(1);
    SEqDot_omega(4) = halfSEq(1)  * Gy(3) + halfSEq(2) * Gy(2) - halfSEq(3) * Gy(1);
    % compute then integrate the estimated quaternion rate
    SEq(1) = SEq(1) + (SEqDot_omega(1) - (beta * SEqHatDot(1))) * deltat;
    SEq(2) = SEq(2) + (SEqDot_omega(2) - (beta * SEqHatDot(2))) * deltat;
    SEq(3) = SEq(3) + (SEqDot_omega(3) - (beta * SEqHatDot(3))) * deltat;
    SEq(4) = SEq(4) + (SEqDot_omega(4) - (beta * SEqHatDot(4))) * deltat;
    % normalise quaternion
    norm = sqrt(SEq(1) * SEq(1) + SEq(2) * SEq(2) + SEq(3) * SEq(3) + SEq(4) * SEq(4));
    SEq(1) = SEq(1) / norm;
    SEq(2) = SEq(2) / norm;
    SEq(3) = SEq(3) / norm;
    SEq(4) = SEq(4) / norm;
    % compute flux in the earth frame
    SEq_1SEq_2 = SEq(1) * SEq(2); % recompute axulirary variables
    SEq_1SEq_3 = SEq(1) * SEq(3);
    SEq_1SEq_4 = SEq(1) * SEq(4);
    SEq_3SEq_4 = SEq(3) * SEq(4);
    SEq_2SEq_3 = SEq(2) * SEq(3);
    SEq_2SEq_4 = SEq(2) * SEq(4);
    
    h_x = twom_x * (0.5 - SEq(3) * SEq(3) - SEq(4) * SEq(4)) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5 - SEq(2) * SEq(2) - SEq(4) * SEq(4)) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5 - SEq(2) * SEq(2) - SEq(3) * SEq(3));
    % normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
    fprintf('----bx=%.3f bz=%.3f   w_bx=%.3f w_by=%.3f w_bz=%.3f \n',...
        b_x,b_z,w_bx,w_by,w_bz);
    disp(SEqDot_omega);

    
    
    %% ORIENTACION Y GRÁFICA EN PANTALLA
    % ORIENTACION
    SErotM = Quat2RotMat(SEq);
    
    % GRAFICA EN PANTALLA
    N=plot3([0;SErotM(1,1)],[0;SErotM(2,1)],[0;SErotM(3,1)],'Color','r','LineWidth',2);
    Nt=text(SErotM(1,1),SErotM(2,1),SErotM(3,1), 'X', 'Rotation',+15);
    O=plot3([0;SErotM(1,2)],[0;SErotM(2,2)],[0;SErotM(3,2)],'Color','g','LineWidth',2);
    Ot=text(SErotM(1,2),SErotM(2,2),SErotM(3,2), 'Y', 'Rotation',+15);
    A=plot3([0;SErotM(1,3)],[0;SErotM(2,3)],[0;SErotM(3,3)],'Color','b','LineWidth',2);
    At=text(SErotM(1,3),SErotM(2,3),SErotM(3,3), 'Z', 'Rotation',+15);
    
%     %NORTE MÁGNETICO
%     NM=plot3([X;X+NMx],[Y;Y+NMy],[Z;Z+NMz],'Color','y','LineWidth',2);
%     NMt=text(X,Y,Z, 'Magnetic', 'Rotation',+15);
%     %AVERAGE
%     angle = acosd( dot(Ac,Mg) / (norm(Ac)*norm(Mg)) );
%     av = (Ac + Mg) / 2;
%     Avt=text(av(1),av(2),av(3), num2str(angle), 'Rotation',+15);
    %eje x
    h1=plot3([0;1],[0;0],[0;0],'Color','r','LineWidth',2);
    h1t=text(ro,0,0, 'Eje X', 'Rotation',+15);
    % %eje y
    h2=plot3([0;0],[0;1],[0;0],'Color','g','LineWidth',2);
    h2t=text(0,ro,0, 'Eje Y', 'Rotation',+15);
    % %eje z
    h3=plot3([0;0],[0;0],[0;1],'Color','b','LineWidth',2);
    h3t=text(0,0,ro, 'Eje Z', 'Rotation',+15);
    
    drawnow
    %refreshdata
    i = i+1;
    
    %pause(0.03)
    delete(N); delete(Nt);
    delete(O); delete(Ot);
    delete(A); delete(At);
    delete(h1); delete(h1t);
    delete(h2); delete(h2t);
    delete(h3); delete(h3t);
    
    
    refresh
    %t = toc;
    %i = i+1;
end

%% FIN DEL PROGRAMA
clc;
fclose(s);
delete(s);
clear s;


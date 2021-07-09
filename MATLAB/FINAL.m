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
beta = sqrt(2.0 / 4.0) * gyroMeasError; % compute beta
zeta = sqrt(3.0 / 4.0) * gyroMeasDrift; % compute zeta
%  Global system variables
% a_x, a_y, a_z; % accelerometer measurements
% w_x, w_y, w_z; % gyroscope measurements in rad/s
% m_x, m_y, m_z; % magnetometer measurements
% SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; % estimated orientation quaternion elements with initial conditions
b_x = 0.454; b_y = -0.189; b_z = -0.871; % reference direction of flux in earth frame
b_E = [b_x b_y b_z]; 
b_E=b_E./norm(b_E);
g_E = [0 0 1];
w_bx = 0;  w_by = 0; w_bz = 0; % estimate gyroscope biases error

%% Local system variables
%magn; % vector magn
%SEqDot_omega = zeros(1,4); % quaternion rate from gyroscopes elements
f(:,1) = zeros(1,6); % objective function elements
%J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, % objective function Jacobian elements
%J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; %
SEqHatDot(1,:) = zeros(1,4); % estimated direction of the gyroscope error
%w_err(1); w_err(2); w_err(3); % estimated direction of the gyroscope error (angular)
%h_S(1), h_S(2), h_S(3); % computed flux in the earth frame
% axulirary variables to avoid reapeated calcualtions
SEq = [1 0 0 0];
SErotM = Quat2RotMat(SEq)';


%% inicializar CALIBRACIÓN
i_capt=1; % Index de captura
nCapt = 20; % #Num de Valores por capturar
Ac_Tol = 1.08; % Tolerancia de Error Total Accel
Mg_Tol = 1.08; % Tolerancia de Error Total Magnet
Ac_capt = ones(nCapt,3); %It can't be zeros(), because make "inf" values
Mg_capt = ones(nCapt,3); %Same comment of "Ac_capt"
Gy_capt = ones(nCapt,3);
Ac_ant = 0.1*[1 1 1]; %Same comment of "Ac_capt"
Gy_ant = 0.1*[1 1 1];
Mg_ant = 0.1*[1 1 1]; %Same comment of "Ac_capt"
nCalib = 2;
d_Ac = ones(1,nCalib); %It can't be zeros()
d_Mg = ones(1,nCalib);
Ac_Adj = [9.5669    8.8451   10.6146    0.0165    0.7612    1.6092]; %Acel_Calib
Ac_Adj(1:3) =  Ac_Adj(1)./Ac_Adj(1:3);
Gy_Adj = [-1.3005    0.0240   -0.6723]; % Gyro_Calib
Mg_Adj = [39.6496   43.5980   40.8667   -5.3843   10.1385  -12.7517]; %Mag_Calib
Mg_Adj(1:3) =  Mg_Adj(1)./Mg_Adj(1:3);

%% Ejecutar bucle infinito
%tic;  t1 = toc;
i=0;  j=0;  t=0;
t2 = 0;

while 1
    % leer del puerto serie
    val = fscanf(s,'Ac=[%f %f %f] Gy=[%f %f %f] Mg=[%f %f %f] %d')';
    Ac(1)= val(1);  Ac(2)= val(2);  Ac(3)= val(3);
    Gy(1)= val(4);  Gy(2)= val(5);  Gy(3)= val(6);    
    Mg(1)= val(7);  Mg(2)= val(8);  Mg(3)= val(9);
    time_sensor = val(10)/1e6;
    
    t1 = t2;   t2 = time_sensor;   deltat = t2-t1;
    Ac_Raw = Ac;    Mg_Raw = Mg;     Gy_Raw = Gy;
    Ac = (Ac - Ac_Adj(4:6)) .* Ac_Adj(1:3);
    Mg = (Mg - Mg_Adj(4:6)) .* Mg_Adj(1:3);
    Gy =  Gy - Gy_Adj(1:3);    
    
   
    %% OBTENCIÓN DE LA ORIENTACIÓN POR MADGWICK
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
    two_m(1) = 2.0 * Mg(1);
    two_m(2) = 2.0 * Mg(2);
    two_m(3) = 2.0 * Mg(3);
    
    % magnalise the accelerometer measurement
    magn = sqrt(Ac(1)*Ac(1) + Ac(2)*Ac(2) + Ac(3)*Ac(3));
    Ac_one = Ac./magn;
    % magnalise the magnetometer measurement
    magn = sqrt(Mg(1)*Mg(1) + Mg(2)*Mg(2) + Mg(3)*Mg(3));
    Mg_one = Mg./magn;
    Gy = Gy.*(pi/180); %Conersion entre (Deg/seg) a (Rad/seg)
    
    fprintf('i=%d Ac=[%.3f %.3f %.3f]  Mg=[%.3f %.3f %.3f] Gy=[%.3f %.3f %.3f]\n',...
        i+1,Ac_one(1),Ac_one(2),Ac_one(3),Mg_one(1),Mg_one(2),Mg_one(3),Gy(1),Gy(2),Gy(3));
    % compute the objective function and Jacobian
    f(1:3,1) = SErotM * g_E' - Ac_one';
    f(4:6,1) = SErotM * b_E' - Mg_one';
    %f(1:3,1) = [0 0 0];
    % compute the gradient (matrix multiplication)
    J(2,4) = twoSEq(3);  J(1,1) = -J(2,4);% J_11 negated in matrix multiplication
    J(1,2) = twoSEq(4);  J(2,3) =  J(1,2);
    J(2,2) = twoSEq(1);  J(1,3) = -J(2,2); % J_12 negated in matrix multiplication
    J(1,4) = twoSEq(2);  J(2,1) =  J(1,4);
    J(3,2) = -2.0 * twoSEq(2); % negated in matrix multiplication
    J(3,3) = -2.0 * twoSEq(3); % negated in matrix multiplication
    J(4,1) = -twob_zSEq(3); % negated in matrix multiplication
    J(4,2) =  twob_zSEq(4);
    J(4,3) = -2.0 * twob_xSEq(3) - twob_zSEq(1); % negated in matrix multiplication
    J(4,4) = -2.0 * twob_xSEq(4) + twob_zSEq(2); % negated in matrix multiplication
    J(5,1) = -twob_xSEq(4) + twob_zSEq(2); % negated in matrix multiplication
    J(5,2) =  twob_xSEq(3) + twob_zSEq(1);
    J(5,3) =  twob_xSEq(2) + twob_zSEq(4);
    J(5,4) = -twob_xSEq(1) + twob_zSEq(3); % negated in matrix multiplication
    J(6,1) = twob_xSEq(3);
    J(6,2) = twob_xSEq(4) - 2.0 * twob_zSEq(2);
    J(6,3) = twob_xSEq(1) - 2.0 * twob_zSEq(3);
    J(6,4) = twob_xSEq(2);
    SEqHatDot = f'*J;
    
    % magnalise the gradient to estimate direction of the gyroscope error
    magn = sqrt(SEqHatDot(1) * SEqHatDot(1) + SEqHatDot(2) * SEqHatDot(2) + SEqHatDot(3) * SEqHatDot(3) + SEqHatDot(4) * SEqHatDot(4));
    SEqHatDot = SEqHatDot ./ magn;
    % compute angular estimated direction of the gyroscope error
    err = 2.*quatmultiply(quatconj(SEq), SEqHatDot);
    w_err = err(2:4);
    % compute and remove the gyroscope baises
    %{
    deltat = toc - t1; %0.150 seg aprox.
    %deltat = t2 - t1;
    %t1 = toc;
    %}
    w_bx = w_bx + w_err(1) * deltat * zeta;
    w_by = w_by + w_err(2) * deltat * zeta;
    w_bz = w_bz + w_err(3) * deltat * zeta;
    w_bx = 0;  w_by = 0;  w_bz = 0;
    Gy(1) = Gy(1) - w_bx;
    Gy(2) = Gy(2) - w_by;
    Gy(3) = Gy(3) - w_bz;
    % compute the quaternion rate measured by gyroscopes
    SEqDot_omega = 0.5.*quatmultiply(SEq, [0 Gy]);
    %SEqDot_omega = [0 0 0 0];
    % compute then integrate the estimated quaternion rate
    SEq(1) = SEq(1) + (SEqDot_omega(1) - (beta * SEqHatDot(1))) * deltat;
    SEq(2) = SEq(2) + (SEqDot_omega(2) - (beta * SEqHatDot(2))) * deltat;
    SEq(3) = SEq(3) + (SEqDot_omega(3) - (beta * SEqHatDot(3))) * deltat;
    SEq(4) = SEq(4) + (SEqDot_omega(4) - (beta * SEqHatDot(4))) * deltat;
    % magnalise quaternion
    magn = sqrt(SEq(1) * SEq(1) + SEq(2) * SEq(2) + SEq(3) * SEq(3) + SEq(4) * SEq(4));
    SEq(1) = SEq(1) / magn;
    SEq(2) = SEq(2) / magn;
    SEq(3) = SEq(3) / magn;
    SEq(4) = SEq(4) / magn;
    % compute flux in the earth frame
    SEq_1SEq_2 = SEq(1) * SEq(2); % recompute axulirary variables
    SEq_1SEq_3 = SEq(1) * SEq(3);
    SEq_1SEq_4 = SEq(1) * SEq(4);
    SEq_3SEq_4 = SEq(3) * SEq(4);
    SEq_2SEq_3 = SEq(2) * SEq(3);
    SEq_2SEq_4 = SEq(2) * SEq(4);
    
    SErotM = Quat2RotMat(SEq);
    h_S = (SErotM * two_m')';
    SErotM = SErotM';
    
    % magnalise the flux vector to have only components in the x and z
%     b_x = sqrt((h_S(1) * h_S(1)) + (h_S(2) * h_S(2)));
%     b_z = h_S(3);
    %fprintf('----bx=%.3f bz=%.3f   w_bx=%.3f w_by=%.3f w_bz=%.3f \n', b_x,b_z,w_bx,w_by,w_bz);
    
    %% CALIBRACIÓN ELÍPTICA AUTOMÁTICA
    btw_angle = dot(Ac,Ac_capt(i_capt,:))/(norm(Ac)*norm(Ac_capt(i_capt,:))); % Angle Between measurements
        if_btw_angAc = btw_angle < 0.9659;  % btw_angle < que 15°
    btw_angle = dot(Mg,Mg_capt(i_capt,:))/(norm(Mg)*norm(Mg_capt(i_capt,:))); % Angle Between measurements
        if_btw_angMg = btw_angle < 0.9659;  % btw_angle < que 15°
    d_Ac(mod(i,nCalib)+1) = norm(Ac-Ac_ant)/norm(Ac_ant);
    d_Mg(mod(i,nCalib)+1) = norm(Mg-Mg_ant)/norm(Mg_ant);
        No_Move_Ac_Mg = (sum(d_Ac)/nCalib) < 0.05  &  (sum(d_Mg)/nCalib) < 0.05;
    %fprintf('Mg_ant= [%.3f  %.3f  %.3f]   Mg= [%.3f  %.3f  %.3f]\n', Mg, Mg_ant);
    %fprintf('Ac_ant= [%.3f  %.3f  %.3f]   Ac= [%.3f  %.3f  %.3f]\n', Ac, Ac_ant);
    %fprintf('btw_angle= %.3f  d_Ac= %.3f   d_Mg= %.3f \n', btw_angle, sum(d_Ac)/nCalib, sum(d_Mg)/nCalib);
    %NoMove_Gy = sum(Gy.^2) < 4; % Gy < 4 [°/seg]
    
    % Proceso de validación y posible captura de datos
    Ac_ant = Ac;
    Gy_ant = Gy;
    Mg_ant = Mg;
    %bol = if_btw_ang & No_Move_Ac_Mg
    if  No_Move_Ac_Mg     %if true, entonces captura datos
        if if_btw_angMg & if_btw_angAc
            i_capt = mod(j,nCapt)+1;
            Ac_capt(i_capt,:) = Ac;
            Gy_capt(i_capt,:) = Gy;
            Mg_capt(i_capt,:) = Mg;
            if i_capt == nCapt
                Ac_Adj_new = RegresiLinealMultip(Ac_capt);
                Mg_Adj_new = RegresiLinealMultip(Mg_capt);
                Gy_Adj_new = sum(Gy_capt,1)/nCapt;
                
                if Ac_Adj_new ~= -1 % Si es diferente de -1
                    Ac_TamH_new = Ac_Adj_new;
                    % Proceso de aprobación/Rechazo de los nuevos parámetros de ajuste
                    Ac_AdjError = 1 - sum(( (Ac_capt-Ac_Adj_new(4:6))./Ac_Adj_new(1:3) ).^2 , 2); % Ac_AdjError = 1 - ((x-h)/m)^2 - ((y-h)/n)^2 - ((z-h)/o)^2                    
                    Ac_AdjRel = sqrt(1./(1-Ac_AdjError));  % Rel = (1/(1-Error))^0.5                    
                        if_Ac_inv = 0 < Ac_AdjRel & Ac_AdjRel < 1;                    
                    Ac_AdjRel = (~if_Ac_inv).*Ac_AdjRel + if_Ac_inv./Ac_AdjRel;                    
                    Ac_Adj_TotalErr = sum(Ac_AdjRel,1) / size(Ac_capt,1);
                    fprintf('AcTol_Err = %.3f  \n',Ac_Adj_TotalErr); 
                    if Ac_Adj_TotalErr < Ac_Tol
                        Ac_Tol = 0.7*(Ac_Tol) + 0.3*(1.08);  %  = Ac_Tol - 0.4*(Ac_Tol-1.08)
                        % If TRUE, enonces son parámetros validos, y los actualizamos
                        Ac_Adj_new(1:3) =  Ac_Adj_new(1)./Ac_Adj_new(1:3);                        
                        Ac_Adj_new(4:6) = Ac_Adj(4:6) + Ac_Adj_new(4:6)./Ac_Adj(1:3);
                        Ac_Adj_new(1:3) = Ac_Adj(1:3).*Ac_Adj_new(1:3);                        
%                         i_adj = mod(k,10)+1;
%                         Ac_Adj_new(4:6) = Ac_Adj(4:6) + Ac_Adj_new(4:6).*Ac_Adj(1:3);
%                         Ac_Adj_new(1:3) = Ac_Adj(1:3).*Ac_Adj_new(1:3);
%                         Mg_Adj_new(4:6) = Mg_Adj(4:6) + Mg_Adj_new(4:6).*Mg_Adj(1:3);
%                         Mg_Adj_new(1:3) = Mg_Adj(1:3).*Mg_Adj_new(1:3);
%                         Ac_Adj_Store(i_adj,:) = Ac_Adj; %%Ac_Adjust_Storage
%                         Mg_Adj_Store(i_adj,:) = Mg_Adj;
                        
                        % Se mezclan los viejos y los nuevos parametros con 60% y 40% c/u
                        Ac_Adj = 0.6*Ac_Adj + 0.4*Ac_Adj_new                        
                        fprintf('Ac_Adj =');  fprintf('   %.3f', Ac_TamH_new(1)./Ac_Adj(1:3));
                            fprintf('   %.3f', Ac_Adj(4:6));  fprintf('\n');           
                        pause(5);
                    else
                        fprintf("ACC: Test de TOLERANCIA de ajuste REPROBADO\n");
                        Ac_Tol = Ac_Tol * 1.08;
                        pause(2);
                    end
                else
                    fprintf("ACCEL: AMPL Y OFFS NEGATIVOS\n");
                    pause(2);
                    return;
                end
                
                if Mg_Adj_new ~= -1 % Si es diferente de -1                    
                    Mg_TamH_new = Mg_Adj_new;
                    % Proceso de aprobación/Rechazo de los nuevos parámetros de ajuste                    
                    Mg_AdjError = 1 - sum(( (Mg_capt-Mg_Adj_new(4:6))./Mg_Adj_new(1:3) ).^2 , 2);                    
                    Mg_AdjRel = sqrt(1./(1-Mg_AdjError));                    
                        if_Mg_inv = 0 < Mg_AdjRel & Mg_AdjRel < 1;                    
                    Mg_AdjRel = (~if_Mg_inv).*Mg_AdjRel + if_Mg_inv./Mg_AdjRel;
                    Mg_Adj_TotalErr = sum(Mg_AdjRel,1) / size(Mg_capt,1);
                    fprintf('MgTol_Err = %.3f\n\n',Mg_Adj_TotalErr);
                    if Mg_Adj_TotalErr < Mg_Tol
                        Mg_Tol = 0.7*(Mg_Tol) + 0.3*(1.08);  %  = Mg_Tol - 0.4*(Mg_Tol-1.08)
                        % If TRUE, enonces son parámetros validos, y los actualizamos                        
                        Mg_Adj_new(1:3) =  Mg_Adj_new(1)./Mg_Adj_new(1:3);                                              
                        Mg_Adj_new(4:6) = Mg_Adj(4:6) + Mg_Adj_new(4:6)./Mg_Adj(1:3);
                        Mg_Adj_new(1:3) = Mg_Adj(1:3).*Mg_Adj_new(1:3);
                        
                        % Se mezclan los viejos y los nuevos parametros con 60% y 40% c/u                        
                        Mg_Adj = 0.6*Mg_Adj + 0.4*Mg_Adj_new                        
                        fprintf('Mg_Adj =');  fprintf('   %.3f', Mg_TamH_new(1)./Mg_Adj(1:3));
                            fprintf('   %.3f', Mg_Adj(4:6));  fprintf('\n');                        
                        pause(20);
                    else
                        fprintf("MAG: Test de TOLERANCIA de ajuste REPROBADO\n");
                        Mg_Tol = Mg_Tol * 1.08;
                        pause(2);
                    end
                else
                    fprintf("MAGNET: AMPL Y OFFS NEGATIVOS\n");
                    pause(2);
                    return;
                end               
                
            end
            j = j + 1;
            fprintf('%d  VALOR(ES) CAPTURADO(S) \n',j);
        end
    end
    
    
    %% ORIENTACION Y GRÁFICA EN PANTALLA
    % ORIENTACION
    
    % GRAFICA EN PANTALLA
    N=plot3([0;SErotM(1,1)],[0;SErotM(1,2)],[0;SErotM(1,3)],'Color','r','LineWidth',2);
    Nt=text(SErotM(1,1),SErotM(1,2),SErotM(1,3), 'X', 'Rotation',+15);
    O=plot3([0;SErotM(2,1)],[0;SErotM(2,2)],[0;SErotM(2,3)],'Color','g','LineWidth',2);
    Ot=text(SErotM(2,1),SErotM(2,2),SErotM(2,3), 'Y', 'Rotation',+15);
    A=plot3([0;SErotM(3,1)],[0;SErotM(3,2)],[0;SErotM(3,3)],'Color','b','LineWidth',2);
    At=text(SErotM(3,1),SErotM(3,2),SErotM(3,3), 'Z', 'Rotation',+15);
    
%     %NORTE MÁGNETICO
%     NM=plot3([X;X+NMx],[Y;Y+NMy],[Z;Z+NMz],'Color','y','LineWidth',2);
%     NMt=text(X,Y,Z, 'Magnetic', 'Rotation',+15);
%     %AVERAGE
%     angle = acosd( dot(Ac,Mg) / (magn(Ac)*magn(Mg)) );
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
    
    %pause(0.03)
    delete(N); delete(Nt);
    delete(O); delete(Ot);
    delete(A); delete(At);
    delete(h1); delete(h1t);
    delete(h2); delete(h2t);
    delete(h3); delete(h3t);
    
    
    refresh
    %t = toc;
    i = i+1;
end

%% FIN DEL PROGRAMA
fclose(s);
delete(s);
clear s;


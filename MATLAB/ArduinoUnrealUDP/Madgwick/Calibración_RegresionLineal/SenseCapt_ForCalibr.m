%{
Entrada: Arduino ax,ay,az,gx,gy,gz,mx,my,mz
Salida: Graficación de la orientación del sensor por medio del 
    filtro Madgwick
%}

clear all
close all
clc

nVal = 20;
%borrar datos previos
delete(instrfind({'Port'},{'COM3'}));
%crear objeto serie y determinar salto de linea como final de dato
s = serial('COM3','BaudRate',115200,'Terminator','CR/LF');
warning('off','MATLAB:serial:fscanf:unsuccessfulRead');
%abrir puerto
fopen(s);


i = 0; j = 0; pos = 1;
Ac_capt = ones(20,3); %It can't be zeros(), because make "inf" values
Mg_capt = ones(20,3); %Same comment of "Ac_capt"
Gy_capt = ones(20,3);
Ac_ant = 0.1*[1 1 1]; %Same comment of "Ac_capt"
Gy_ant = 0.1*[1 1 1];
Mg_ant = 0.1*[1 1 1]; %Same comment of "Ac_capt"
d_Ac = ones(1,5); %It can't be zeros()
d_Mg = ones(1,5);
while  1 % < nVal
    % leer del puerto serie
    val = fscanf(s,'Ac=[%f %f %f] Gy=[%f %f %f] Mg=[%f %f %f] %d')';
    ax=val(1);    ay=val(2);    az=val(3);
    gx=val(4);    gy=val(5);    gz=val(6);    
    mx=val(7);    my=val(8);    mz=val(9);
    Ac = [ax ay az];
    Gy = [gx gy gz];
    Mg = [mx my mz];
    %fprintf('i=%d  Ac=[%.3f %.3f %.3f]  Mg=[%.3f %.3f %.3f]\n',...
    %    i,ax,ay,az,mx,my,mz);
    
    btw_angle = dot(Ac,Ac_capt(pos,:))/(norm(Ac)*norm(Ac_capt(pos,:))); % Angle Between measurements
        if_btw_angAc = btw_angle < 0.9659;  % btw_angle < que 15°
    btw_angle = dot(Mg,Mg_capt(pos,:))/(norm(Mg)*norm(Mg_capt(pos,:))); % Angle Between measurements
        if_btw_angMg = btw_angle < 0.9659;  % btw_angle < que 15°
    d_Ac(mod(i,5)+1) = norm(Ac-Ac_ant)/norm(Ac_ant);
    d_Mg(mod(i,5)+1) = norm(Mg-Mg_ant)/norm(Mg_ant);
        No_Move_Ac_Mg = (sum(d_Ac)/5) < 0.05  &  (sum(d_Mg)/5) < 0.05;
    %fprintf('Mg_ant= [%.3f  %.3f  %.3f]   Mg= [%.3f  %.3f  %.3f]\n', Mg, Mg_ant);
    %fprintf('Ac_ant= [%.3f  %.3f  %.3f]   Ac= [%.3f  %.3f  %.3f]\n', Ac, Ac_ant);
    fprintf('btw_angle= %.3f  d_Ac= %.3f   d_Mg= %.3f \n', btw_angle, sum(d_Ac)/5, sum(d_Mg)/5);
    %NoMove_Gy = sum(Gy.^2) < 4; % Gy < 4 [°/seg]
    
    % Proceso de validación y posible captura de datos
    Ac_ant = Ac;
    Gy_ant = Gy;
    Mg_ant = Mg;
    %bol = if_btw_ang & No_Move_Ac_Mg
    if  No_Move_Ac_Mg     %if true, entonces captura datos
        if if_btw_angMg & if_btw_angAc
            pos = mod(j,20)+1;
            Ac_capt(pos,:) = Ac;
            Gy_capt(pos,:) = Gy;
            Mg_capt(pos,:) = Mg;
            j = j + 1;
            fprintf('%d  VALOR(ES) CAPTURADO(S) \n',j); pause(1);
            if pos == 20
                Ac_correct = RegresiLinealMultip(Ac_capt)
                Mg_correct = RegresiLinealMultip(Mg_capt)
                Gy_correct = -sum(Gy_capt,1)/20
            end
        end
    end
    i = i + 1;
end

%Correct = RegresiLinealMultip()


                %BOSCH BMX055 (Diciembre 2020)
%   Mg_escale = [1.0434  1.0584  1.0125];
%   Mg_offset = [4.299 -2.627   3.177];
%x =[1.330 12.190 4.290 -1.770 -7.980 -12.920 -18.230 -22.820 -26.870 -28.880 -28.900 -28.170 -26.120 -22.740 -16.710 -11.130 -4.780 4.050 -6.570 -17.850 -21.770 -22.530 -22.360 -21.750 -18.190 -12.990 -4.220 4.870 14.650 20.130 12.100 0.780 -11.740 -20.800 -25.610 -28.830 -30.480 -27.430 -20.850 -10.090 0.830 11.900 19.610 26.260 31.430 29.630 20.200 11.350 2.430 -7.440 -16.810 -20.630 -21.510 -21.320 -20.640 -15.080 -7.930 1.280 8.970 11.660 8.920]
%y =[-2.570 -34.420 -36.620 -34.700 -31.670 -27.470 -19.940 -10.730 -2.820 6.040 11.200 13.990 15.770 11.410 2.730 -3.920 -11.920 -20.480 -24.650 -17.490 -10.310 -6.370 -2.170 1.540 6.670 9.580 5.570 -1.580 -7.610 -15.420 -22.700 -25.700 -25.380 -22.760 -20.220 -16.770 -10.730 -4.830 1.660 6.560 9.790 8.810 0.350 7.510 13.110 20.220 27.360 28.620 24.980 19.480 9.980 0.980 -8.580 -17.700 -25.540 -33.440 -35.960 -35.690 -32.470 -23.820 -16.700]
%z =[2.960 21.810 9.820 -1.630 -12.480 -19.930 -22.920 -24.610 -19.460 -14.750 -6.100 2.690 12.790 20.560 27.570 32.590 36.160 36.720 30.490 19.800 7.570 1.350 -4.740 -10.300 -19.700 -27.860 -31.670 -31.810 -28.200 -23.290 -22.340 -23.710 -21.100 -12.470 -4.180 4.540 14.380 22.630 28.910 34.160 38.190 38.400 37.530 31.150 21.730 13.630 13.230 18.780 24.610 28.540 29.500 27.400 24.080 19.730 14.620 5.860 -1.930 -8.690 -15.120 -23.190 -28.560]

                %INVENSENSE (Febrero 14 2021) -> 20 valores
    %[9.5669    8.8451   10.6146    0.0165    0.7612    1.6092];
%Ac      = [6.110; 1.320; -5.730; -8.890; 3.350; -4.470; -4.580; -5.290; 2.890; 5.760; 1.800; -6.780; -4.050; 6.780; -3.530; -8.620; 1.950; -3.710; -9.130; 7.760];
%Ac(:,2) = [7.600; 0.340; -3.930; 2.810; 8.180; -0.560; 8.290; 7.560; 0.670; 7.680; 8.960; 6.800; -4.830; 5.570; -4.860; 2.680; 8.680; -6.580; -0.410; 5.060];
%Ac(:,3) = [1.850; 12.160; 9.390; 5.150; 6.350; -6.320; 0.730; 5.200; 11.780; 3.820; 3.840; 1.640; 7.570; 4.740; 7.840; 3.600; -4.130; 6.250; -3.120; 5.350];
    %[1.2225    0.0145    0.6495];
%Gy      = [0.330; -2.460; -3.280; -3.140; -2.800; -2.930; -3.050; -2.040; -2.540; -3.640; 1.850; 0.340; -0.220; 1.190; 0.760; 0.460; -0.340; 0.010; 0.200; -3.150];
%Gy(:,2) = [0.420; 0.060; -0.290; 0.700; 0.840; -0.260; -0.970; -0.020; -0.740; 0.220; -0.510; 1.300; 1.520; -2.630; 0.730; 0.940; 0.930; 0.890; -0.890; -2.530];
%Gy(:,3) = [-0.160; -1.640; -0.640; -0.450; -0.610; -0.260; -1.190; -0.430; -1.780; -3.580; 0.050; -0.130; 0.110; 2.930; 0.310; -0.010; -3.230; 1.070; -1.080; -2.270];
    %[39.6496   43.5980   40.8667   -5.3843   10.1385  -12.7517];
%Mg      = [-9.890; 0.480; 27.880; 25.630; -15.330; 7.280; 13.600; 0.480; 1.040; -35.400; -10.960; 4.490; 11.200; -26.550; 5.410; 23.080; -8.900; 10.690; 28.370; -23.370];
%Mg(:,2) = [-33.170; -10.120; 4.380; -13.410; -31.740; 26.290; -16.160; -34.280; -5.620; -13.410; -31.260; -28.290; 7.390; -27.690; 6.370; -19.640; -12.100; 14.350; -11.290; -29.430];
%Mg(:,3) = [-13.500; -48.300; -34.860; -2.910; -6.420; 23.240; 14.890; -16.870; -51.440; -0.200; -4.640; -6.640; -48.710; -12.090; -52.230; -18.190; 21.390; -48.910; -1.980; -15.330];



fclose(s);
delete(s);
clear s;
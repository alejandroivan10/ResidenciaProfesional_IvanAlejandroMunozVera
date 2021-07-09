clear all
close all
clc

%Creación de datos ficticios (magnétometro)
nVal = 200;
m = 45; n = 40; o = 35; 
ehx = -20; ehy = 20; ehz = 10;  % Introducir error OFFSET en la 'data'

data(:,1) = (rand(nVal,1)-0.5) * m*2; % rand() -> Valores entre 0 y 1
data(:,2) = (rand(nVal,1)-0.5).*sqrt(1 - (data(:,1)/m).^2) * n*2;
data(:,3) = sqrt(1 - (data(:,1)/m).^2 - (data(:,2)/n).^2) * o;
answ = rand(nVal,1) >= 0.5;
data(:,3) = ((-1).^answ).*data(:,3);
%comprobar = 1 - (data(:,1)/45).^2 - (data(:,2)/40).^2 - (data(:,3)/35).^2
data(:,1) = data(:,1) + ehx;
data(:,2) = data(:,2) + ehy;
data(:,3) = data(:,3) + ehz;
x = data(:,1);
y = data(:,2);
z = data(:,3);
x =[1.330 12.190 4.290 -1.770 -7.980 -12.920 -18.230 -22.820 -26.870 -28.880 -28.900 -28.170 -26.120 -22.740 -16.710 -11.130 -4.780 4.050 -6.570 -17.850 -21.770 -22.530 -22.360 -21.750 -18.190 -12.990 -4.220 4.870 14.650 20.130 12.100 0.780 -11.740 -20.800 -25.610 -28.830 -30.480 -27.430 -20.850 -10.090 0.830 11.900 19.610 26.260 31.430 29.630 20.200 11.350 2.430 -7.440 -16.810 -20.630 -21.510 -21.320 -20.640 -15.080 -7.930 1.280 8.970 11.660 8.920]';
y =[-2.570 -34.420 -36.620 -34.700 -31.670 -27.470 -19.940 -10.730 -2.820 6.040 11.200 13.990 15.770 11.410 2.730 -3.920 -11.920 -20.480 -24.650 -17.490 -10.310 -6.370 -2.170 1.540 6.670 9.580 5.570 -1.580 -7.610 -15.420 -22.700 -25.700 -25.380 -22.760 -20.220 -16.770 -10.730 -4.830 1.660 6.560 9.790 8.810 0.350 7.510 13.110 20.220 27.360 28.620 24.980 19.480 9.980 0.980 -8.580 -17.700 -25.540 -33.440 -35.960 -35.690 -32.470 -23.820 -16.700]';
z =[2.960 21.810 9.820 -1.630 -12.480 -19.930 -22.920 -24.610 -19.460 -14.750 -6.100 2.690 12.790 20.560 27.570 32.590 36.160 36.720 30.490 19.800 7.570 1.350 -4.740 -10.300 -19.700 -27.860 -31.670 -31.810 -28.200 -23.290 -22.340 -23.710 -21.100 -12.470 -4.180 4.540 14.380 22.630 28.910 34.160 38.190 38.400 37.530 31.150 21.730 13.630 13.230 18.780 24.610 28.540 29.500 27.400 24.080 19.730 14.620 5.860 -1.930 -8.690 -15.120 -23.190 -28.560]';

clear data;

% inicializar
i = 1; j = 0; t = 0;
m_ant = [1 1 1];
LGM_bool = 0;
%Resultados esperados = [45; 40; 35; -2; 2.5; 10]
tam(1)= 45; tam(2)= 45; tam(3)= 45; hx= 0; hy= 0; hz= 0;
% elip_mt = zeros(6,6);  % Tag #1. Sist de ecuaciones Gauss-Jordan
%elip_ft_mt = zeros(20,3); % ft= filtro.   mt= matriz
%elip_dp_mt = zeros(20,6); % dp= dervadas parciales
df_dp = zeros(6,6); %Derivadas parciales de cada funcion entre cada variable
continuar = 'S';
% ejecutar bucle cronometrado
tic
e1 = 1 - ((x-hx)/tam(1)).^2 - ((y-hy)/tam(2)).^2 - ((z-hz)/tam(3)).^2;
e2 = (tam(1)*tam(2)*tam(3))^2 - (tam(2)*tam(3)*(x-hx)).^2 ...
                - (tam(1)*tam(3)*(y-hy)).^2 - (tam(1)*tam(2)*(z-hz)).^2;
e1_abs = sum(abs(e1),1)/size(x,1);
e2_abs = sum(abs(e2),1)/size(x,1);
fprintf('iter_init= %d   e1 = %d   e2=%d \n',0, e1_abs, e2_abs);
while i <= 200
    t = toc;
%     mx= data(i,1);
%     my= data(i,2);
%     mz= data(i,3);
%    m = [mx my mz];
%{ }%  
    %% CORRECCIÓN ELIPSE MAGNÉTICA
    %if ( dot(m,m_ant) / (norm(m)*norm(m_ant)) )  < 0.9659  % Menor que 15°
    %    m_ant = m;
        %elip_ft_mt(mod(j,20)+1,:) = m; 
    %    fprintf('%d',j);
    %    fr = 0.8; % fr => Forward Rate
    %    if (mod(j,10) == 9) & (j >= 19)
            % Código #1. Produce Resultados no deseados (No converge correctamente)
            %{ 
            x_hx_4 = sum( (elip_ft_mt(:,1) - hx).^4, 1 ); %matriz de tamaño [20,1]
            y_hy_4 = sum( (elip_ft_mt(:,2) - hy).^4, 1 );
            z_hz_4 = sum( (elip_ft_mt(:,3) - hz).^4, 1 );
            m_div = (1 - ( (elip_ft_mt(:,2)-hy)/tam(2)).^2 ... %Matriz de tamaño [20,1]
                - ((elip_ft_mt(:,3)-hz)/tam(3)).^2 ).*((elip_ft_mt(:,1) - hx).^2); 
            n_div = (1 - ( (elip_ft_mt(:,1)-hx)/tam(1)).^2 ...
                - ((elip_ft_mt(:,3)-hz)/tam(3)).^2 ).*((elip_ft_mt(:,2) - hy).^2);
            o_div = (1 - ( (elip_ft_mt(:,1)-hx)/tam(1)).^2 ...
                - ((elip_ft_mt(:,2)-hy)/tam(2)).^2 ).*((elip_ft_mt(:,3) - hz).^2);
            divide = [sum(m_div,1) sum(n_div,1) sum(o_div,1)]
            fprintf('\n');
            m_increment = sqrt(x_hx_4 / sum(m_div,1)); 
            if isreal(m_increment)
                tam(1) = tam(1) + 0.3*(m_increment - tam(1));  else
                fprintf('i=%d ; j=%d imaginario Valor ´m =%d %dj´ \n', i, j, ...
                    real(m_increment), imag(m_increment)); end
            n_increment = sqrt(y_hy_4 / sum(n_div,1)); 
            if isreal(n_increment)
                tam(2) = tam(2) + 0.3*(n_increment - tam(2)); else
                fprintf('i=%d ; j=%d  Valor ´n =%d %dj´ imaginario \n', i, j, ...
                    real(n_increment), imag(n_increment)); end
            o_increment = sqrt(z_hz_4 / sum(o_div,1)); 
            if isreal(o_increment)
                tam(3) = tam(3) + 0.3*(o_increment - tam(3)); else
                fprintf('i=%d ; j=%d  Valor ´o =%d %dj´ imaginario \n', i, j, ...
                    real(o_increment), imag(o_increment)); end
            fprintf('tam(1)= %.3f ; tam(2)= %.3f ; tam(3)= %.3f \n\n', tam(1), tam(2), tam(3));
            %} 
            
            %Código #2. Falta por análizar minusiosamente los resultados...
            %{
            fprintf('\n');
            e = ( 1 -((elip_ft_mt(:,1)-hx)/tam(1)).^2 -((elip_ft_mt(:,2)-hy)/tam(2)).^2 ...
                -((elip_ft_mt(:,3)-hz)/tam(3)).^2 );
            de_dm = (4/tam(1)^3) * e .* ((elip_ft_mt(:,1)-hx).^2);
            de_dn = (4/tam(2)^3) * e .* ((elip_ft_mt(:,2)-hy).^2);
            de_do = (4/tam(3)^3) * e .* ((elip_ft_mt(:,3)-hz).^2);
            de_dhx = (4/tam(1)^2)* e .* (elip_ft_mt(:,1)-hx);
            de_dhy = (4/tam(2)^2)* e .* (elip_ft_mt(:,2)-hy);
            de_dhz = (4/tam(3)^2)* e .* (elip_ft_mt(:,3)-hz);
            tam(1) = tam(1) - fr *12* sum(de_dm,1);
            tam(2) = tam(2) - fr *12* sum(de_dn,1);
            tam(3) = tam(3) - fr *12* sum(de_do,1);
            hx = hx - fr * sum(de_dhx,1);
            hy = hy - fr * sum(de_dhy,1);
            hz = hz - fr * sum(de_dhz,1);
            fprintf('tam(1)= %.3f ; tam(2)= %.3f ; tam(3)= %.3f \n', tam(1), tam(2), tam(3));
            fprintf('hx  = %.3f ; hy  = %.3f ; hz  = %.3f \n', hx, hy, hz);
            %}
            
            fprintf('\n');
            de1_dm = (2*(x-hx).^2)/tam(1)^3;
            de1_dn = (2*(y-hy).^2)/tam(2)^3;
            de1_do = (2*(z-hz).^2)/tam(3)^3;
            de1_dhx = (2*(x-hx)/tam(1)^2);
            de1_dhy = (2*(y-hy)/tam(2)^2);
            de1_dhz = (2*(z-hz)/tam(3)^2);
            %{
            %derivadas de la función e1^2
            e1_de1dm = 2*sum(e1.*de1_dm, 1);
            e1_de1dn = 2*sum(e1.*de1_dn, 1);
            e1_de1do = 2*sum(e1.*de1_do, 1);
            e1_de1dhx = 2*sum(e1.*de1_dhx, 1);
            e1_de1dhy = 2*sum(e1.*de1_dhy, 1);
            e1_de1dhz = 2*sum(e1.*de1_dhz, 1);
            %derivadas de la función e2^2
            e2_de2dm = 2*sum(e2.*de2_dm, 1);
            e2_de2dn = 2*sum(e2.*de2_dn, 1);
            e2_de2do = 2*sum(e2.*de2_do, 1);
            e2_de2dhx = 2*sum(e2.*de2_dhx, 1);
            e2_de2dhy = 2*sum(e2.*de2_dhy, 1);
            e2_de2dhz = 2*sum(e2.*de2_dhz, 1);
            %}
            
            de2_dm  = 2*tam(1)*( (tam(2)*tam(3))^2 - (tam(3)*(y-hy)).^2 - (tam(2)*(z-hz)).^2 );
            de2_dn  = 2*tam(2)*( (tam(1)*tam(3))^2 - (tam(3)*(x-hx)).^2 - (tam(1)*(z-hz)).^2 );
            de2_do  = 2*tam(3)*( (tam(1)*tam(2))^2 - (tam(2)*(x-hx)).^2 - (tam(1)*(y-hy)).^2 );
            de2_dhx = 2*(x-hx)*(tam(2)*tam(3))^2;
            de2_dhy = 2*(y-hy)*(tam(1)*tam(3))^2;
            de2_dhz = 2*(z-hz)*(tam(1)*tam(2))^2;
            %(Abajo) Derivadas parciales de f1(m,n,o,hx,hy,hz))
            d2e2_dm2 = sum(e2*2.*((tam(2)*tam(3))^2 -(tam(3)*(y-hy)).^2 ...
                -(tam(2)*(z-hz)).^2) + de2_dm.*de2_dm, 1);
            d2e2_dmdn  = sum(e2.*(4*tam(1)*tam(2)*(tam(3)^2 -(z-hz).^2)) + de2_dm.*de2_dn, 1);
            d2e2_dmdo  = sum(e2.*(4*tam(1)*tam(3)*(tam(2)^2 -(y-hy).^2)) + de2_dm.*de2_do, 1);
            d2e2_dmdhx = sum(de2_dm.*de2_dhx, 1);
            d2e2_dmdhy = sum(e2.*(4*tam(1)*tam(3)^2*(y-hy)) + de2_dm.*de2_dhy, 1);
            d2e2_dmdhz = sum(e2.*(4*tam(1)*tam(2)^2*(z-hz)) + de2_dm.*de2_dhz, 1);
            %(Abajo) Derivadas parciales de f2(m,n,o,hx,hy,hz))
            d2e2_dndm  = sum(e2.*(4*tam(2)*tam(1)*(tam(3)^2 -(z-hz).^2)) + de2_dn.*de2_dm, 1);
            d2e2_dn2   = sum(e2*2.*((tam(1)*tam(3))^2 -(tam(3)*(x-hx)).^2 ...
                -(tam(1)*(z-hz)).^2) + de2_dn.*de2_dn, 1);
            d2e2_dndo  = sum(e2.*(4*tam(2)*tam(3)*(tam(1)^2 -(x-hx).^2)) + de2_dn.*de2_do, 1);
            d2e2_dndhx = sum(e2.*(4*tam(2)*tam(3)^2*(x-hx)) + de2_dn.*de2_dhx, 1);
            d2e2_dndhy = sum(de2_dn.*de2_dhy, 1);
            d2e2_dndhz = sum(e2.*(4*tam(2)*tam(1)^2*(z-hz)) + de2_dn.*de2_dhz, 1);
            %(Abajo) Derivadas parciales de f3(m,n,o,hx,hy,hz))
            d2e2_dodm  = sum(e2.*(4*tam(3)*tam(1)*(tam(2)^2 -(y-hy).^2)) + de2_do.*de2_dm, 1);
            d2e2_dodn  = sum(e2.*(4*tam(3)*tam(2)*(tam(1)^2 -(x-hx).^2)) + de2_do.*de2_dn, 1);
            d2e2_do2   = sum(e2*2.*((tam(1)*tam(2))^2 -(tam(1)*(y-hy)).^2 ...
                -(tam(2)*(x-hx)).^2) + de2_do.*de2_do, 1);
            d2e2_dodhx = sum(e2.*(4*tam(3)*tam(2)^2*(x-hx)) + de2_do.*de2_dhx, 1);
            d2e2_dodhy = sum(e2.*(4*tam(3)*tam(1)^2*(y-hy)) + de2_do.*de2_dhy, 1);
            d2e2_dodhz = sum(de2_do.*de2_dhz, 1);
            
            %{
            %(Abajo) Derivadas parciales de f1(m,n,o,hx,hy,hz))
            d2e1_dm2   = sum(e1.*(-6*(x-hx).^2/tam(1)^4) + de1_dm.*de1_dm, 1);
            d2e1_dmdn  = sum(de1_dn .* de1_dm, 1);
            d2e1_dmdo  = sum(de1_do .* de1_dm, 1);
            d2e1_dmdhx = sum(e1.*(-4*(x-hx)/tam(1)^3) + de1_dhx.*de1_dm, 1);
            d2e1_dmdhy = sum(de1_dhy .* de1_dm, 1);
            d2e1_dmdhz = sum(de1_dhz .* de1_dm, 1);
            %(Abajo) Derivadas parciales de f2(m,n,o,hx,hy,hz))
            d2e1_dndm  = sum(de1_dm .* de1_dn, 1);
            d2e1_dn2   = sum(e1.*(-6*(y-hy).^2/tam(2)^4) + de1_dn.*de1_dn, 1);
            d2e1_dndo  = sum(de1_do .* de1_dn, 1);
            d2e1_dndhx = sum(de1_dhx .* de1_dn, 1);
            d2e1_dndhy = sum(e1.*(-4*(y-hy)/tam(2)^3) + de1_dhy.*de1_dn, 1);
            d2e1_dndhz = sum(de1_dhz .* de1_dn, 1);
            %(Abajo) Derivadas parciales de f3(m,n,o,hx,hy,hz))
            d2e1_dodm  = sum(de1_dm .* de1_do, 1);
            d2e1_dodn  = sum(de1_dn .* de1_do, 1);
            d2e1_do2   = sum(e1.*(-6*(z-hz).^2/tam(3)^4) + de1_do.*de1_do, 1);
            d2e1_dodhx = sum(de1_dhx .* de1_do, 1);
            d2e1_dodhy = sum(de1_dhy .* de1_do, 1);
            d2e1_dodhz = sum(e1.*(-4*(z-hz)/tam(3)^3) + de1_dhz.*de1_do, 1);
            %}
            %(Abajo) Derivadas parciales de f4(m,n,o,hx,hy,hz))
            d2e1_dhxdm  = sum(e1.*(-4*(x-hx)/tam(1)^3) + de1_dm.*de1_dhx, 1);
            d2e1_dhxdn  = sum(de1_dn .* de1_dhx, 1);
            d2e1_dhxdo  = sum(de1_do .* de1_dhx, 1);
            d2e1_dhx2   = sum(e1.*(-2/tam(1)^2) + de1_dhx.*de1_dhx, 1);
            d2e1_dhxdhy = sum(de1_dhy .* de1_dhx, 1);
            d2e1_dhxdhz = sum(de1_dhz .* de1_dhx, 1);
            %(Abajo) Derivadas parciales de f5(m,n,o,hx,hy,hz))
            d2e1_dhydm  = sum(de1_dm .* de1_dhy, 1);
            d2e1_dhydn  = sum(e1.*(-4*(y-hy)/tam(2)^3) + de1_dn.*de1_dhy, 1);
            d2e1_dhydo  = sum(de1_do .* de1_dhy, 1);
            d2e1_dhydhx = sum(de1_dhx .* de1_dhy, 1);
            d2e1_dhy2   = sum(e1.*(-2/tam(2)^2) + de1_dhy.*de1_dhy, 1);
            d2e1_dhydhz = sum(de1_dhz .* de1_dhy, 1);
            %(Abajo) Derivadas parciales de f6(m,n,o,hx,hy,hz))
            d2e1_dhzdm  = sum(de1_dm .* de1_dhz, 1);
            d2e1_dhzdn  = sum(de1_dn .* de1_dhz, 1);
            d2e1_dhzdo  = sum(e1.*(-4*(z-hz)/tam(3)^3) + de1_do.*de1_dhz, 1);
            d2e1_dhzdhx = sum(de1_dhx .* de1_dhz, 1);
            d2e1_dhzdhy = sum(de1_dhy .* de1_dhz, 1);
            d2e1_dhz2   = sum(e1.*(-2/tam(3)^2) + de1_dhz.*de1_dhz, 1);
            % Matriz Jacobiana Newton-Raphson
            df_dp(1,:) = [d2e2_dm2 d2e2_dmdn d2e2_dmdo d2e2_dmdhx d2e2_dmdhy d2e2_dmdhz];
            df_dp(2,:) = [d2e2_dndm d2e2_dn2 d2e2_dndo d2e2_dndhx d2e2_dndhy d2e2_dndhz];
            df_dp(3,:) = [d2e2_dodm d2e2_dodn d2e2_do2 d2e2_dodhx d2e2_dodhy d2e2_dodhz];
            df_dp(4,:) = [d2e1_dhxdm d2e1_dhxdn d2e1_dhxdo d2e1_dhx2 d2e1_dhxdhy d2e1_dhxdhz];
            df_dp(5,:) = [d2e1_dhydm d2e1_dhydn d2e1_dhydo d2e1_dhydhx d2e1_dhy2 d2e1_dhydhz];
            df_dp(6,:) = [d2e1_dhzdm d2e1_dhzdn d2e1_dhzdo d2e1_dhzdhx d2e1_dhzdhy d2e1_dhz2];
            
            % Vector de funciones [f1; f2; f3; f4; f5; f6]
            f1 = sum(e2 .* de2_dm, 1);
            f2 = sum(e2 .* de2_dn, 1);
            f3 = sum(e2 .* de2_do, 1);
            f4 = sum(e1 .* de1_dhx, 1);
            f5 = sum(e1 .* de1_dhy, 1);
            f6 = sum(e1 .* de1_dhz, 1);
            func = [-f1; -f2; -f3; -f4; -f5; -f6];
            
            deriv = mldivide(df_dp, func);
            NAN_deriv = isnan(deriv);
            deriv = deriv.*not(NAN_deriv);
            if sum(NAN_deriv,2) > 0
                fprintf('Error NaN number: %d\n', NAN_deriv);
            end
            % st --> Step 
            st_tam(1)= 0.1+rand()/2; st_tam(2)= 0.1+rand()/2;  st_tam(3)= 0.1+rand()/2;
            st_hx  = 0.1+rand()/2; st_hy  = 0.1+rand()/2;  st_hz  = 0.1+rand()/2;
            dtam(1) = st_tam(1)*deriv(1,1);  dtam(2) = st_tam(2)*deriv(2,1);
            dtam(3) = st_tam(2)*deriv(3,1);  dhx = st_hx*deriv(4,1); 
            dhy = st_hy*deriv(5,1);      dhz = st_hz*deriv(6,1);
            % Correcciones abajo
            tam(1)= tam(1)+dtam(1);  tam(2)= tam(2)+dtam(2); tam(3)= tam(3)+dtam(3);
            hx = hx+dhx;  hy = hy+dhy;  hz = hz+dhz;
            % ¿Las correcciones mejoraron el resultado ¿SI o NO?
            e1_ant = e1_abs;  e2_ant = e2_abs;
            e1 = 1 - ((x-hx)/tam(1)).^2 - ((y-hy)/tam(2)).^2 - ((z-hz)/tam(3)).^2;
            e2 = (tam(1)*tam(2)*tam(3))^2 - (tam(2)*tam(3)*(x-hx)).^2 ...
                - (tam(1)*tam(3)*(y-hy)).^2 - (tam(1)*tam(2)*(z-hz)).^2;
            e1_abs = sum(abs(e1),1)/size(x,1);
            e2_abs = sum(abs(e2),1)/size(x,1);
            fprintf('        e1_ant= %d   e2_ant= %d \n', e1_ant, e2_ant);
            fprintf('iter= %d    e1= %d       e2= %d \n',i, e1_abs, e2_abs);
            
            if (e1_abs >= e1_ant | e2_abs >= e2_ant) 
                tam(1) = tam(1) - st_tam(1)*deriv(1,1);
                tam(2) = tam(2) - st_tam(2)*deriv(2,1);
                tam(3) = tam(3) - st_tam(3)*deriv(3,1);
                hx = hx - st_hx*deriv(4,1);
                hy = hy - st_hy*deriv(5,1);
                hz = hz - st_hz*deriv(6,1);
                e1_abs = e1_ant; e2_abs = e2_ant;
                %fprintf('Corrección: e1= %d      e2= %d \n', e1_abs, e2_abs);
            end
            tam(1)=abs(tam(1));  tam(2)=abs(tam(2));  tam(3)=abs(tam(3));
            fprintf('tam(1)= %.3f ; tam(2)= %.3f ; tam(3)= %.3f \n', tam(1), tam(2), tam(3));
            fprintf('hx  = %.3f ; hy  = %.3f ; hz  = %.3f \n', hx, hy, hz);
            
            LMG = sort(tam); % Lower Medium Greater
            r_ML = LMG(2)/LMG(1);% Rate Medium vs Lower
            r_GM = LMG(3)/LMG(2);
            if r_GM > 1.2
                LGM_bool = 1;
                tam(tam==LMG(3)) = 1*LMG(2) + 0.154*LMG(3) % 1.1, 0.154
            end
            if r_ML > 1.2
                LGM_bool = 1;
                tam(tam==LMG(1)) = 0.4*LMG(2) + 0.53*LMG(1) %0.4, 0.48
            end
            if LGM_bool
                LGM_bool = 0;
                e1 = 1 - ((x-hx)/tam(1)).^2 - ((y-hy)/tam(2)).^2 - ((z-hz)/tam(3)).^2;
                e2 = (tam(1)*tam(2)*tam(3))^2 - (tam(2)*tam(3)*(x-hx)).^2 ...
                    - (tam(1)*tam(3)*(y-hy)).^2 - (tam(1)*tam(2)*(z-hz)).^2;
                e1_abs = sum(abs(e1),1)/size(x,1);
                e2_abs = sum(abs(e2),1)/size(x,1);
                fprintf('LMG :   e1_LMG= %d   e2_LMG= %d \n', e1_abs, e2_abs);
                fprintf('LMG tam(1)= %.3f ; tam(2)= %.3f ; tam(3)= %.3f \n', tam(1), tam(2), tam(3));
            end
            
    %    end
    %    j = j + 1;
    %end
%}
    
% Tag #1 Sistema de ecuaciones Gauss-Jordan
%{
     elip_fila = mod(i,6);
     elip_mt(elip_fila+1,:) = [mx^2  -2*mx  my^2  -2*my  mz^2  -2*mz];
     
     if(elip_fila == 5 && i > 10)
         solv = mldivide(elip_mt, ones(6,1));
         lambda = (solv(2)^2)/solv(1) + (solv(4)^2)/solv(3) +(solv(6)^2)/solv(5);
         P = 1 / (1+lambda);
         Q = 1-P;
         tam(1) = 1 / sqrt(solv(1));  % Radio eje X
         tam(2) = 1 / sqrt(solv(3));  % Radio eje X
         tam(3) = 1 / sqrt(solv(5));  % Radio eje X
         hx = solv(2)/solv(1); % Offset eje X
         hy = solv(4)/solv(3); % Offset eje Y
         hz = solv(6)/solv(5); % Offset eje Z
         elip_sol = [m n o hx hy hz]
     end
%}

    i = i+1;
end
fprintf('\nResultados esperados. "%d Valores" \n', nVal);
fprintf('tam(1)= %.3f ; tam(2)= %.3f ; tam(3)= %.3f \n', m, n, o);
fprintf('hx  = %.3f ; hy  = %.3f ; hz  = %.3f \n', ehx, ehy, ehz);
%% FIN DEL PROGRAMA
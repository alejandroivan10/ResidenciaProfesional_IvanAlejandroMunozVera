%{
Entrada: Arduino ax,ay,az,mx,my,mz
Salida: Graficación del vector gravedad y campo magnetico (corregido)
%}

clear all
close all
clc

%borrar datos previos
delete(instrfind({'Port'},{'COM3'}));
%crear objeto serie y determinar salto de linea como final de dato
s = serial('COM3','BaudRate',115200,'Terminator','CR/LF');
warning('off','MATLAB:serial:fscanf:unsuccessfulRead');
fopen(s); %abrir puerto
xlabel('Eje x')
ylabel('Ejtmax = 100;e y')
zlabel('Eje z')
title('Acelerometro en tiempo real con Arduino')
ro = 50;
xlim ([-1.1*ro 1.1*ro])
ylim ([-1.1*ro 1.1*ro])
zlim ([-1.1*ro 1.1*ro])
grid on
hold on
set(gca, 'CameraPosition', [1.1*ro -1.1*ro 1.1*ro]);

% inicializar
i = 1; j = 0; t = 0;
Mg_ant = [1 1 1];
tam1= 38; tam2= 38; tam3= 38; hx= 0; hy= 0; hz= 0;
% elip_mt = zeros(6,6);  % Tag #1. Sist de ecuaciones Gauss-Jordan
%elip_ft_mt = zeros(20,3); % ft= filtro.   mt= matriz
elip_dp_mt = zeros(20,6); % dp= dervadas parciales
continuar = 'S';
% ejecutar bucle cronometrado
tic
nVal = 50;
x = zeros(20,1); y = zeros(20,1); z = zeros(20,1);
while continuar == 'S'
    refresh
    %t = toc;
    % leer del puerto serie
    val = fscanf(s,'%f %f %f %f %f %f %f %f %f')';
    Mg_escale = [1.0434  1.0584  1.0125];
    Mg_offset = [4.299 -2.627   3.177];
    %iter= 200    e1= 1.140291e-01       e2= 2.359442e+08
    %tam(1)= 35.842 ; tam(2)= 35.333 ; tam(3)= 37.398 
    %hx  = 4.299 ; hy  = -2.627 ; hz  = 3.177
    ax=val(1)*5;
    ay=val(2)*5;
    az=val(3)*5;
    %gx=val(4);
    %gy=val(5);
    %gz=val(6);    
    mx=-val(7);
    my=-val(8);
    mz=-val(9);
    mx = (mx - Mg_offset(1)) * Mg_escale(1);
    my = (my - Mg_offset(2)) * Mg_escale(2);
    mz = (mz - Mg_offset(3)) * Mg_escale(3);
    Ac = [ax ay az];
    Mg = [mx my mz];
    fprintf('i=%d  ax=%.3f ay=%.3f az=%.3f   mx=%.3f my=%.3f mz=%.3f \n', i,ax,ay,az,mx,my,mz);
%{   
    %% CORRECCIÓN ELIPSE MAGNÉTICA
    if ( (dot(Mg,Mg_ant) / (norm(Mg)*norm(Mg_ant)))  < 0.9659 ) & 0 % Menor que 15°
        Mg_ant = Mg;
        %elip_ft_mt(mod(j,20)+1,:) = Mg; 
        x(mod(j,200)+1) = mx; y(mod(j,200)+1) = my;  z(mod(j,200)+1) = mz;
        fprintf('%d',j);
        fr = 0.8; % fr => Forward Rate
        if (mod(j,10) == 9) & (j >= 19)
            % Código anterior. Produce Resultados no deseados (No converge correctamente)
            %{ 
            x_hx_4 = sum( (elip_ft_mt(:,1) - hx).^4, 1 ); %matriz de tamaño [20,1]
            y_hy_4 = sum( (elip_ft_mt(:,2) - hy).^4, 1 );
            z_hz_4 = sum( (elip_ft_mt(:,3) - hz).^4, 1 );
            m_div = (1 - ( (elip_ft_mt(:,2)-hy)/tam2).^2 ... %Matriz de tamaño [20,1]
                - ((elip_ft_mt(:,3)-hz)/tam3).^2 ).*((elip_ft_mt(:,1) - hx).^2); 
            n_div = (1 - ( (elip_ft_mt(:,1)-hx)/tam1).^2 ...
                - ((elip_ft_mt(:,3)-hz)/tam3).^2 ).*((elip_ft_mt(:,2) - hy).^2);
            o_div = (1 - ( (elip_ft_mt(:,1)-hx)/tam1).^2 ...
                - ((elip_ft_mt(:,2)-hy)/tam2).^2 ).*((elip_ft_mt(:,3) - hz).^2);
            divide = [sum(m_div,1) sum(n_div,1) sum(o_div,1)]
            fprintf('\n');
            m_increment = sqrt(x_hx_4 / sum(m_div,1)); 
            if isreal(m_increment)
                tam1 = tam1 + 0.3*(m_increment - tam1);  else
                fprintf('i=%d ; j=%d imaginario Valor ´m =%d %dj´ \n', i, j, ...
                    real(m_increment), imag(m_increment)); end
            n_increment = sqrt(y_hy_4 / sum(n_div,1)); 
            if isreal(n_increment)
                tam2 = tam2 + 0.3*(n_increment - tam2); else
                fprintf('i=%d ; j=%d  Valor ´n =%d %dj´ imaginario \n', i, j, ...
                    real(n_increment), imag(n_increment)); end
            o_increment = sqrt(z_hz_4 / sum(o_div,1)); 
            if isreal(o_increment)
                tam3 = tam3 + 0.3*(o_increment - tam3); else
                fprintf('i=%d ; j=%d  Valor ´o =%d %dj´ imaginario \n', i, j, ...
                    real(o_increment), imag(o_increment)); end
            fprintf('tam1= %.3f ; tam2= %.3f ; tam3= %.3f \n\n', tam1, tam2, tam3);
            %} 
            
            %{
            fprintf('\n');
            e = ( 1 -((elip_ft_mt(:,1)-hx)/tam1).^2 -((elip_ft_mt(:,2)-hy)/tam2).^2 ...
                -((elip_ft_mt(:,3)-hz)/tam3).^2 );
            de_dm = (4/tam1^3) * e .* ((elip_ft_mt(:,1)-hx).^2);
            de_dn = (4/tam2^3) * e .* ((elip_ft_mt(:,2)-hy).^2);
            de_do = (4/tam3^3) * e .* ((elip_ft_mt(:,3)-hz).^2);
            de_dhx = (4/tam1^2)* e .* (elip_ft_mt(:,1)-hx);
            de_dhy = (4/tam2^2)* e .* (elip_ft_mt(:,2)-hy);
            de_dhz = (4/tam3^2)* e .* (elip_ft_mt(:,3)-hz);
            tam1 = tam1 - fr *12* sum(de_dm,1);
            tam2 = tam2 - fr *12* sum(de_dn,1);
            tam3 = tam3 - fr *12* sum(de_do,1);
            hx = hx - fr * sum(de_dhx,1);
            hy = hy - fr * sum(de_dhy,1);
            hz = hz - fr * sum(de_dhz,1);
            fprintf('tam1= %.3f ; tam2= %.3f ; tam3= %.3f \n', tam1, tam2, tam3);
            fprintf('hx  = %.3f ; hy  = %.3f ; hz  = %.3f \n', hx, hy, hz);
            %}
            
            fprintf('\n');
            e = 1 - ((x-hx)/tam1).^2 - ((y-hy)/tam2).^2 - ((z-hz)/tam3).^2;
            de_dm = (2*(x-hx).^2)/tam1^3;
            de_dn = (2*(y-hy).^2)/tam2^3;
            de_do = (2*(z-hz).^2)/tam3^3;
            de_dhx = (2*(x-hx)/tam1^2);
            de_dhy = (2*(y-hy)/tam2^2);
            de_dhz = (2*(z-hz)/tam3^2);
            
            %(Abajo) Derivadas parciales de f2(m,n,o,hx,hy,hz))
            d2e_dm2   = sum(e.*(-6*(x-hx).^2/tam1^4) + de_dm.*de_dm, 1);
            d2e_dmdn  = sum(de_dn .* de_dm, 1);
            d2e_dmdo  = sum(de_do .* de_dm, 1);
            d2e_dmdhx = sum(e.*(-4*(x-hx)/tam1^3) + de_dhx.*de_dm, 1);
            d2e_dmdhy = sum(de_dhy .* de_dm, 1);
            d2e_dmdhz = sum(de_dhz .* de_dm, 1);
            %(Abajo) Derivadas parciales de f2(m,n,o,hx,hy,hz))
            d2e_dndm  = sum(de_dm .* de_dn, 1);
            d2e_dn2   = sum(e.*(-6*(y-hy).^2/tam2^4) + de_dn.*de_dn, 1);
            d2e_dndo  = sum(de_do .* de_dn, 1);
            d2e_dndhx = sum(de_dhx .* de_dn, 1);
            d2e_dndhy = sum(e.*(-4*(y-hy)/tam2^3) + de_dhy.*de_dn, 1);
            d2e_dndhz = sum(de_dhz .* de_dn, 1);
            %(Abajo) Derivadas parciales de f3(m,n,o,hx,hy,hz))
            d2e_dodm  = sum(de_dm .* de_do, 1);
            d2e_dodn  = sum(de_dn .* de_do, 1);
            d2e_do2   = sum(e.*(-6*(z-hz).^2/tam3^4) + de_do.*de_do, 1);
            d2e_dodhx = sum(de_dhx .* de_do, 1);
            d2e_dodhy = sum(de_dhy .* de_do, 1);
            d2e_dodhz = sum(e.*(-4*(z-hz)/tam3^3) + de_dhz.*de_do, 1);
            %(Abajo) Derivadas parciales de f4(m,n,o,hx,hy,hz))
            d2e_dhxdm  = sum(e.*(-4*(x-hx)/tam1^3) + de_dm.*de_dhx, 1);
            d2e_dhxdn  = sum(de_dn .* de_dhx, 1);
            d2e_dhxdo  = sum(de_do .* de_dhx, 1);
            d2e_dhx2   = sum(e.*(-2/tam1^2) + de_dhx.*de_dhx, 1);
            d2e_dhxdhy = sum(de_dhy .* de_dhx, 1);
            d2e_dhxdhz = sum(de_dhz .* de_dhx, 1);
            %(Abajo) Derivadas parciales de f5(m,n,o,hx,hy,hz))
            d2e_dhydm  = sum(de_dm .* de_dhy, 1);
            d2e_dhydn  = sum(e.*(-4*(y-hy)/tam2^3) + de_dn.*de_dhy, 1);
            d2e_dhydo  = sum(de_do .* de_dhy, 1);
            d2e_dhydhx = sum(de_dhx .* de_dhy, 1);
            d2e_dhy2   = sum(e.*(-2/tam2^2) + de_dhy.*de_dhy, 1);
            d2e_dhydhz = sum(de_dhz .* de_dhy, 1);
            %(Abajo) Derivadas parciales de f6(m,n,o,hx,hy,hz))
            d2e_dhzdm  = sum(de_dm .* de_dhz, 1);
            d2e_dhzdn  = sum(de_dn .* de_dhz, 1);
            d2e_dhzdo  = sum(e.*(-4*(z-hz)/tam3^3) + de_do.*de_dhz, 1);
            d2e_dhzdhx = sum(de_dhx .* de_dhz, 1);
            d2e_dhzdhy = sum(de_dhy .* de_dhz, 1);
            d2e_dhz2   = sum(e.*(-2/tam3^2) + de_dhz.*de_dhz, 1);
            % Matriz Jacobiana Newton-Raphson
            df_dp(1,:) = [d2e_dm2 d2e_dmdn d2e_dmdo d2e_dmdhx d2e_dmdhy d2e_dmdhz];
            df_dp(2,:) = [d2e_dndm d2e_dn2 d2e_dndo d2e_dndhx d2e_dndhy d2e_dndhz];
            df_dp(3,:) = [d2e_dodm d2e_dodn d2e_do2 d2e_dodhx d2e_dodhy d2e_dodhz];
            df_dp(4,:) = [d2e_dhxdm d2e_dhxdn d2e_dhxdo d2e_dhx2 d2e_dhxdhy d2e_dhxdhz];
            df_dp(5,:) = [d2e_dhydm d2e_dhydn d2e_dhydo d2e_dhydhx d2e_dhy2 d2e_dhydhz];
            df_dp(6,:) = [d2e_dhzdm d2e_dhzdn d2e_dhzdo d2e_dhzdhx d2e_dhzdhy d2e_dhz2];
            
            % Vector de funciones [f1; f2; f3; f4; f5; f6]
            f1 = sum(e .* de_dm, 1);
            f2 = sum(e .* de_dn, 1);
            f3 = sum(e .* de_do, 1);
            f4 = sum(e .* de_dhx, 1);
            f5 = sum(e .* de_dhy, 1);
            f6 = sum(e .* de_dhz, 1);
            func = [-f1; -f2; -f3; -f4; -f5; -f6];
            
            deriv = mldivide(df_dp, func);
            tam1 = tam1 + 0.2*deriv(1,1);
            tam2 = tam2 + 0.2*deriv(2,1);
            tam3 = tam3 + 0.2*deriv(3,1);
            hx = hx + 0.5*deriv(4,1);
            hy = hy + 0.5*deriv(5,1);
            hz = hz + 0.5*deriv(6,1);
            %[tam1; tam2; tam3; hx; hy; hz]
        end
        
        j = j + 1;
    end
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
         tam1 = 1 / sqrt(solv(1));  % Radio eje X
         tam2 = 1 / sqrt(solv(3));  % Radio eje X
         tam3 = 1 / sqrt(solv(5));  % Radio eje X
         hx = solv(2)/solv(1); % Offset eje X
         hy = solv(4)/solv(3); % Offset eje Y
         hz = solv(6)/solv(5); % Offset eje Z
         elip_sol = [m n o hx hy hz]
     end
%}

    
    
    %% OBTENCIÓN DE LA ORIENTACIÓN TOTAL DEL SISTEMA X, Y, Z
    % 1) Conseguir plano perpendicular a la gravedad, a la vez que el plano
    % contiene el punto en donde termina el campo magnetico (mx,my,mz)
    % Eq del plano => a(x-x0)+b(y-y0)+c(z-z0)=0  ó bien => ax+by+cz+d=0
    % Eq de recta 3D =>  X = xt; Y = yt; Z = zt          POR TANTO:
    x0 = mx; y0 = my; z0 = mz;
    a = ax; b = ay; c = az;  d = -a*x0-b*y0-c*z0;
    t = (-d)/(a^2+b^2+c^2);
    X = a*t; Y = b*t; Z = c*t;
    
    % 2) Obtener el vector perpendicular a la aceleración: i(X-x0)+j(Y-y0)+k(Z-z0)
    % El vector apunta al NORTE MÁGNETICO (NM)
    NMx = x0-X; NMy = y0-Y; NMz = z0-Z;
    NM = [ NMx NMy NMz ];
    
    %% GRÁFICA EN PANTALLA
    % GRAVEDAD
    G=plot3([0;ax],[0;ay],[0;az],'Color','r','LineWidth',2);
    Gt=text(ax,ay,az, 'Gravity', 'Rotation',+15);
    %MAGNETOMETRO
    M=plot3([0;mx],[0;my],[0;mz],'Color','y','LineWidth',2);
    Mt=text(mx,my,mz, 'Magnetic', 'Rotation',+15);
    
    %NORTE MÁGNETICO
    NM=plot3([X;X+NMx],[Y;Y+NMy],[Z;Z+NMz],'Color','y','LineWidth',2);
    NMt=text(X,Y,Z, 'Magnetic', 'Rotation',+15);
    %AVERAGE
    angle = acosd( dot(Ac,Mg) / (norm(Ac)*norm(Mg)) );
    av = (Ac + Mg) / 2;
    Avt=text(av(1),av(2),av(3), num2str(angle), 'Rotation',+15);
    %eje x
    h1=plot3([0;ro],[0;0],[0;0],'Color','r','LineWidth',2);
    h1t=text(ro,0,0, 'Eje X', 'Rotation',+15);
    % %eje y
    h2=plot3([0;0],[0;ro],[0;0],'Color','g','LineWidth',2);
    h2t=text(0,ro,0, 'Eje Y', 'Rotation',+15);
    % %eje z
    h3=plot3([0;0],[0;0],[0;ro],'Color','b','LineWidth',2);
    h3t=text(0,0,ro, 'Eje Z', 'Rotation',+15);
    
    drawnow
    %refreshdata
    i = i+1;
    
    %pause(0.03)
    delete(G); delete(Gt);
    delete(M); delete(Mt);
    delete(NM); delete(NMt);
    delete(Avt);
    delete(h1); delete(h1t);
    delete(h2); delete(h2t);
    delete(h3); delete(h3t);
end

%x =[1.330 12.190 4.290 -1.770 -7.980 -12.920 -18.230 -22.820 -26.870 -28.880 -28.900 -28.170 -26.120 -22.740 -16.710 -11.130 -4.780 4.050 -6.570 -17.850 -21.770 -22.530 -22.360 -21.750 -18.190 -12.990 -4.220 4.870 14.650 20.130 12.100 0.780 -11.740 -20.800 -25.610 -28.830 -30.480 -27.430 -20.850 -10.090 0.830 11.900 19.610 26.260 31.430 29.630 20.200 11.350 2.430 -7.440 -16.810 -20.630 -21.510 -21.320 -20.640 -15.080 -7.930 1.280 8.970 11.660 8.920]
%y =[-2.570 -34.420 -36.620 -34.700 -31.670 -27.470 -19.940 -10.730 -2.820 6.040 11.200 13.990 15.770 11.410 2.730 -3.920 -11.920 -20.480 -24.650 -17.490 -10.310 -6.370 -2.170 1.540 6.670 9.580 5.570 -1.580 -7.610 -15.420 -22.700 -25.700 -25.380 -22.760 -20.220 -16.770 -10.730 -4.830 1.660 6.560 9.790 8.810 0.350 7.510 13.110 20.220 27.360 28.620 24.980 19.480 9.980 0.980 -8.580 -17.700 -25.540 -33.440 -35.960 -35.690 -32.470 -23.820 -16.700]
%z =[2.960 21.810 9.820 -1.630 -12.480 -19.930 -22.920 -24.610 -19.460 -14.750 -6.100 2.690 12.790 20.560 27.570 32.590 36.160 36.720 30.490 19.800 7.570 1.350 -4.740 -10.300 -19.700 -27.860 -31.670 -31.810 -28.200 -23.290 -22.340 -23.710 -21.100 -12.470 -4.180 4.540 14.380 22.630 28.910 34.160 38.190 38.400 37.530 31.150 21.730 13.630 13.230 18.780 24.610 28.540 29.500 27.400 24.080 19.730 14.620 5.860 -1.930 -8.690 -15.120 -23.190 -28.560]

%% FIN DEL PROGRAMA
% resultado del cronometro
fclose(s);
delete(s);
clear s;
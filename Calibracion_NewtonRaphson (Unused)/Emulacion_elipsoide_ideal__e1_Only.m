clear all
close all
clc

%Creación de datos ficticios (magnétometro)
nVal = 20;
data(:,1) = (rand(nVal,1)-0.5) * 90; % rand() -> Valores entre 0 y 1
data(:,2) = (rand(nVal,1)-0.5).*sqrt(1 - (data(:,1)/45).^2) * 80;
data(:,3) = sqrt(1 - (data(:,1)/45).^2 - (data(:,2)/40).^2) * 35;
answ = rand(nVal,1) >= 0.5;
data(:,3) = ((-1).^answ).*data(:,3);
%comprobar = 1 - (data(:,1)/45).^2 - (data(:,2)/40).^2 - (data(:,3)/35).^2
ehx = -20; ehy = -20; ehz = -10;  % Introducir error OFFSET en la 'data'
data(:,1) = data(:,1) + ehx;
data(:,2) = data(:,2) + ehy;
data(:,3) = data(:,3) + ehz;
x = data(:,1);
y = data(:,2);
z = data(:,3);
clear data;

% inicializar
i = 1; j = 0; t = 0;
m_ant = [1 1 1];
%Resultados esperados = [45; 40; 35; -2; 2.5; 10]
tam1= 10; tam2= 10; tam3= 10; hx= 0; hy= 0; hz= 0;
% elip_mt = zeros(6,6);  % Tag #1. Sist de ecuaciones Gauss-Jordan
%elip_ft_mt = zeros(20,3); % ft= filtro.   mt= matriz
%elip_dp_mt = zeros(20,6); % dp= dervadas parciales
df_dp = zeros(6,6); %Derivadas parciales de cada funcion entre cada variable
continuar = 'S';
% ejecutar bucle cronometrado
tic
while i < 200
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
            
            %Código #2. Falta por análizar minusiosamente los resultados...
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
            de_dm = (2*(x-hx).^2)/tam1^3;   m3_de_dm = de_dm .* tam1^1;
            de_dn = (2*(y-hy).^2)/tam2^3;   n3_de_dn = de_dn .* tam2^1;
            de_do = (2*(z-hz).^2)/tam3^3;   o3_de_do = de_do .* tam3^1;
            de_dhx = (2*(x-hx)/tam1^2);
            de_dhy = (2*(y-hy)/tam2^2);
            de_dhz = (2*(z-hz)/tam3^2);
            
            %(Abajo) Derivadas parciales de f2(m,n,o,hx,hy,hz))
            d2e_dm2   = sum(e.*(-4*(x-hx).^2/tam1^3) + de_dm.*m3_de_dm, 1);
            d2e_dmdn  = sum(de_dn .* m3_de_dm, 1);
            d2e_dmdo  = sum(de_do .* m3_de_dm, 1);
            d2e_dmdhx = sum(e.*(-4*(x-hx)) + de_dhx.*m3_de_dm, 1);
            d2e_dmdhy = sum(de_dhy .* m3_de_dm, 1);
            d2e_dmdhz = sum(de_dhz .* m3_de_dm, 1);
            %(Abajo) Derivadas parciales de f2(m,n,o,hx,hy,hz))
            d2e_dndm  = sum(de_dm .* n3_de_dn, 1);
            d2e_dn2   = sum(e.*(-4*(y-hy).^2/tam2^3) + de_dn.*n3_de_dn, 1);
            d2e_dndo  = sum(de_do .* n3_de_dn, 1);
            d2e_dndhx = sum(de_dhx .* n3_de_dn, 1);
            d2e_dndhy = sum(e.*(-4*(y-hy)) + de_dhy.*n3_de_dn, 1);
            d2e_dndhz = sum(de_dhz .* n3_de_dn, 1);
            %(Abajo) Derivadas parciales de f3(m,n,o,hx,hy,hz))
            d2e_dodm  = sum(de_dm .* o3_de_do, 1);
            d2e_dodn  = sum(de_dn .* o3_de_do, 1);
            d2e_do2   = sum(e.*(-4*(z-hz).^2/tam3^3) + de_do.*o3_de_do, 1);
            d2e_dodhx = sum(de_dhx .* o3_de_do, 1);
            d2e_dodhy = sum(de_dhy .* o3_de_do, 1);
            d2e_dodhz = sum(e.*(-4*(z-hz)/tam3^3) + de_dhz.*o3_de_do, 1);
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
            f1 = sum(e .* m3_de_dm, 1);
            f2 = sum(e .* n3_de_dn, 1);
            f3 = sum(e .* o3_de_do, 1);
            f4 = sum(e .* de_dhx, 1);
            f5 = sum(e .* de_dhy, 1);
            f6 = sum(e .* de_dhz, 1);
            func = [-f1; -f2; -f3; -f4; -f5; -f6];
            
            deriv = mldivide(df_dp, func);
            tam1 = tam1 + 0.2*deriv(1,1);
            tam2 = tam2 + 0.2*deriv(2,1);
            tam3 = tam3 + 0.2*deriv(3,1);
            hx = hx + 0.2*deriv(4,1);
            hy = hy + 0.2*deriv(5,1);
            hz = hz + 0.2*deriv(6,1);
            [tam1; tam2; tam3; hx; hy; hz]
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
         tam1 = 1 / sqrt(solv(1));  % Radio eje X
         tam2 = 1 / sqrt(solv(3));  % Radio eje X
         tam3 = 1 / sqrt(solv(5));  % Radio eje X
         hx = solv(2)/solv(1); % Offset eje X
         hy = solv(4)/solv(3); % Offset eje Y
         hz = solv(6)/solv(5); % Offset eje Z
         elip_sol = [m n o hx hy hz]
     end
%}

    i = i+1;
end



%% FIN DEL PROGRAMA
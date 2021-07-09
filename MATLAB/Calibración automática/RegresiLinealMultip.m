%{
Entrada: Arduino [ax,ay,az,gx,gy,gz,mx,my,mz]
Salida: Amplitud y Bias; o -1(si ocurre un error)
%}
  

function ampli_bias = RegresiLinealMultip(raw) % [Amplitud Bias]
    x = raw(:,1);   y = raw(:,2);   z = raw(:,3);
    nVal = size(x,1);
    % Regresión lineal multivariable: "Ax = Y"  o bien, "SysEq*XRes = SysRes"
    SysRes = ones(nVal,1); % Resultado del sistema de ecuaciones
    SysEq = [x.^2 -2*x y.^2 -2*y z.^2 -2*z]; % Sistema de ecuaciones
    XRes = mldivide(SysEq'*SysEq, SysEq'*SysRes);
    %XRes = inv(SysEq'*SysEq)*SysEq'*SysRes;
%     [re,co] = rref([SysEq SysRes]);
%     XRes = re(1:6,7);
    
    if XRes(1:2:5) > 0
        % Obtención de parametros elípticos
        lambda = (XRes(2)^2)/XRes(1) + (XRes(4)^2)/XRes(3) + (XRes(6)^2)/XRes(5);
        p = lambda/(1+lambda);   q = 1-p;    %q = 1 - p_org;
        XRes = XRes*q;
        tam(1,1) = sqrt(1/XRes(1));   tam(1,2) = sqrt(1/XRes(3));   tam(1,3) = sqrt(1/XRes(5));
        hx = XRes(2)/XRes(1);   hy = XRes(4)/XRes(3);   hz = XRes(6)/XRes(5);
                
        ampli_bias(1,:) = [tam hx hy hz];
    else
        ampli_bias = -1;
    end
end
%



%% Independent Code (It can run by itself, it is not a function)
%{

%{
Entrada: Arduino ax,ay,az,gx,gy,gz,mx,my,mz
Salida: Graficación de la orientación del sensor por medio del 
    filtro Madgwick
%}

clear all
close all
clc

%PARÁMETROS
nVal = 20;  nIteraciones = 10;
m = 400; n = 40; o = 35; 
ehx = -200; ehy = 20; ehz = 10;  % Introducir error OFFSET en la 'data'

values_1 = ones(nIteraciones,6);
values_2 = [ones(1,3) zeros(1,3)];
values_2 = [406.234  43.572  38.548  -19.670  20.631  10.584];
values_2(1:3) = values_2(1)./values_2(1:3);
Tol = 1.08; % Tolerancia de Error Total

%p_org = (ehx/m)^2 + (ehy/n)^2 + (ehz/o)^2;
i=0; j=0;
nNegat = 0; %Contador de la cantidad de "XRes" negativos presentes en bucle While
q = 1; %Cambia con cada ciclo del While  

while i < nIteraciones
    i=i+1;
    j=j+1;
    data(:,1) = (rand(nVal,1)-0.5) * m*2; % rand() -> Valores entre 0 y 1
    data(:,2) = (rand(nVal,1)-0.5).*sqrt(1 - (data(:,1)/m).^2) * n*2;
    data(:,3) = sqrt(1 - (data(:,1)/m).^2 - (data(:,2)/n).^2) * o;
    answ = rand(nVal,1) >= 0.5;
    data(:,3) = ((-1).^answ).*data(:,3);
    %comprobar = 1 - (data(:,1)/45).^2 - (data(:,2)/40).^2 - (data(:,3)/35).^2
    x = data(:,1) + ehx + 15*(rand(nVal,1)-0.5);
    y = data(:,2) + ehy + 15*(rand(nVal,1)-0.5);
    z = data(:,3) + ehz + 15*(rand(nVal,1)-0.5);
    
        
    
%     %if ( dot(m,m_ant) / (norm(m)*norm(m_ant)) )  < 0.9659  % Menor que 15°
%     %    m_ant = m;
%     %elip_ft_mt(mod(j,20)+1,:) = m; 
%     %    fprintf('%d',j);
%     %    fr = 0.8; % fr => Forward Rate
%     %    if (mod(j,10) == 9) & (j >= 19)


%% CORRECCIÓN ELIPSE MAGNÉTICA #1
%{
    % Regresión lineal multivariable: "Ax = Y"  o bien, "SysEq*XRes = SysRes"
    SysRes = ones(nVal,1); % Resultado del sistema de ecuaciones
    SysEq = [x.^2 -2*x y.^2 -2*y z.^2 -2*z]; % Sistema de ecuaciones
    XRes = mldivide(SysEq'*SysEq, SysEq'*SysRes);
    %XRes = inv(SysEq'*SysEq)*SysEq'*SysRes;
%     [re,co] = rref([SysEq SysRes]);
%     XRes = re(1:6,7);
    
    if XRes(1:2:5) > 0
        % Obtención de parametros elípticos
        lambda = (XRes(2)^2)/XRes(1) + (XRes(4)^2)/XRes(3) + (XRes(6)^2)/XRes(5);
        p = lambda/(1+lambda);   q = 1-p;    %q = 1 - p_org;
        XRes = XRes*q;
        tam(1) = sqrt(1/XRes(1));   tam(2) = sqrt(1/XRes(3));   tam(3) = sqrt(1/XRes(5));
        hx = XRes(2)/XRes(1);   hy = XRes(4)/XRes(3);   hz = XRes(6)/XRes(5);
        
        %XresH = XRes';        

        fprintf("Val_Obtenidos:\n");
        fprintf("   m=%.3f   n=%.3f   o=%.3f \n", tam(1), tam(2), tam(3)); %= tam*(m/tam(1))
        fprintf("   hx=%.3f   hy=%.3f   hz=%.3f \n", hx, hy, hz);
        fprintf("Val_Esperados:\n");
        fprintf("   m=%.3f   n=%.3f   o=%.3f \n", m, n, o); %= tam*(m/tam(1))
        fprintf("   hx=%.3f   hy=%.3f   hz=%.3f \n\n", ehx, ehy, ehz);
        values_1(i,:) = [tam hx hy hz];
    else
        i = i - 1;
        nNegat = nNegat + 1; 
        fprintf("NEGATIVO_1\n\n");
    end
%}        


%% CORRECCIÓN ELIPSE MAGNÉTICA #2
    %capt = [x y z];  % Datos capturados (Sensor) 
    Val = ([x y z] - values_2(4:6)) .* values_2(1:3); % Valores ajustados Offset y Amplitud
    x_V = Val(:,1);   y_V = Val(:,2);   z_V = Val(:,3);
    
    SysRes = ones(nVal,1); % Resultado del sistema de ecuaciones
    SysEq = [x_V.^2 -2*x_V y_V.^2 -2*y_V z_V.^2 -2*z_V]; % Sistema de ecuaciones
    XRes = mldivide(SysEq'*SysEq, SysEq'*SysRes);
    if XRes(1:2:5) > 0 
        % Obtención de parametros elípticos
        lambda = (XRes(2)^2)/XRes(1) + (XRes(4)^2)/XRes(3) + (XRes(6)^2)/XRes(5);
        p = lambda/(1+lambda);   q = 1-p;    %q = 1 - p_org;
        XRes = XRes*q;
        tam(1) = sqrt(1/XRes(1));   tam(2) = sqrt(1/XRes(3));   tam(3) = sqrt(1/XRes(5));
        hx = XRes(2)/XRes(1);   hy = XRes(4)/XRes(3);   hz = XRes(6)/XRes(5);
        
        fprintf("Val_Obtenidos:\n");
        fprintf("   m=%.3f   n=%.3f   o=%.3f \n", tam(1), tam(2), tam(3)); %= tam*(m/tam(1))
        fprintf("   hx=%.3f   hy=%.3f   hz=%.3f \n", hx, hy, hz);
        fprintf("Val_Esperados:\n");
        fprintf("   m=%.3f   n=%.3f   o=%.3f \n", m, n, o); %= tam*(m/tam(1))
        fprintf("   hx=%.3f   hy=%.3f   hz=%.3f \n", ehx, ehy, ehz);
        values_new = [tam hx hy hz];               
                
        % Proceso de aprobación/Rechazo de los nuevos parámetros de ajuste
        valuesError = 1 - sum(( (Val-values_new(4:6))./values_new(1:3) ).^2 , 2); % valuesError = 1 - ((x-h)/m)^2 - ((y-h)/n)^2 - ((z-h)/o)^2        
        valuesRel = sqrt(1./(1-valuesError));  % Rel = (1/(1-Error))^0.5
%         Val = [x y z] - [ehx ehy ehz];
%         Param = [(n*o)/m (m*o)/n (m*n)/o];
%         valuesError = m*n*o - sum(Param.*(Val.^2), 2)
%         valuesRel = 1 - valuesError;                
        if_inv = 0 < valuesRel & valuesRel < 1;
        valuesRel = (~if_inv).*valuesRel + if_inv./valuesRel;
        
        values_TotalErr = sum(valuesRel,1) / size(Val,1)
        if_Tol = values_TotalErr < Tol;  % "1" or "0"
        if if_Tol % If TRUE, enonces se consideran parámetros validos
            Tol = 0.7*(Tol) + 0.3*(1.08);  %  = Tol - 0.4*(Tol-1.08)
            values_new(1:3) =  values_new(1)./values_new(1:3);
            values_new(4:6) = values_2(4:6) + values_new(4:6)./values_2(1:3);
            values_new(1:3) = values_2(1:3).*values_new(1:3);
%             values_Store(i_adj,:) = values_2;

            % Se mezclan los viejos y los nuevos parametros con 50% y 50% c/u
            values_2 = 0.6*values_2 + 0.4*values_new;
            fprintf('values_2 =');  fprintf('   %.3f', tam(1)./values_2(1:3));
                fprintf('   %.3f', values_2(4:6));
            fprintf('\nTol = %.3f\n\n',values_TotalErr);
        else
            fprintf("Test de TOLERANCIA de ajuste REPROBADO\n");
            Tol = Tol * 1.08;
            pause(2);
        end
    else
        j = j - 1;
        nNegat = nNegat + 1; 
        fprintf("NEGATIVO_2\n\n");
    end

    
end

% values_1
% prom = sum(values_1,1)/size(values_1,1);
% fprintf("values_1 = ");  fprintf("  %.3f", prom); fprintf("\n");
% fprintf('Obtuviste %d numero(s) NEGATIVO(S)\n', nNegat);


%values_2(1:3) = tam(1)./values_2(1:3);
fprintf("values_2_FINAL = ");  fprintf("  %.3f", values_2); fprintf("\n");

%}

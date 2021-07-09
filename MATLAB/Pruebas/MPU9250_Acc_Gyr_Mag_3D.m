clear all
close all
clc
%borrar datos previos
delete(instrfind({'Port'},{'COM3'}));
%crear objeto serie y determinar salto de linea como final de dato
s = serial('COM3','BaudRate',115200,'Terminator','CR/LF');
warning('off','MATLAB:serial:fscanf:unsuccessfulRead');
%abrir puerto
fopen(s);
% parámetros de medidas(tiempo)
 % tiempo de captura en s
rate = 60; % resultado experimental
xlabel('Eje x')
ylabel('Ejtmax = 100;e y')
zlabel('Eje z')
title('Acelerometro en tiempo real con Arduino')
xlim ([- 150 150])
ylim ([- 150 150])
zlim ([- 150 150])
grid on
hold on
% inicializar
i = 1;
t = 0;
% ejecutar bucle cronometrado
tic
continuar = 'S';
set(gca, 'CameraPosition', [150 -150 150]);
while continuar == 'S'

refresh
 t = toc;
 % leer del puerto serie
val = fscanf(s,'Ac=[%f %f %f] Gy=[%f %f %f] Mg=[%f %f %f] %d')';
ax=val(1)*15;
ay=val(2)*15;
az=val(3)*15;
mx=-val(7)*3;
my=-val(8)*3;
mz=-val(9)*3;
ro=150;
fprintf('i=%d  Ac=[%.3f %.3f %.3f]  Mg=[%.3f %.3f %.3f]\n',...
        i,ax,ay,az,mx,my,mz);

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

%% IMPRESIÓN EN PANTALLA
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
a = [ax ay az]; m = [mx my mz];
angle = acosd( dot(a,m) / (norm(a)*norm(m)) );
av = (a + m) / 2; 
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
if i > 6000
    continuar = input('¿Desea continuar? [S/N]:  ','s');
end
%pause(0.03)
delete(G); delete(Gt);
delete(M); delete(Mt);
delete(NM); delete(NMt);
delete(Avt);
delete(h1); delete(h1t);
delete(h2); delete(h2t);
delete(h3); delete(h3t);
end

%% FIN DEL PROGRAMA
% resultado del cronometro
clc;
fprintf('%g s de captura a %g cap/s \n',t,i/t);
fclose(s);
delete(s);
clear s;
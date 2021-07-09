clear all
close all
clc

u = udp('192.168.1.1',63604, 'LocalPort', 8887);
fopen(u);
%data = fscanf(u)
while true
    fwrite(u, 'Hola desde MATLAB');
end
fclose(u);
clc; clear; close all
robot;           
q3_val = 0; 
q5_val = 0; 
q7_val = 0;

% Barridos que sí afectan XY
n2 = 30;  n4 = 30;     
n6 = 9;               
q2_range = linspace(R.qlim(2,1), R.qlim(2,2), n2);
q4_range = linspace(R.qlim(4,1), R.qlim(4,2), n4);
q6_range = linspace(R.qlim(6,1), R.qlim(6,2), n6);

radial_xy = [];  
for i2 = 1:n2
    for i4 = 1:n4
        for i6 = 1:n6
            q = [0, q2_range(i2), q3_val, q4_range(i4), q5_val, q6_range(i6), q7_val];
            T = R.fkine(q);
            p = T.t' * 1000;                 % [mm]
            radial_xy = [radial_xy; p(1) p(2)];
        end
    end
end

q1_range = linspace(-pi, pi, 36);    
XY = zeros(0,2);
for iq = 1:numel(q1_range)
    c = cos(q1_range(iq)); s = sin(q1_range(iq));
    Rz = [c -s; s c];
    XY = [XY; (Rz * radial_xy.').'];  
end

figure('Name','Espacio de Trabajo - Vista Superior XY'); hold on
if exist('alphaShape','class')
    shp = alphaShape(XY(:,1), XY(:,2), 60);
    plot(shp, 'FaceColor', [0.85 0.95 0.85], 'EdgeColor', [0 0.6 0], 'LineWidth', 1.5);
else
    plot(XY(:,1), XY(:,2), '.', 'Color', [0 0.6 0], 'MarkerSize', 3);
end

plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'w', 'LineWidth', 1.5);
xline(0, '--k', 'Alpha', 0.3); 
yline(0, '--r', 'Alpha', 0.4, 'Label','Eje Y');

axis equal; grid on
xlabel('X [mm]'); ylabel('Y [mm]')
title('Espacio de Trabajo - Vista Superior XY')
xlim([-900 900]); ylim([-900 900])
hold off

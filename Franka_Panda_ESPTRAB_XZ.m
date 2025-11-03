clc, clear, close all
robot;

n2 = 50; 
n4 = 50; 
n6 = 30; 

q1_val = 0;
q2_range = linspace(R.qlim(2,1), R.qlim(2,2), n2);
q3_val = 0; 
q4_range = linspace(R.qlim(4,1), R.qlim(4,2), n4);
q5_val = 0; 
q6_range = linspace(R.qlim(6,1), R.qlim(6,2), n6);
q7_val = 0; 

q1 = [q1_val, 0, q3_val, -pi/2, q5_val, pi/2, q7_val];
%R.plot(q1,'scale',0.65,'jointdiam',0.5,'notiles','floorlevel',10,'nobase','trail',{'r', 'LineWidth', 2})
workspace_points = [];
for i2 = 1:length(q2_range)
    for i4 = 1:length(q4_range)
        for i6 = 1:3:length(q6_range)

            q = q1;
            q(2) = q2_range(i2);
            q(4) = q4_range(i4);
            q(6) = q6_range(i6);
            
            %R.animate(q)
            T = R.fkine(q);
            pos = T.t' * 1000; % En mm
            workspace_points = [workspace_points; pos];
        end
    end
end

%% Graficar vista XZ
hold on
x_points = workspace_points(:,1);
z_points = workspace_points(:,3);

if exist('alphaShape', 'class')
    shp = alphaShape(x_points, z_points, 60);
    plot(shp, 'FaceColor', [0.7 0.85 0.9], 'EdgeColor', 'b', 'LineWidth', 2)
end

plot(0, 333, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'w', 'LineWidth', 2)

yline(333, '--r', 'Alpha', 0.5, 'Label', 'Base')
xline(0, '--k', 'Alpha', 0.3)

grid on
axis equal
xlabel('X [mm]')
ylabel('Z [mm]')
title('Espacio de Trabajo - Vista Lateral XZ')
xlim([-900 900])
ylim([-200 1300])

text(0, 1250, '1190', 'HorizontalAlignment', 'center', 'FontSize', 10)
text(-855, -50, '-855', 'HorizontalAlignment', 'center', 'FontSize', 10)
text(855, -50, '855', 'HorizontalAlignment', 'center', 'FontSize', 10)

hold off

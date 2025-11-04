%Archivo robot.m
clc, clear, close all

dh = [
    % theta    d       a        alpha
    0        0.333    0         0       
    0        0        0        -pi/2    
    0        0.316    0         pi/2    
    0        0        0.0825    pi/2   
    0        0.384   -0.0825   -pi/2    
    0        0        0         pi/2    
    0        0        0.088     pi/2    
];

L = Link.empty(0,7);
L(1) = Link('d', 0.333, 'a', 0, 'alpha', 0, 'modified');
L(2) = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'modified');
L(3) = Link('d', 0.316, 'a', 0, 'alpha', pi/2, 'modified');
L(4) = Link('d', 0, 'a', 0.0825, 'alpha', pi/2, 'modified');
L(5) = Link('d', 0.384, 'a', -0.0825, 'alpha', -pi/2, 'modified');
L(6) = Link('d', 0, 'a', 0, 'alpha', pi/2, 'modified');
L(7) = Link('d', 0, 'a', 0.088, 'alpha', pi/2, 'modified');

R = SerialLink(L, 'name','Franka Panda Emika');

R.tool = trotz(pi/4) * transl(0,0,0.1034) * transl(0,0,0.107);

R.qlim(1,1:2) = [-166,  166]*pi/180;
R.qlim(3,1:2) = [-166,  166]*pi/180;
R.qlim(5,1:2) = [-166,  166]*pi/180;
R.qlim(7,1:2) = [-166,  166]*pi/180;
R.qlim(2,1:2) = [-101,  101]*pi/180;
R.qlim(4,1:2) = [-176,  -4]*pi/180;
R.qlim(6,1:2) = [-1,    215]*pi/180;
R.base = transl(0, 0, 0) * trotx(0);  
R.offset = [0, 0, 0, 0, 0, 0, 0]; 

workspace = [-1 1 -1 1 -0.5 1.5];

%% ========= PRUEBA DE IK (Panda 7-DoF, q7 fijado) =========
q_true = [0.0, -0.4, 0.0, -1.5, 0.0, 1.2, 0.4];   % postura típica
disp('T_EE')
T_EE   = R.fkine(q_true).double                  % objetivo en la punta

q0         = q_true;
q7_fijado  = q_true(7);
mejor      = true;     % <<--- cambia a false si querés TODAS las soluciones

Q = cin_inv_panda(R, T_EE, q0, mejor, q7_fijado);   % [7xN] o [7x1]

if isempty(Q)
    error('cin_inv_panda no encontró soluciones para esta pose/q7.');
end

% Mostrar errores de FK
if isvector(Q)
    q = Q(:);
    Tchk = R.fkine(q.').double;
    Rerr = Tchk(1:3,1:3)' * T_EE(1:3,1:3);
    ang_err = acos( max(-1,min(1,(trace(Rerr)-1)/2)) );
    pos_err = norm(Tchk(1:3,4) - T_EE(1:3,4));
    fprintf('Única sol: |Δp|=%.3g m, Δang=%.3g rad\n', pos_err, ang_err);
else
    fprintf('\nSe obtuvieron %d soluciones.\n', size(Q,2));
    for k = 1:size(Q,2)
        Tchk = R.fkine(Q(:,k).').double;
        Rerr = Tchk(1:3,1:3)' * T_EE(1:3,1:3);
        ang_err = acos( max(-1,min(1,(trace(Rerr)-1)/2)) );
        pos_err = norm(Tchk(1:3,4) - T_EE(1:3,4));
        fprintf('Sol %d:  |Δp|=%.3g m,  Δang=%.3g rad\n', k, pos_err, ang_err);
    end
    q = Q(:,1); % elegí la que quieras para graficar
end

T_encontrada  = R.fkine(q.').double
% compara T_EE con T_encontrada
Tdiff = T_EE - T_encontrada;

tol = 1e-12;                         % umbral (ajustalo si hace falta)
Tclean = Tdiff;
Tclean(abs(Tclean) < tol) = 0;       % pone a 0 los valores pequeñitos

disp('Resta entre matrices (con tolerancia):')
Tclean

% si querés un booleano de “todo OK”:
ok = all(abs(Tdiff) < tol, 'all');   % true si todo está dentro de tol
if ok == true
    fprintf('Error 0');
end

R.plot(q','workspace', workspace,'scale',0.65,'jointdiam',0.5,'notiles','floorlevel',0,'nobase','trail',{'r', 'LineWidth', 2})
title('IK Panda - solución');
hold on
plot3(T_EE(1,4),T_EE(2,4),T_EE(3,4),'*k','markersize',40,'LineWidth',1)

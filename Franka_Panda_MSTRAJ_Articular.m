clc, clear, close all
robot;

rtb = 'C:\Users\Juan\AppData\Roaming\MathWorks\MATLAB Add-Ons\Toolboxes\Robotics Toolbox for MATLAB';

addpath('-begin', fullfile(rtb,'lib','spatial-math'));
addpath('-begin', rtb);
rehash toolboxcache

figure(1);
q = zeros(5,7);
dx = 0.85;    
dy = -0.12;   
dz = 0.20;      

Rtool  = troty(pi);
Rtool2 = Rtool * troty(-pi/2);

T0 = transl(-0.40+dx, 0.1, 0.30+dz) * Rtool;
q0 = cin_inv_panda(R, T0, q(1,:), true, 0);

R.plot(q0','workspace',workspace,'scale',0.001,'jointdiam',0.5, ...
    'notiles','floorlevel',0,'nobase','trail',{'r','LineWidth',2})

ax = gca; hold(ax,'on');

V = draw_cube(ax, [-0.50+dx, 0.25+dy, 0.15+dz], 0.20, ...
    'Color',[0.8 0.8 1],'Alpha',0.3);
draw_layout_celda(ax, V, R);


xc = -0.50 + dx;  yc = 0.25 + dy;  zc = 0.15 + dz;
L = 0.20;  h = L/2;
xmin = xc - h;  xmax = xc + h;
ymin = yc - h;  ymax = yc + h;
zmin = zc - h;
ztop = zc + h;

T2  = transl(-0.40+dx, 0.35+dy, ztop)*Rtool;   
T3  = transl(-0.40+dx, 0.15+dy, ztop)*Rtool;
T4  = transl(-0.45+dx, 0.15+dy, ztop)*Rtool;
T5  = transl(-0.45+dx, 0.35+dy, ztop)*Rtool;
T6  = transl(-0.50+dx, 0.35+dy, ztop)*Rtool;
T7  = transl(-0.50+dx, 0.15+dy, ztop)*Rtool;
T8  = transl(-0.55+dx, 0.15+dy, ztop)*Rtool;
T9  = transl(-0.55+dx, 0.35+dy, ztop)*Rtool;
T10 = transl(-0.60+dx, 0.35+dy, ztop)*Rtool;
T11 = transl(-0.60+dx, 0.15+dy, ztop)*Rtool;

Rtool_yneg = Rtool * trotx(-pi/2);  

yS   = ymin;
xA   = xmin; 
xB   = xmax;
stepZ = 0.05;

T12 = transl(-0.60+dx, yS - 0.08, ztop) * Rtool_yneg;

T13 = transl(xA, yS, ztop)            * Rtool_yneg;
T14 = transl(xA, yS, ztop-stepZ)      * Rtool_yneg;
T15 = transl(xB, yS, ztop-stepZ)      * Rtool_yneg;
T16 = transl(xB, yS, ztop-2*stepZ)    * Rtool_yneg;
T17 = transl(xA, yS, ztop-2*stepZ)    * Rtool_yneg;
T18 = transl(xA, yS, ztop-3*stepZ)    * Rtool_yneg;
T19 = transl(xB, yS, ztop-3*stepZ)    * Rtool_yneg;
T20 = transl(xB, yS, zmin)            * Rtool_yneg;
T21 = transl(xA, yS, zmin)            * Rtool_yneg;

TT1 = cat(3, T0, T2,T3,T4,T5,T6,T7,T8,T9,T10,T11, ...
              T12,T13,T14,T15,T16,T17,T18,T19,T20,T21, ...
              T12, T0);


for k = 2:size(TT1,3)
    p = transl(TT1(:,:,k));
    plot3(p(1),p(2),p(3),'*k','MarkerSize',5);
    text(p(1)+0.01,p(2)+0.01,p(3),sprintf('T%d',k),'FontSize',7);
end
set(ax,'Units','normalized');
posRobot = get(ax,'Position');
axis(ax,'manual');

axBar = axes('Units','normalized','Position',[0.10 0.05 0.80 0.03]);
hold(axBar,'on');
barra = fill([0 0 0 0],[0 1 1 0],'g','EdgeColor','none','Parent',axBar);
xlim(axBar,[0 1]); ylim(axBar,[0 1]);
set(axBar,'XTick',0:0.25:1,'XTickLabel',{'0','25','50','75','100'}, ...
          'YTick',[],'Box','on','Color',[0.9 0.9 0.9]);
xlabel(axBar,'|det(J)| normalizado [%]');
texto = text(0.5,0.5,'|det(J)|: 0.0 %','HorizontalAlignment','center', ...
             'VerticalAlignment','middle','FontWeight','bold','Parent',axBar);
set(ax,'Position',posRobot);
axes(ax);

mejor = true;
q7fijo = q0(7);
Qvias = zeros(size(TT1,3), 7);
qcurr = q0(:).';
for k = 1:size(TT1,3)
    QQ = cin_inv_panda(R, TT1(:,:,k), qcurr, mejor, q7fijo); % tus IK
    qsol = QQ(:,1).';          
    Qvias(k,:) = qsol;
    qcurr = qsol;             
end


dt   = 0.1;           
tacc = 0.5;           
vj  = deg2rad([90 90 90 120 120 150 150]);  
Qvias = unwrap(Qvias, [], 1);  

Qtraj = mstraj(Qvias(2:end,:), vj, [], Qvias(1,:), dt, tacc); 
Qvias = unwrap(Qvias, [], 1);      


[~,~,files] = fileparts(mfilename('fullpath'));
thisDir = fileparts(mfilename('fullpath'));
stlFiles = dir(fullfile(thisDir, 'pulidora.stl'));

if isempty(stlFiles)
    error('No se encontró ningún archivo .stl en el directorio actual (%s)', thisDir);
else
    TOOL_STL = fullfile(stlFiles(1).folder, stlFiles(1).name);
    fprintf('Usando STL: %s\n', TOOL_STL);
end

T_mount  = transl(0,-0.05,0.10) * trotx(-pi/2) * trotz(pi);
toolScale = [1 1 1];                    

hURDF = panda_skin('init', ax, TOOL_STL, T_mount, toolScale);  
Qgraf = zeros(7, size(Qtraj,1));

for i = 1:size(Qtraj,1)
    q = Qtraj(i,:);
    J = R.jacob0(q);
    detJ = abs(det(J(1:6,1:6)));
    val = min(detJ/0.1, 1); 

    R.animate(q); panda_skin(q,[]);
    set(barra,'XData',[0 val val 0], 'YData',[0 0 1 1]);
    set(texto,'String',sprintf('|det(J)|: %.1f %%',val*100));

    Qgraf(:,i) = q(:);
    drawnow;
end

vel_acel(Qgraf')


function hRoot = panda_skin(arg, axIn, TOOL_STL, T_mount, toolScale)
persistent panda cfg ax S perm initialized lim0 dasp0 hURDF

if nargin < 3, TOOL_STL = ''; end
if nargin < 4, T_mount  = eye(4); end
if nargin < 5, toolScale = [1 1 1]; end

if ischar(arg) && strcmpi(arg,'init')
    % --- Importar URDF (parche rutas) ---
    currentFolder = fileparts(mfilename('fullpath'));
    base = fullfile(currentFolder, 'franka_description');
    d = dir(fullfile(base,'robots','*.urdf'));
    assert(~isempty(d),'No encontré ningún .urdf en: %s', fullfile(base,'robots'));
    urdfFile = fullfile(d(1).folder, d(1).name);

    txt = fileread(urdfFile);
    baseNorm = strrep(base,'\','/');
    txt = strrep(txt,'package://franka_description', baseNorm);
    [bs,be] = regexp(txt,'<visual>[\s\S]*?</visual>');
    txtPatched = txt;
    for k = numel(bs):-1:1
        block = txtPatched(bs(k):be(k));
        tok = regexp(block,'mesh filename="([^"]+)"','tokens','once');
        if isempty(tok), continue; end
        visPath = tok{1};
        if exist(strrep(visPath,'/','\'),'file')~=2
            [folder,fname,~] = fileparts(visPath);
            visFolder   = strrep(folder,'/visual','/collision');
            colPathURDF = [visFolder '/' fname '.stl'];
            block = regexprep(block,'mesh filename="[^"]+"', ...
                              ['mesh filename="' colPathURDF '"']);
            txtPatched = [txtPatched(1:bs(k)-1) block txtPatched(be(k)+1:end)];
        end
    end
    tmpURDF = fullfile(tempdir,'panda_local_patched.urdf');
    fid=fopen(tmpURDF,'w'); fwrite(fid,txtPatched); fclose(fid);

    panda = importrobot(tmpURDF,'DataFormat','row');

    % --- Agregar STL al efector ---
    if ~isempty(TOOL_STL) && isfile(TOOL_STL)
        handCandidates = {'panda_hand','hand','tool0','flange','panda_link8'};
        eeName = '';
        for c = 1:numel(handCandidates)
            if any(strcmpi(panda.BodyNames, handCandidates{c})), eeName = handCandidates{c}; break; end
        end
        if isempty(eeName), eeName = panda.BodyNames{end}; end

        rbTool = rigidBody('polisher_tool');
        jTool  = rigidBodyJoint('polisher_fix','fixed');
        setFixedTransform(jTool, T_mount);
        rbTool.Joint = jTool;

        try, addVisual(rbTool,"Mesh",TOOL_STL,toolScale);
        catch, addVisual(rbTool,"Mesh",TOOL_STL);
        end
        addBody(panda, rbTool, eeName);
    end

    % --- Config inicial ---
    cfg = homeConfiguration(panda);
    ji = 1;
    for i = 1:numel(panda.Bodies)
        j = panda.Bodies{i}.Joint;
        if ~strcmpi(char(j.Type),'fixed')
            lim = j.PositionLimits;
            if ~isempty(lim) && all(isfinite(lim(:))), cfg(ji) = mean(lim); end
            ji = ji + 1;
        end
    end
    S = [1 1 1 1 1 1 1]; perm = 1:7;

    % --- Mostrar sin frames (no borra nada) ---
    ax = axIn;
    daspect(ax,[1 1 1]);
    lim0  = [xlim(ax); ylim(ax); zlim(ax)];
    dasp0 = daspect(ax); axis(ax,'manual');

    try
        hURDF = show(panda, cfg, 'Parent', ax, ...
            'Visuals','on','Frames','off', ...
            'PreservePlot', false, 'FastUpdate', true);
    catch
        hURDF = show(panda, cfg, 'Parent', ax, ...
            'Visuals','on', ...
            'PreservePlot', false, 'FastUpdate', true);
    end

    xlim(ax, lim0(1,:)); ylim(ax, lim0(2,:)); zlim(ax, lim0(3,:));
    daspect(ax, dasp0);

    initialized = true;
    if nargout, hRoot = hURDF; end
    return
end

% --- Actualización (animate) ---
if ~initialized || isempty(arg)
    if nargout, hRoot = hURDF; end
    return
end
qrow = arg;
qurdf = (qrow(perm) .* S);
cfg(:) = qurdf;

try
    hURDF = show(panda, cfg, 'Parent', ax, ...
        'Visuals','on','Frames','off', ...
        'PreservePlot', false, 'FastUpdate', true);
catch
    hURDF = show(panda, cfg, 'Parent', ax, ...
        'Visuals','on', ...
        'PreservePlot', false, 'FastUpdate', true);
end

xlim(ax, lim0(1,:)); ylim(ax, lim0(2,:)); zlim(ax, lim0(3,:));
daspect(ax, dasp0);
if nargout, hRoot = hURDF; end
end



function V = draw_cube(ax, centro, lado, varargin)
p = inputParser;
p.addParameter('Color', [0.8 0.8 0.8]);
p.addParameter('Alpha', 0.4);
p.addParameter('Label', 'Cubo');
p.parse(varargin{:});
col  = p.Results.Color;
alph = p.Results.Alpha;
txt  = p.Results.Label;

[xc, yc, zc] = ndgrid([-1 1]*lado/2, [-1 1]*lado/2, [-1 1]*lado/2);
V = [xc(:), yc(:), zc(:)] + centro;

faces = [
    1 3 7 5;  % cara +X
    2 4 8 6;  % cara -X
    1 2 6 5;  % cara -Y
    3 4 8 7;  % cara +Y
    5 6 8 7;  % cara +Z (superior)
    1 2 4 3   % cara -Z (inferior)
];

patch('Parent', ax, ...
      'Vertices', V, 'Faces', faces, ...
      'FaceColor', col, 'FaceAlpha', alph, ...
      'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 0.8);

fprintf('\nCoordenadas de las esquinas del cubo (x y z):\n');
disp(round(V,4));
end

function graf = vel_acel(q)

    [N, ~] = size(q);
    Ts = 0.08;
    t = (0:N-1)'*Ts;
    qd  = zeros(N,size(q,2));
    qdd = zeros(N,size(q,2));
    for c = 1:size(q,2)
        qd(:,c)  = gradient(q(:,c),  Ts);
        qdd(:,c) = gradient(qd(:,c), Ts);
    end

    figure(2); clf
    subplot(3,1,1); qplot(t,q);   grid on; ylabel('q [rad]');     title('Articulares')
    subplot(3,1,2); qplot(t,qd);  grid on; ylabel('q̇ [rad/s]')
    subplot(3,1,3); qplot(t,qdd); grid on; ylabel('q̈ [rad/s²]'); xlabel('t [s]')

    R = evalin('base','R');
    p = zeros(N,3);
    for k = 1:N
        T = R.fkine(q(k,:));
        p(k,:) = transl(T);    
    end

    pd  = zeros(N,3);
    pdd = zeros(N,3);
    for c = 1:3
        pd(:,c)  = gradient(p(:,c),  Ts);
        pdd(:,c) = gradient(pd(:,c), Ts);
    end

    figure(3); clf
    subplot(3,1,1); plot(t,p,'LineWidth',1.2);   grid on
    ylabel('pos [m]'); title('Variables cartesianas')
    legend({'x','y','z'},'Location','best')
    subplot(3,1,2); plot(t,pd,'LineWidth',1.2);  grid on
    ylabel('vel [m/s]'); legend({'ẋ','ẏ','ż'},'Location','best')
    subplot(3,1,3); plot(t,pdd,'LineWidth',1.2); grid on
    ylabel('acel [m/s^2]');legend({'ẍ','ÿ','z̈'},'Location','best')
end

function P = Tstack2pose(Tstack, unit)

    if nargin < 2, unit = 'rad'; end
    N = size(Tstack,3);
    P = NaN(N,6);

    for i = 1:N
        Ti = Tstack(:,:,i);
        assert(isequal(size(Ti),[4 4]), 'T(:,:,i) debe ser 4x4');

        p   = transl(Ti);    p   = p(:).';       
        rpy = tr2rpy(Ti);    rpy = rpy(:).';    

        if strcmpi(unit,'deg'), rpy = rad2deg(rpy); end
        P(i,:) = [p rpy];                       
    end
end

function draw_layout_celda(ax, V, R)

hold(ax,'on');

xc = mean(V(:,1));
yc = mean(V(:,2));
zc = mean(V(:,3));
L  = norm(V(1,:)-V(2,:));   

floorLx = 1.2; floorLy = 1.2; floorH = 0.015;
draw_cuboid(ax, [0 0 -floorH/2], floorLx, floorLy, floorH, ...
    'FaceColor',[0.96 0.96 0.96], 'EdgeColor','none');

beltW = 0.18;          
beltL = 0.80;         
beltH = 0.07;          
y_end = yc - L/2;      
beltCy = y_end + beltL/2;            
beltTopZ = zc - L/2;               
beltCz   = beltTopZ - beltH/2;      
beltCx   = xc;                        

draw_cuboid(ax, [beltCx, beltCy, beltCz], beltW, beltL, beltH, ...
    'FaceColor',[0.75 0.85 0.75], 'EdgeColor','none', 'FaceAlpha',0.95);
deckH = 0.01;
draw_cuboid(ax, [beltCx, beltCy, beltTopZ - deckH/2], beltW*0.98, beltL*0.98, deckH, ...
    'FaceColor',[0.40 0.40 0.40], 'EdgeColor','none');

text(beltCx, y_end + 0.02, beltTopZ + 0.015, 'Cinta', ...
    'HorizontalAlignment','center','FontSize',12,'Color',[0.2 0.2 0.2]);

rb = transl(R.base); 
xb = rb(1); yb = rb(2);

wallH = 0.55; wallT = 0.015;

% pared larga paralela a X, detrás del robot
draw_cuboid(ax, [xb-0.15, yb+0.55, wallH/2], 0.8, wallT, wallH, ...
    'FaceColor',[0.92 0.92 0.95], 'EdgeColor','none', 'FaceAlpha',0.95);
% pared corta perpendicular
draw_cuboid(ax, [xb-0.55, yb, wallH/2], wallT, 1.10, wallH, ...
    'FaceColor',[0.92 0.92 0.95], 'EdgeColor','none', 'FaceAlpha',0.95);

    function draw_cuboid(axh, center, Lx, Ly, H, varargin)
        cx=center(1); cy=center(2); cz=center(3);
        x = [-Lx/2 Lx/2]; y = [-Ly/2 Ly/2]; z = [-H/2 H/2];
        [X,Y,Z] = ndgrid(x,y,z);
        Vc = [X(:)+cx, Y(:)+cy, Z(:)+cz];
        F = [1 3 4 2; 5 6 8 7; 1 2 6 5; 3 7 8 4; 1 5 7 3; 2 4 8 6];
        patch('Parent',axh,'Vertices',Vc,'Faces',F, ...
              'FaceColor',[0.9 0.9 0.9],'EdgeColor',[0.5 0.5 0.5], ...
              'FaceAlpha',0.9, varargin{:});
    end
end


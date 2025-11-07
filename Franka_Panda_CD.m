clc, clear, close all
robot;
q = [deg2rad(0),deg2rad(0),deg2rad(0),deg2rad(0),deg2rad(0),deg2rad(0),0];
sistemas = [0,0,0,0, 0,0, 0];
R.plot(q,'workspace', workspace,'nojoints','scale',0.5,'jointdiam',0.5,'notiles','floorlevel',0,'nobase','trail',{'r', 'LineWidth', 2})
ax = gca; hold(ax,'on');

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
panda_skin(q,[]);
%R.teach()
hold on
for i = 1:length(sistemas)
    if sistemas(i) == 1
        T = R.base;
        for j = 1:i-1
            T = T * R.links(j).A(q(j));
        end
        trplot(T, 'frame', ['{' num2str(i-1) '}'], 'length', 0.8);
    end
end

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
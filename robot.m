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


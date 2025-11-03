clc, clear, close all
q4 = deg2rad(-45);
q = [0 0 0 q4 0 0 0];
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
J = (R.jacob0(q));
disp('Jacobiano')
disp(J)

Jr = J(:,1:6)        
DJr = (det(Jr));
fprintf('det(J) = ')
disp(DJr)
disp('Rank:')
disp(rank(J))
disp('Singu')
jsingu(J)
    

 
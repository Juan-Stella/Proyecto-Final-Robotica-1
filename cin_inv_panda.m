function Q = cin_inv_panda(R, T_EE, q0, mejor, q7_fijado)

if nargin < 5 || isempty(q7_fijado), q7_fijado = (nargin>=3 && ~isempty(q0))*q0(7); end
if nargin < 4 || isempty(mejor),     mejor     = false; end
if nargin < 3 || isempty(q0),        q0        = zeros(1,7); end

% Constantes que vamos a usar, para no repetir
d1 = R.links(1).d;      % 0.333
d3 = R.links(3).d;      % 0.316
a4 = R.links(4).a;      % 0.0825
a5 = R.links(5).a;      % -0.0825
d5 = R.links(5).d;      % 0.384
a7 = R.links(7).a;      % 0.088
d7e = 0.2104;           % 0.107 + 0.1034

% datos del EE y desacople de tool
T_tool = trotz(pi/4) * transl(0,0,0.1034) * transl(0,0,0.107);
T7     = T_EE * invHomog(T_tool);   %Referenciamos a sistema 7

R7 = T7(1:3,1:3);
p7 = T7(1:3,4);

% reconstruir {6} desde {7} con q7 fijado
x6 = R7 * [cos(-q7_fijado); sin(-q7_fijado); 0];   x6 = x6/norm(x6);
p6 = p7 - a7*x6;

% construir R6 ortonormal
z7 = R7(:,3);
Z6 = cross(z7, x6);  Z6 = Z6/norm(Z6);
Y6 = cross(Z6, x6);  Y6 = Y6/norm(Y6);
Z6 = cross(x6, Y6);  Z6 = Z6/norm(Z6);
R6 = [x6, Y6, Z6];

% triángulo O2-O4-O6
p2  = [0;0;d1];
V26 = p6 - p2;
L26 = norm(V26);
L24 = hypot(a4, d3);
L46 = hypot(abs(a5), d5);

if (L24 + L46 < L26) || (L24 + L26 < L46) || (L26 + L46 < L24)
    Q = [];  return
end

clip = @(x) max(-1,min(1,x));  % robustecer acos/asin (fue una prueba cuando el error daba más grande)

% q4
theta246 = acos( clip((L24^2 + L46^2 - L26^2)/(2*L24*L46)) );
thetaH46 = atan2(d5, abs(a5));
theta342 = atan2(d3, a4);
q4 = theta246 + thetaH46 + theta342 - 2*pi;

% q6
theta462 = acos( clip((L26^2 + L46^2 - L24^2)/(2*L26*L46)) );
theta46H = atan2(abs(a5), d5);
theta26H = theta46H + theta462;
D26      = -L26 * cos(theta26H);

V_6_62 = R6.' * (-V26);                 % Expreso el vector respecto al sistema 6
phi6    = atan2(V_6_62(2), V_6_62(1));
rho     = hypot(V_6_62(1), V_6_62(2));
Theta6  = asin( clip(D26/max(rho,eps)) );
q6_list = [pi - Theta6 - phi6,  Theta6 - phi6];

% Conjunto de 4 soluciones
qq = zeros(0,7);

for q6 = q6_list
    z56 = [sin(q6); cos(q6); 0];
    z5  = R6 * z56;  z5 = z5/norm(z5);

    thetaP26 = 3*pi/2 - theta462 - theta246 - theta342;
    thetaP   = pi - thetaP26 - theta26H;
    LP6      = L26 * sin(thetaP26) / sin(thetaP);

    V2P = p6 - LP6*z5 - p2;   L2P = norm(V2P);

    % q1, q2 (dos ramas)
    if abs(V2P(3)/L2P) > 0.999
        q1c = [0, 0];
        q2c = [0, 0];
    else
        q1c = [atan2(V2P(2),V2P(1)), 0];
        q2c = [acos( clip(V2P(3)/L2P) ), 0];
        if q1c(1) < 0, q1c(2) = q1c(1) + pi; else, q1c(2) = q1c(1) - pi; end
        q2c(2) = -q2c(1);
    end

    for k = 1:2
        q1 = q1c(k);  q2 = q2c(k);

        z3 = V2P/L2P;
        y3 = -cross(V26, V2P);  y3 = y3/norm(y3);
        x3 =  cross(y3, z3);    x3 = x3/norm(x3);

        % q3
        c1 = cos(q1); s1 = sin(q1);
        c2 = cos(q2); s2 = sin(q2);
        R1  = [ c1 -s1  0;  s1  c1  0;  0  0  1];
        R12 = [ c2 -s2  0;  0   0  1; -s2 -c2  0];
        R2  = R1 * R12;

        x3_2 = R2.' * x3;
        q3   = atan2(x3_2(3), x3_2(1));   % (z,x)

        % q5
        VH4  = p2 + d3*z3 + a4*x3 - p6 + d5*z5;
        c6 = cos(q6);  s6 = sin(q6);
        R56 = [ c6 -s6  0;  0 0 -1;  s6  c6  0 ];
        R5  = R6 * R56.';    
        V5H4 = R5.' * VH4;
        q5   = -atan2(V5H4(2), V5H4(1));

        qq(end+1,:) = [q1 q2 q3 q4 q5 q6 q7_fijado];
    end
end

qq = qq.';   % 7 x N

% --- Selección "q_mejor" ---
if mejor
   
    if isrow(q0), q0 = q0(:); end                 % a columna
    N = size(qq, 2);

    % diferencia angular elemento a elemento
    D = atan2( sin(qq - q0*ones(1,N)), cos(qq - q0*ones(1,N)) );  % 7xN

    % norma-2 por columna y elección
    [~, pos] = min( vecnorm(D, 2, 1) );
    Q = qq(:, pos);                                % 7x1 (la más cercana a q0)
else
    Q = qq;                                        % 7xN (todas)
end

end

function iT = invHomog(T)
iT = eye(4);
iT(1:3, 1:3) = T(1:3, 1:3)';
iT(1:3, 4)   = - iT(1:3, 1:3) * T(1:3, 4);
end


clear all
close all
%Supongamos valores para las constantes:
const = 0.027 / (0.027 + 4.32e-6 / 0.02^2);
g = 9.81;

a = [ 0  1   0  0;
      0  0  const*g 0;
      0  0   0  1;
      0  0   0  0 ];

b = [ 0;0;0;1];



c = [ 0 0 1 0];


d = zeros(4,1);



% Sistema aumentado con integral en X
Aa = [a, zeros(4,1);
      -[1 0 0 0], 0];
Ba = [b; 0];

% Verificamos controlabilidad
fprintf('size(Aa)=%d, rank(ctrb)=%d\n', size(Aa,1), rank(ctrb(Aa,Ba)));

% Diseñamos controlador LQI
Q = diag([10 10 1 1 100]);  % peso fuerte al integrador
R = 1;
K = lqr(Aa, Ba, Q, R);

% Separar ganancias proporcional e integral
Kp = K(:, 1:4);
Ki = K(:, 5);

fprintf('Kp = \n'); disp(Kp);
fprintf('Ki = \n'); disp(Ki);



% --- Datos del sistema (ya definidos por ti) ---
% a (4x4), b (4x1), c_x = [1 0 0 0]
% Kp (1x4), Ki (scalar) obtenidos antes.

c_x = [1 0 0 0];
Kp = [15.6179, 11.6960, 33.7431, 8.2756];
Ki = -10.0000;

% Construcción de sistema aumentado lazo cerrado:
A11 = a - b * Kp;        % 4x4
A12 = -b * Ki;           % 4x1
A21 = -c_x;              % 1x4
A22 = 0;                 % 1x1

A_cl = [A11, A12;
        A21, A22];       % 5x5

B_cl = [zeros(4,1);
        1];              % entrada = referencia x_ref (because z_dot = x_ref - c_x*x)

% simulación: respuesta a paso en x_ref
tfinal = 5; dt = 0.001;
t = 0:dt:tfinal;
x0 = zeros(4,1); z0 = 0;
X = [x0; z0];

x_ref = 500; % 1 cm paso
Xhist = zeros(5, length(t));
uhist = zeros(1, length(t));

for k = 1:length(t)
    % dinámicas continuas integradas por Euler (suficiente para ver comportamiento)
    Xdot = A_cl * X + B_cl * x_ref;
    X = X + dt * Xdot;
    Xhist(:,k) = X;
    % calcula u = -Kp*x - Ki*z
    x_state = X(1:4);
    z_state = X(5);
    u = -Kp * x_state - Ki * z_state;
    uhist(k) = u;
end

% plots
figure;
subplot(3,1,1);
plot(t, Xhist(1,:)); grid on;
xlabel('t (s)'); ylabel('x (m)'); title('Posición X');

subplot(3,1,2);
plot(t, uhist); grid on;
xlabel('t (s)'); ylabel('u'); title('Señal de control u');

subplot(3,1,3);
plot(t, Xhist(5,:)); grid on;
xlabel('t (s)'); ylabel('z'); title('Estado integrador z');

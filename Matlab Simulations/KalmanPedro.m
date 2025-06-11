clear; close all; clc

% Configuracoes da simulacao
kpassos = 200;

% Criando matrizes do modelo
Phi = [1 0.1;
       0 1];
Gamma = [0.005;
        0.1];

C = [1 0];

G = [1; 1];
 
% Variancias dos ruidos
Rv = 1; % ruido de saida
Rw = 0.01; % ruido de estado

% Escolhendo uma realimentacao de estados
K_cont = place(Phi, Gamma, [0.9 0.91]);

% Scopes
x_real = zeros(2, kpassos);
y_real = zeros(1, kpassos); % saida medida
x_hat = zeros(2, kpassos);
u = zeros(1, kpassos);
xp = [0;0];

% Condicao inicial
x_real(:, 2) = [5; 0];

% Inicializando matriz de cov
P_minus = eye(2);

% laco de controle
for k = 2:kpassos
    
    %% Leitura do sensor
    y_real(:, k) = C*x_real(:, k) + randn(1)*0.1;
    
    %% Fase de correcao do FK
    K_kal = P_minus*C' / (C*P_minus*C' + Rv);
    
    x_hat(:, k) = xp + K_kal*(y_real(:, k) - C*xp);
    
    %% Calculando a acao de controle
    u(:, k) = -K_cont*x_hat(:, k);
    
    %% Simulando a din. da planta
    x_real(:, k + 1) = Phi*x_real(:, k) + Gamma*u(:, k) + G*randn(1)*0.00;
    
    %% Fase de predicao do FK
    P_plus = (eye(2) - K_kal*C)*P_minus;
   
    P_minus = Phi*P_plus*Phi' + G*Rw*G';
    
    xp = Phi*x_hat(:, k) + Gamma*u(:, k); % estado predito
end


figure
subplot(1,2,1)
plot(x_real(1,:),'b','LineWidth', 2)
hold on, grid on
plot(x_hat(1,:),'r--','LineWidth', 2)

subplot(1,2, 2)
plot(x_real(2,:),'b','LineWidth', 2)
hold on, grid on
plot(x_hat(2,:),'r--','LineWidth', 2)
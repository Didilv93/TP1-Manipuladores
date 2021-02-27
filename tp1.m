clear
close all
clc

l1=0.29;
l2=0.27;
l3=0.07;
l4=0.302;
l5=0.072;

L(1)=Link([0 l1 0 -pi/2]);
L(2)=Link([0 0 l2 0]);
L(3)=Link([0 0 l3 -pi/2]);
L(4)=Link([0 l4 0 pi/2]);
L(5)=Link([0 0 0 -pi/2]);
L(6)=Link([0 l5 0 0]);

irb120=SerialLink(L,'name','IRB120');

qdot_lim = pi*[25/18 25/18 25/18 16/9 7/3];

% Controle

thetad = [.380 .380 .500]; % Define configuração desejada
Rd = SO3(); % Pega o objeto SO3 correspondente à rotação do efetuador
Rd = Rd.R; %Pega matriz de rotação do efetuador
Td = SE3(Rd, thetad);
rpyd = rotm2eul(Rd);

K = 1; % Define ganho
epsilon = 1e-5; % Define critério de parada
e_ant = 1;
e = 0; 


i = 0;
theta = rad2deg([0 0 0 0 -pi/2 0]'); % Define configuração inicial do robô
figure(1)
irb120.plot(theta'); % Plot robô na configuração inicial
hold on
Td.plot('rgb') % Plot pose desejada

while (norm(e - e_ant) > epsilon) % Critério de parada
    i = i+1; % contador
    J = irb120.jacob0(theta, 'rpy'); % Jacobiana geométrica
    T = irb120.fkine(theta); % Cinemática direta para pegar a pose do efetuador 
    p = transl(T); % Translação do efetuador
    R = SO3(T); 
    R = R.R(); % Extrai rotação do efetuador
    rpy = rotm2eul(R);
    
    p_err = thetad-p; % Erro de translação
    rpy_err = rpyd - rpy;
    
    e_ant = e;
    e = [p_err'; rpy_err']; % Vetor de erro
    
    u = inv(J) * K * e; % Lei de controle

    theta = theta + 0.1*u; % Cálculo de theta (Regra do trapézio)
    
    irb120.plot(theta');
    control_sig(:,i) = u; % Sinal de controle
    err(i) = norm(e); % Norma do erro
    norm(e)
end
hold off

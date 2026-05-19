# Robotics Project
clear; clc; close all;

%% 1. Inizializzazione del Robot
% Carichiamo il modello del Panda
panda = loadrobot("frankaEmikaPanda", "DataFormat", "column");
ee_name = 'panda_hand'; % Nome del frame dell'end-effector

%% 2. Parametri di Simulazione e Traiettoria
dt = 0.005; % Passo di integrazione molto fine per la stabilità in 3D
T_traj = 3.0; % La traiettoria deve essere eseguita in 1 secondo 
t = 0:dt:T_traj;
N = length(t);

r = 0.1; % Raggio del cerchio 
omega = 2*pi / T_traj;

% Guadagni del controllore (ora abbiamo 6 gradi di libertà nel task: 3 rot, 3 trasl)
K = 20 * eye(6); 

%% 3. Configurazione Iniziale
q = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4]; % I nostri 7 giunti di controllo

% Creiamo una versione "paddata" a 9 giunti aggiungendo due zeri per la pinza chiusa
q_9 = [q; 0; 0]; 

% Usiamo q_9 per interrogare MATLAB
T_init = getTransform(panda, q_9, ee_name);
p_init = T_init(1:3, 4);
R_des = T_init(1:3, 1:3); 

center = p_init + [r; 0; 0];


% Definiamo il centro del cerchio (cerchio orizzontale rispetto alla partenza)
center = p_init + [r; 0; 0]; 

%% 4. Generazione della Traiettoria (OFFLINE)
% Pre-allocazione per un task a 6 gradi di libertà (Pose)
Pd_dot = zeros(6, N); % Prime 3: vel angolare, Ultime 3: vel lineare
P_des  = zeros(3, N);

for k = 1:N
    % Posizione desiderata (Cerchio orizzontale sul piano XY locale) 
    P_des(1, k) = center(1) - r * cos(omega * t(k));
    P_des(2, k) = center(2) + r * sin(omega * t(k));
    P_des(3, k) = center(3); % Altezza costante
    
    % Velocità lineare desiderata
    Pd_dot(4, k) =  r * omega * sin(omega * t(k));
    Pd_dot(5, k) =  r * omega * cos(omega * t(k));
    Pd_dot(6, k) =  0;
    
    % Le velocità angolari desiderate (Pd_dot 1,2,3) restano 0 
    % perché l'orientamento è costante.
end

%% 5. Setup Grafico
fig = figure; % <-- SALVA L'HANDLE QUI! ASSEGNALO A 'fig'
show(panda, q_9, 'Frames', 'off', 'PreservePlot', false);
hold on; grid on; light;
view(3); 
plot3(P_des(1,:), P_des(2,:), P_des(3,:), 'k--', 'LineWidth', 1.5);

%% 6. Loop di Controllo CLIK
for k = 1:N
    
    % --- CONTROLLO ESISTENZA FINESTRA ---
    if ~isgraphics(fig)
        disp('Hai chiuso la finestra. Simulazione interrotta!');
        break; % Questo comando distrugge immediatamente il ciclo for
    end
    % ------------------------------------
    
    % 1. Padding per accontentare MATLAB
    q_9 = [q; 0; 0];
    
    % Cinematica Diretta
    T_curr = getTransform(panda, q_9, ee_name);
    p_curr = T_curr(1:3, 4);
    R_curr = T_curr(1:3, 1:3);
    
    % Calcolo dell'errore
    e_p = P_des(:, k) - p_curr;
    e_o = 0.5 * ( cross(R_curr(:,1), R_des(:,1)) + ...
                  cross(R_curr(:,2), R_des(:,2)) + ...
                  cross(R_curr(:,3), R_des(:,3)) );
    e = [e_o; e_p]; 
    
    % 2. Calcolo e "Taglio" dello Jacobiano
    J_full = geometricJacobian(panda, q_9, ee_name);
    J = J_full(:, 1:7); % <--- CRITICO: Estraiamo la sottomatrice 6x7 del solo braccio!
    
    % Legge di controllo per il task primario
    v_task = Pd_dot(:, k) + K * e;
    
    % Calcolo della Pseudoinversa (ora J è 6x7, quindi J_pinv sarà 7x6)
    J_pinv = pinv(J);
    
    % Null-space
    q_dot_0 = zeros(7, 1); 
    
    % Equazione completa (tutto è coerentemente a dimensione 7)
    q_dot = J_pinv * v_task + (eye(7) - J_pinv * J) * q_dot_0;
    
    % Integrazione (aggiorniamo solo i 7 giunti reali)
    q = q + q_dot * dt;
    
    if mod(k, 5) == 0
        show(panda, [q; 0; 0], 'Frames', 'off', 'PreservePlot', false);
        drawnow;
    end
end




% ------------------------------ CODICE AGGIORANTO CON APPROCCIO NEL PUNTO DEL CERCHIO DOVE LA DISTANZA TRA IL PUNTO INIZIALE DEL ROBOT E DEL CERCHIO E' MINORE ------------
clear; clc; close all;

%% 1. Inizializzazione del Robot % Carichiamo il modello del Panda 
% "Panda" è il nome comune del robot frankaEmikaPanda: un braccio robotico
% collaborativo a 7 giunti prodotto da Franka Emika, spesso usato in ricerca.
panda = loadrobot("frankaEmikaPanda", "DataFormat", "column"); %column indica i dati in vett. colonna
ee_name = 'panda_hand'; % Nome del frame dell'end-effector

%% 2. Parametri di Simulazione e Traiettoria 
dt = 0.005; % Passo di integrazione molto fine per la stabilità in 3D 
T_traj = 5.0; % La traiettoria deve essere eseguita in tot secondi

r = 0.4; % Raggio del cerchio 
omega = 2*pi / T_traj;

% Guadagni del controllore (ora abbiamo 6 gradi di libertà nel task: 3 rot, 3 trasl) 
K = 10 * eye(6);

%% 3. Configurazione Iniziale 
q = [pi/4; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4];
q_9 = [q; 0; 0];
T_init = getTransform(panda, q_9, ee_name);
p_init = T_init(1:3, 4);
R_des = T_init(1:3, 1:3);

center = [0.0; 0.0; 0.0];

% Definiamo il punto di partenza: dove si trova il cerchio a t=0
[P_start, t_start] = minDist(p_init, center, r, omega, dt, T_traj);
%% 4 e 5. Generazione delle Traiettorie (OFFLINE)

% --- FASE A: Traiettoria di Allineamento (Polinomio Cubico) ---
T_align = 1.0; % Secondi per raggiungere il punto di inizio cerchio
t_align = 0:dt:T_align;
N_align = length(t_align);

% Variabile di scaling del tempo s(t) da 0 a 1 (polinomio cubico per fluidità)
s = 3*(t_align/T_align).^2 - 2*(t_align/T_align).^3;
s_dot = 6*(t_align)/(T_align^2) - 6*(t_align.^2)/(T_align^3);

P_des_align = zeros(3, N_align);
Pd_dot_align = zeros(6, N_align);

for k = 1:N_align
    % Posizione: interpolazione lineare tra p_init e P_start
    P_des_align(:, k) = p_init + (P_start - p_init) * s(k);
    
    % Velocità lineare (righe 4-6 poiché usi [e_o; e_p])
    Pd_dot_align(4:6, k) = (P_start - p_init) * s_dot(k);
    % La velocità angolare rimane 0 (righe 1-3)
end

% --- FASE B: Traiettoria Circolare ---
t_circle = dt:dt:T_traj; % Partiamo da dt per non duplicare il punto di giunzione
N_circle = length(t_circle);
P_des_circle = zeros(3, N_circle);
Pd_dot_circle = zeros(6, N_circle);

for k = 1:N_circle 
    P_des_circle(1, k) = center(1) - r* cos(omega * (t_circle(k)+t_start)); 
    P_des_circle(2, k) = center(2) + r * sin(omega * (t_circle(k)+t_start)); 
    P_des_circle(3, k) = center(3); 
    
    Pd_dot_circle(4, k) =  r * omega * sin(omega * (t_circle(k)+t_start));
    Pd_dot_circle(5, k) =  r * omega * cos(omega * (t_circle(k)+t_start));
    Pd_dot_circle(6, k) =  0;
end

% --- FASE C: Concatenazione delle Traiettorie ---
P_des_total = [P_des_align, P_des_circle];
Pd_dot_total = [Pd_dot_align, Pd_dot_circle];

N_total = size(P_des_total, 2);
t_total = 0:dt:(N_total-1)*dt;

%% 6. Setup Grafico 
fig = figure; 
show(panda, [q;0;0], 'Frames', 'off', 'PreservePlot', false); 
hold on; grid on; light; view(3); 

% Disegno la traiettoria totale
plot3(P_des_total(1,:), P_des_total(2,:), P_des_total(3,:), 'k--', 'LineWidth', 1.5);
plot3(P_start(1), P_start(2), P_start(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Punto di giunzione

errors = zeros(3, N_total); % Pre-allocazione corretta della matrice errori
%% 7. Loop di Controllo CLIK Unificato

% Usiamo q_9 per estrarre lo stato iniziale
q_9 = [q; 0; 0];
T_curr = getTransform(panda, q_9, ee_name);
R_curr = T_curr(1:3, 1:3); % Inizializzazione R_curr

for k = 1:N_total
    
    if ~isgraphics(fig)
        disp('Hai chiuso la finestra. Simulazione interrotta!');
        break;
    end
    
    % 1. Cinematica Diretta
    q_9 = [q; 0; 0];
    T_curr = getTransform(panda, q_9, ee_name);
    p_curr = T_curr(1:3, 4);
    R_curr = T_curr(1:3, 1:3);
    
    % 2. Calcolo Errore
    e_p = P_des_total(:, k) - p_curr;
    e_o = 0.5 * ( cross(R_curr(:,1), R_des(:,1)) + cross(R_curr(:,2), R_des(:,2)) + cross(R_curr(:,3), R_des(:,3)) );
    e = [e_o; e_p];
    
    errors(:, k) = e_p; % Salvataggio
    
    % 3. Calcolo Jacobiano
    J_full = geometricJacobian(panda, q_9, ee_name);
    J = J_full(:, 1:7);
    
    % 4. Legge di Controllo CLIK (Feedforward + Feedbacks)
    v_task = Pd_dot_total(:, k) + K * e;
    
    % 5. Inversione
    % lambda = 0.1; % Fattore di smorzamento (da tarare)
    % J_dls = J' * inv(J * J' + lambda^2 * eye(6));
    
    % Equazione completa con proiezione nel nucleo
    q_dot = pinv(J) * v_task;
    
    % 6. Integrazione (Metodo di Eulero)
    q = q + q_dot * dt;
    
    % 7. Aggiornamento Grafico
    if mod(k, 10) == 0 % Aumentato a 10 per fluidità della simulazione
        show(panda, [q; 0; 0], 'Frames', 'off', 'PreservePlot', false);
        drawnow;
    end
end

%% Plot Errori Totali
figure;
plot(t_total, errors(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t_total, errors(2,:), 'g', 'LineWidth', 1.5);
plot(t_total, errors(3,:), 'b', 'LineWidth', 1.5);
grid on;
legend("Errore X", "Errore Y", "Errore Z");
title("Errori di Inseguimento Traiettoria Completa");
xlabel("Tempo [s]"); ylabel("Errore [m]");


function [P_start, t_start] = minDist(p_init, center, r, omega, dt, T_traj)
    t = 0:dt:T_traj; 
    N = length(t);
    dist_min = inf; 
    
    P_start = zeros(3,1);
    t_start = 0;
    
    for k = 1:N 
        P_circle = [center(1) - r * cos(omega * t(k)); 
                    center(2) + r * sin(omega * t(k)); 
                    center(3)];
        
        dist = norm(P_circle - p_init);
        
        if (dist < dist_min)
            dist_min = dist;
            P_start = P_circle;
            t_start = t(k); 
        end
    end
end

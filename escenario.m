function [Msgs, Veh, cfg] = escenario()
% ESCENARIO
% Escenario urbano simplificado:
% • Una avenida rectilínea unidireccional (eje X).
% • Dos carriles paralelos en el mismo sentido (y = +5 m y y = -5 m).
% • Zona escolar (más carga CAM en un intervalo temporal).
% • Evento VRU (peatón/ciclista) cerca del origen.
% • Evento de frenazo con vehículos próximos al origen.
% • Generación de CAM y DENM (solo timeline y posición, sin MAC ni canal).
%
% Salidas:
%   Msgs : tabla [t, id, type, txNode, x, y]
%   Veh  : struct con estado de vehículos (posición, velocidad)
%   cfg  : configuración del escenario

%% ====================== CONFIGURACIÓN ======================
cfg.dt       = 0.01;      % Paso temporal de simulación (s)
cfg.tEnd     = 120;       % Duración total (s)
cfg.seed     = 1;         % Semilla RNG para reproducibilidad
rng(cfg.seed);

% Vehículos y movilidad
cfg.numVeh   = 40;        % Número total de vehículos
cfg.roadHalf = 120;       % Semilargo del tramo (m) → eje X en [-120, 120]
cfg.vx_mean  = 10;        % Velocidad media (m/s) en el eje X
cfg.v_jitter = 2;         % Jitter de velocidad (m/s) para dar variabilidad

% Tráfico CAM (cooperative awareness)
cfg.rateCAM_Hz       = 10;    % tasa base CAM por vehículo (Hz)
cfg.schoolBoostFactor = 0.7;  % +70% CAM en zona escolar
cfg.lenCAM_B         = 300;   % tamaño lógico del CAM (bytes, informativo)

% Zona escolar (equivalente conceptual al "verde" antes)
cfg.tSchool  = [30 60];       % intervalo donde aumenta la tasa CAM (s)

% Eventos y DENM
cfg.tVRU     = [35 55];       % ventana con presencia de VRU (peatón/ciclista)
cfg.tBrake   = [60 90];       % ventana con frenazos / perturbaciones

cfg.rateDENM_VRU_Hz  = 5;     % tasa DENM por vehículo afectado en VRU (Hz)
cfg.rateDENM_BRK_Hz  = 5;     % tasa DENM por vehículo en frenazo (Hz)

cfg.R_VRU_m  = 100;           % radio de influencia del VRU (m)
cfg.R_BRK_m  = 120;           % radio para frenazos (m)
cfg.lenDENM_B = 700;          % tamaño lógico del DENM (bytes, informativo)

cfg.makePlots = true;         % si true, genera gráficas de tasa y distribución

%% ====================== INICIALIZACIÓN ======================
T = 0:cfg.dt:cfg.tEnd;        % vector temporal global
N = cfg.numVeh;

% Vehículos: todos en el eje X, dos carriles unidireccionales (y = ±5)
Veh = initVehicles(N, cfg);

% Tabla de mensajes (t, id, type, txNode, x, y)
Msgs = table('Size',[0 6], ...
    'VariableTypes',{'double','uint64','string','uint32','double','double'}, ...
    'VariableNames',{'t','id','type','txNode','x','y'});

pktId = uint64(0);            % contador global de identificador de paquete

%% ====================== SIMULACIÓN ======================
for k = 1:numel(T)
    t = T(k);

    % 1) Actualizar movimiento de todos los vehículos (un solo sentido + wrap)
    Veh = stepVehicles(Veh, cfg);

    % 2) Flags de eventos activos
    inSchool = (t >= cfg.tSchool(1) && t < cfg.tSchool(2));
    inVRU    = (t >= cfg.tVRU(1)    && t < cfg.tVRU(2));
    inBRK    = (t >= cfg.tBrake(1)  && t < cfg.tBrake(2));

    % 3) Generación de CAM (Poisson discretizado)
    lamCAM = cfg.rateCAM_Hz * (1 + cfg.schoolBoostFactor*inSchool);
    pCAM   = lamCAM * cfg.dt;
    for i = 1:N
        if rand < pCAM
            pktId = pktId + 1;
            pos = Veh(i).pos;
            Msgs = [Msgs; {t, pktId, "CAM", uint32(i), pos(1), pos(2)}]; %#ok<AGROW>
        end
    end

    % 4) DENM por evento VRU (vehículos cerca del origen)
    if inVRU
        allPos = vertcat(Veh.pos);
        idxNear = find(vecnorm(allPos,2,2) <= cfg.R_VRU_m);
        if ~isempty(idxNear)
            pDENM = cfg.rateDENM_VRU_Hz * cfg.dt;
            for ii = idxNear(:)'
                if rand < pDENM
                    pktId = pktId + 1;
                    pos = Veh(ii).pos;
                    Msgs = [Msgs; {t, pktId, "DENM", uint32(ii), pos(1), pos(2)}]; %#ok<AGROW>
                end
            end
        end
    end

    % 5) DENM por frenazos (vehículos más cercanos al origen)
    if inBRK
        allPos = vertcat(Veh.pos);
        dists  = vecnorm(allPos,2,2);
        nSel   = max(8, round(0.4*N));
        [~, order] = sort(dists, 'ascend');
        sel    = order(1:nSel);
        pDENM  = cfg.rateDENM_BRK_Hz * cfg.dt;
        for ii = sel(:)'
            if rand < pDENM
                pktId = pktId + 1;
                pos   = Veh(ii).pos;
                Msgs = [Msgs; {t, pktId, "DENM", uint32(ii), pos(1), pos(2)}]; %#ok<AGROW>
            end
        end
    end
end

%% ====================== PLOTS ======================
if cfg.makePlots
    plotRates(Msgs, cfg);
    plotXY(Veh, cfg);
end

end

%% ====================== AUXILIARES ======================
function Veh = initVehicles(N, cfg)
% INITVEHICLES
% Distribución más realista:
% - Todos los vehículos en el eje X (avenida unidireccional).
% - Dos carriles: y = +5 m (superior) y y = -5 m (inferior).
% - Posiciones iniciales espaciadas a lo largo de X para evitar solapamientos.

Veh = repmat(struct('pos',[0 0],'vel',[0 0]), N, 1);

nLane1 = ceil(N/2);          % nº en carril y=+5
nLane2 = N - nLane1;         % nº en carril y=-5

L = 2*cfg.roadHalf;          % longitud total del tramo (240 m)
margin = 5;                  % margen en extremos
usableL = L - 2*margin;      % longitud útil para colocar coches
% Espaciado base por carril
if nLane1 > 0
    baseX1 = linspace(-cfg.roadHalf+margin, cfg.roadHalf-margin, nLane1).';
else
    baseX1 = [];
end
if nLane2 > 0
    baseX2 = linspace(-cfg.roadHalf+margin, cfg.roadHalf-margin, nLane2).';
else
    baseX2 = [];
end

% Pequeño jitter para que no estén en una rejilla perfecta
jitAmp = 2;  % ±2 m
if ~isempty(baseX1)
    baseX1 = baseX1 + (rand(size(baseX1))-0.5)*2*jitAmp;
end
if ~isempty(baseX2)
    baseX2 = baseX2 + (rand(size(baseX2))-0.5)*2*jitAmp;
end

% Carril superior (y = +5), todos con vx > 0
for i = 1:nLane1
    vx = cfg.vx_mean + cfg.v_jitter*(rand-0.5);
    Veh(i).pos = [baseX1(i),  5];
    Veh(i).vel = [max(3,vx),  0];
end

% Carril inferior (y = -5), también en sentido +X
for j = 1:nLane2
    idx = nLane1 + j;
    vx  = cfg.vx_mean + cfg.v_jitter*(rand-0.5);
    Veh(idx).pos = [baseX2(j), -5];
    Veh(idx).vel = [max(3,vx),  0];
end
end

function Veh = stepVehicles(Veh, cfg)
% STEPVEHICLES
% Actualiza la posición de los vehículos con movimiento rectilíneo
% unidireccional en X y aplica "wrap-around" al cruzar los límites.
%
% Si x > roadHalf → x := x - 2*roadHalf (reaparece por el inicio)
% Si x < -roadHalf → x := x + 2*roadHalf (caso raro por jitter negativo)

L = 2*cfg.roadHalf;

for i = 1:numel(Veh)
    Veh(i).pos = Veh(i).pos + Veh(i).vel * cfg.dt;

    % Wrap-around en el eje X
    if Veh(i).pos(1) >  cfg.roadHalf
        Veh(i).pos(1) = Veh(i).pos(1) - L;
    elseif Veh(i).pos(1) < -cfg.roadHalf
        Veh(i).pos(1) = Veh(i).pos(1) + L;
    end
    % y se mantiene fijo (carril)
end
end

%% ====================== GRÁFICA XY ======================
function plotXY(Veh, cfg)
figure('Name','Snapshot posiciones final');

pos = vertcat(Veh.pos);
x = pos(:,1); 
y = pos(:,2);

scatter(x, y, 36, 'o', 'MarkerEdgeColor',[0.2 0.6 1], ...
    'LineWidth',1.2, 'DisplayName','vehículos');
hold on; grid on; axis equal;

% Líneas que marcan los carriles (y = ±5)
plot([-cfg.roadHalf cfg.roadHalf],[ 5  5],'k--','LineWidth',1,'HandleVisibility','off');
plot([-cfg.roadHalf cfg.roadHalf],[-5 -5],'k--','LineWidth',1,'HandleVisibility','off');

title(sprintf('Posiciones a t=%.1fs', cfg.tEnd));
xlabel('x (m)'); ylabel('y (m)');
xlim([-cfg.roadHalf cfg.roadHalf]);
ylim([-10 10]);
legend('Location','southoutside','Orientation','horizontal');
end

%% ====================== GRÁFICA DE TASAS ======================
function plotRates(Msgs, cfg)
edges = 0:1:cfg.tEnd;
isCAM = Msgs.type=="CAM";
isDEN = Msgs.type=="DENM";

CAM_counts  = histcounts(Msgs.t(isCAM), edges);
DENM_counts = histcounts(Msgs.t(isDEN), edges);

figure('Name','Tasa de mensajes');
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');

nexttile; hold on; grid on; title('Mensajes CAM por segundo');
stairs(edges(1:end-1), CAM_counts, 'LineWidth',1.2);
xline(cfg.tSchool(1),'g--','zona escolar on'); 
xline(cfg.tSchool(2),'g--','zona escolar off');
ylabel('CAM/s'); xlim([0 cfg.tEnd]);

nexttile; hold on; grid on; title('Mensajes DENM por segundo');
stairs(edges(1:end-1), DENM_counts, 'LineWidth',1.2);
xline(cfg.tVRU(1),'m--','VRU on'); 
xline(cfg.tVRU(2),'m--','VRU off');
xline(cfg.tBrake(1),'r--','brake on'); 
xline(cfg.tBrake(2),'r--','brake off');
xlabel('t (s)'); ylabel('DENM/s'); xlim([0 cfg.tEnd]);
end
function main_aim3_dynamic()


clc; close all;

%% 1) Escenario
[Msgs, ~, ~] = escenario();
Msgs = sortrows(Msgs,'t');

%% 2) Parámetros base
cfg.dt        = 0.01;
cfg.tEnd      = ceil(max(Msgs.t));
cfg.numNodes  = max(double(Msgs.txNode));
cfg.cap_pps_10MHz = 300;      % capacidad nominal (pkts/s) por 10 MHz
cfg.slot_cv2x = 0.05;         % ranura PC5 (igual que en PDR dinámico)

% Espectro total = 20 MHz, arrancamos 10/10
State.spec.MHz_11p  = 10;
State.spec.MHz_cv2x = 10;

% Canal abstracto: carga -> PER (solo para consistencia con MAC)
alpha11 = 0.4;
alphaCV = 0.3;

%% 3) Controlador Aim-3
ctrl.period_s        = 0.25;   % cada cuánto decide (4 veces/seg)
ctrl.minGap_s        = 0.75;   % anti-aleteo
ctrl.minMHz          = 2;      % límite inferior por tecnología
ctrl.maxMHz          = 18;     % límite superior
ctrl.stepQuantMHz    = 2;      % cuantización de los pasos
ctrl.maxStepMHz      = 6;      % salto máximo por decisión
ctrl.targetOcc       = 0.75;   % objetivo de ocupación total
ctrl.maxDOcc_target  = 0.30;   % objetivo para Δocc
kOcc                 = 2.0;    % peso de déficit de ocupación
kBal                 = 0.6;    % peso de balance (fairness) occ11 vs occCV

% EWMA para demanda y ocupación (suaviza las gráficas)
betaArr   = 0.2;
betaOcc   = 0.15;

%% 4) Estado, nodos, colas
State.t  = 0;
State.dt = cfg.dt;

Nodes = repmat(struct('pos',[0 0],'vel',[0 0],'tec','11p'), cfg.numNodes, 1);
for i = 1:cfg.numNodes
    if rand < 0.5
        Nodes(i).tec = '11p';
    else
        Nodes(i).tec = 'cv2x';
    end
end

Queues = repmat(struct('pkts',[]), cfg.numNodes, 1);

% Demanda suavizada (CAM + DENM) por tecnología
arr11 = 0; arrCV = 0;

% Ocupación suavizada
occ11_EWMA = 0;
occCV_EWMA = 0;

% Telemetría para plotting
tNextTel = 0;
Occ11 = []; OccCV = []; OccTot = []; DOcc = []; TT = [];
S11 = []; SCV = []; TS = [];

% Temporización del controlador
tNextCtrl    = 0;
lastChange_t = -Inf;

fprintf('[INICIO] Split -> 11p=%d MHz | CV2X=%d MHz\n', ...
    State.spec.MHz_11p, State.spec.MHz_cv2x);

%% 5) Bucle principal
while State.t < cfg.tEnd
    %% 5.1 Inyección de mensajes del escenario en [t, t+dt)
    sel = (Msgs.t >= State.t) & (Msgs.t < State.t + cfg.dt);
    step11 = 0; stepCV = 0;
    if any(sel)
        stepMsgs = Msgs(sel,:);
        % actualizar posiciones
        for r = 1:height(stepMsgs)
            n = double(stepMsgs.txNode(r));
            Nodes(n).pos = [stepMsgs.x(r), stepMsgs.y(r)];
        end
        % encolar
        for r = 1:height(stepMsgs)
            n   = double(stepMsgs.txNode(r));
            tec = Nodes(n).tec;
            pkt = struct('id',    stepMsgs.id(r), ...
                         'type',  char(stepMsgs.type(r)), ...
                         'tec',   tec, ...
                         'lenB',  300*(stepMsgs.type(r)=="CAM") + ...
                                  700*(stepMsgs.type(r)=="DENM"), ...
                         't_enq', State.t, ...
                         'txNode',n);
            Queues(n).pkts = [Queues(n).pkts; pkt]; %#ok<AGROW>
            if strcmp(tec,'11p')
                step11 = step11 + 1;
            else
                stepCV = stepCV + 1;
            end
        end
        % Demanda suavizada (pkts/s)
        arr11 = (1-betaArr)*arr11 + betaArr*(step11/cfg.dt);
        arrCV = (1-betaArr)*arrCV + betaArr*(stepCV/cfg.dt);
    end

    %% 5.2 MACs
    [tx11p, ~] = mac80211p_step(Queues, Nodes, State, cfg);
    [txcvx, ~] = macCV2X_step (Queues, Nodes, State, cfg);
    txAll = align_tx([tx11p(:); txcvx(:)]);

    % borrar de colas los transmitidos
    for k = 1:numel(txAll)
        iNode = double(txAll(k).txNode);
        if iNode>=1 && iNode<=numel(Queues) && ~isempty(Queues(iNode).pkts)
            idx = find([Queues(iNode).pkts.id] == txAll(k).id, 1, 'first');
            if ~isempty(idx), Queues(iNode).pkts(idx) = []; end
        end
    end

    %% 5.3 Carga / ocupación instantánea
    [n11, ncv] = count_by_tec(txAll);
    cap11 = cfg.cap_pps_10MHz * (State.spec.MHz_11p  / 10);
    capcv = cfg.cap_pps_10MHz * (State.spec.MHz_cv2x / 10);

    L11 = n11 / max(cap11*cfg.dt, eps);   % carga 11p (0..∞)
    Lcv = ncv / max(capcv*cfg.dt, eps);   % carga C-V2X

    % (NOTA: podríamos calcular PER aquí con alpha11/alphaCV si quisiéramos
    % enlazar con PDR, pero para Aim-3 sólo nos interesa L11/Lcv.)

    % Ocupación suavizada (limitada a 1)
    occ11_EWMA = (1-betaOcc)*occ11_EWMA + betaOcc*min(1, L11);
    occCV_EWMA = (1-betaOcc)*occCV_EWMA + betaOcc*min(1, Lcv);

    %% 5.4 Telemetría de ocupación
    if State.t >= tNextTel
        TT(end+1)    = State.t;             %#ok<AGROW>
        Occ11(end+1) = occ11_EWMA;          %#ok<AGROW>
        OccCV(end+1) = occCV_EWMA;          %#ok<AGROW>
        OccTot(end+1)= 0.5*(occ11_EWMA + occCV_EWMA); %#ok<AGROW>
        DOcc(end+1)  = abs(occ11_EWMA - occCV_EWMA);  %#ok<AGROW>
        TS(end+1)    = State.t;             %#ok<AGROW>
        S11(end+1)   = State.spec.MHz_11p;  %#ok<AGROW>
        SCV(end+1)   = State.spec.MHz_cv2x; %#ok<AGROW>
        tNextTel = tNextTel + 0.5;          % mismo refresco que en otros aims
    end

    %% 5.5 Controlador dinámico de split (Aim-3)
    if State.t >= tNextCtrl
        % Ocupaciones actuales (suavizadas)
        occ11 = occ11_EWMA;
        occCV = occCV_EWMA;
        occTot = 0.5*(occ11 + occCV);
        dOcc   = abs(occ11 - occCV);

        % Déficit de ocupación total
        defOcc = max(0, ctrl.targetOcc - occTot);

        % Pesos de demanda (CAM+DENM) ajustados por balance de ocupación:
        %   - si occ11 > occCV, se incrementa peso de C-V2X y se reduce 11p.
        %   - si occCV > occ11, al revés.
        occBias = occ11 - occCV;   % >0 => 11p más cargado
        w11 = (arr11 + 1e-3) * (1 + kOcc*defOcc) * (1 - kBal*occBias);
        wCV = (arrCV + 1e-3) * (1 + kOcc*defOcc) * (1 + kBal*occBias);

        % Forzar pesos positivos
        w11 = max(w11, 1e-3);
        wCV = max(wCV, 1e-3);

        % Objetivo de MHz proporcional a pesos
        totalMHz = 20;
        target11 = totalMHz * w11 / (w11 + wCV);
        target11 = min(max(target11, ctrl.minMHz), ctrl.maxMHz);

        % Cuantización
        q = ctrl.stepQuantMHz;
        target11 = q * round(target11 / q);
        targetCV = totalMHz - target11;

        canChange = (State.t - lastChange_t) >= ctrl.minGap_s;

        if canChange
            cur11 = State.spec.MHz_11p;
            delta = target11 - cur11;

            % Si Δocc está muy por encima de objetivo, forzamos un paso
            if dOcc > ctrl.maxDOcc_target
                step = sign(occ11 - occCV) * q;  % mover MHz hacia la tec. menos cargada
                target11 = cur11 - step;
                target11 = min(max(target11, ctrl.minMHz), ctrl.maxMHz);
                target11 = q * round(target11 / q);
                targetCV = totalMHz - target11;
                delta = target11 - cur11;
            end

            if abs(delta) >= q
                % limitar salto máximo por decisión
                step = sign(delta) * min(abs(delta), ctrl.maxStepMHz);
                new11 = cur11 + step;
                new11 = min(max(new11, ctrl.minMHz), ctrl.maxMHz);
                new11 = q * round(new11 / q);
                newCV = totalMHz - new11;

                if new11 ~= State.spec.MHz_11p
                    State.spec.MHz_11p  = new11;
                    State.spec.MHz_cv2x = newCV;
                    lastChange_t = State.t;
                    fprintf('[t=%.2fs] Split -> 11p=%2d MHz | CV2X=%2d MHz | OccTot=%.2f DOcc=%.2f\n', ...
                        State.t, new11, newCV, occTot, dOcc);
                end
            end
        end

        tNextCtrl = tNextCtrl + ctrl.period_s;
    end

    %% 5.6 Avanzar tiempo
    State.t = State.t + cfg.dt;
end

%% 6) Gráficas Aim-3
figure; tiledlayout(2,1,'Padding','compact','TileSpacing','compact');

% Ocupación
nexttile; hold on; grid on;
title('Ocupación del canal');
plot(TT, Occ11, 'DisplayName','11p');
plot(TT, OccCV, 'DisplayName','C-V2X');
plot(TT, OccTot,'w','LineWidth',1.2,'DisplayName','Total');
yline(0.60,'r--','0.60','LineWidth',1.0,'DisplayName','Valor umbral');
ylim([0 1]); xlabel('t (s)'); ylabel('Ocupación');
legend('Location','southwest');

% Δocc
nexttile; hold on; grid on;
title('\Deltaocc = |Occ_{11p} - Occ_{C-V2X}|');
plot(TT, DOcc, 'b','DisplayName','\Deltaocc');
yline(0.40,'r--','0.40','LineWidth',1.0,'DisplayName','Valor umbral');
ylim([0 1]); xlabel('t (s)'); ylabel('\Deltaocc');
legend('Location','southwest');

figure; hold on; grid on;
title('Split de espectro (MHz) — modelo dinámico');
stairs(TS, S11, 'LineWidth',1.1,'DisplayName','11p MHz');
stairs(TS, SCV, 'LineWidth',1.1,'DisplayName','C-V2X MHz');
ylim([0 20]); xlabel('t (s)'); ylabel('MHz');
legend('Location','southwest');

fprintf('\n[FIN AIM-3 DINÁMICO] Últimos valores medios: OccTot=%.2f  DOcc=%.2f\n', ...
    mean(OccTot), mean(DOcc));
end

%% ===== Auxiliares =======================================================
function tx = align_tx(tx)
tmpl = struct('id',uint64(0),'type','','tec','','txNode',uint32(0),'t_tx',0.0);
if isempty(tx), tx = tmpl([]); return; end
if iscell(tx), tx = [tx{:}]; end
if ~isstruct(tx), tx = tmpl([]); return; end
out = tmpl([]); out(end+1:numel(tx),1) = tmpl; %#ok<AGROW>
for k = 1:numel(tx)
    if isfield(tx(k),'id'),     out(k).id     = uint64(tx(k).id);    end
    if isfield(tx(k),'type'),   out(k).type   = char(tx(k).type);    end
    if isfield(tx(k),'tec'),    out(k).tec    = char(tx(k).tec);     end
    if isfield(tx(k),'txNode'), out(k).txNode = uint32(tx(k).txNode);end
    if isfield(tx(k),'t_tx'),   out(k).t_tx   = double(tx(k).t_tx);  end
end
tx = out(:);
end

function [n11, ncv] = count_by_tec(txAll)
if isempty(txAll), n11 = 0; ncv = 0; return; end
tecC = {txAll.tec};
n11 = sum(strcmp(tecC,'11p'));
ncv = sum(strcmp(tecC,'cv2x'));
end

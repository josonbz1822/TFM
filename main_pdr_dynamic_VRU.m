function main_pdr_dynamic_aim5()
% MAIN_PDR_DYNAMIC_AIM5 — KPI específico Aim-5:
%   PDR(VRU) para mensajes DENM debidos a presencia de peatones/ciclistas
%   a menos de 50 m (objetivo ≥ 90 %).
%
% Usa el mismo escenario, bloques PHY-MAC y canal abstracto del modelo
% dinámico original, añadiendo telemetría específica para VRU.

clc; close all;

%% 1) Escenario
[Msgs, ~, cfgScn] = escenario();      % cfgScn.tVRU se usa para marcar DENM de VRU
Msgs = sortrows(Msgs,'t');

%% 2) Parámetros base
cfg.dt   = 0.01;
cfg.tEnd = ceil(max(Msgs.t));
cfg.numNodes = max(double(Msgs.txNode));

cfg.telemetry.window_s = 4;
cfg.telemetry.update_s = 0.5;
cfg.telemetry.dmax_PDR = 100;     % radio clásico (no esencial para Aim-5)

% AIM-5: radio específico para KPI VRU a menos de 50 m
cfg.telemetry.dmax_VRU = 50;      

cfg.cap_pps_10MHz      = 300;

% Espectro total 20 MHz (arranque 10/10)
State.spec.MHz_11p  = 10;
State.spec.MHz_cv2x = 10;

% Canal
alpha11 = 0.4;   
alphaCV = 0.3;   

% Controlador (lo mantenemos, aunque Aim-5 solo mida PDR VRU)
ctrl.period_s       = 0.25;
ctrl.minGap_s       = 0.75;
ctrl.minMHz         = 1;      
ctrl.maxMHz         = 19;
ctrl.stepQuantMHz   = 2;      
ctrl.maxStepMHz     = 10;     
ctrl.snapThreshMHz  = 8;      
ctrl.targetPDR      = 0.95;   
ctrl.emergencyTo    = 18;     

% Pesos
gammaDENM = 4.0;
kDeficit  = 3.0;
betaArr   = 0.3;
betaOcc   = 0.3;

cfg.slot_cv2x = 0.05;

% DCC (opcional)
useDCC    = true;
dccThresh = 0.55;
dccDropP  = 0.70;

%% 3) Estado, nodos, colas
State.t  = 0; 
State.dt = cfg.dt;

Nodes = repmat(struct('pos',[0 0],'vel',[0 0],'tec','11p'), cfg.numNodes, 1);
for i=1:cfg.numNodes
    if rand < 0.5
        Nodes(i).tec = '11p';
    else
        Nodes(i).tec = 'cv2x';
    end
end
Queues = repmat(struct('pkts',[]), cfg.numNodes, 1);

% Telemetría
% AIM-5: añadimos hasRx50 e isVRU
TX = struct('id',uint64([]), 'type','','tec','', ...
            't_tx',[], 'hasRx100',false, 'hasRx50',false, 'isVRU',false);
TX = TX([]);

RX = struct('id',uint64([]), 'type','','tec','', ...
            't_tx',[], 't_rx',[], 'ok',false, ...
            'dist',[], 'isVRU',false);
RX = RX([]);

% Buffers solo para Aim-5
T         = [];
P11d_VRU  = [];   % 11p DENM (VRU @50m)
PCVd_VRU  = [];   % CV2X DENM (VRU @50m)

% (el resto de PDR generales se pueden omitir si quieres,
% pero los mantengo internos por si usas el controlador)
P11c = []; P11d = []; PCVc = []; PCVd = [];
TS  = []; S11  = []; SCV  = [];

% Para control
occEWMA_11 = 0; occEWMA_cv = 0;
arrEWMA_11_CAM = 0; arrEWMA_11_DENM = 0;
arrEWMA_cv_CAM = 0; arrEWMA_cv_DENM = 0;

tNextTel    = 0;
tNextCtrl   = 0; 
lastChange_t = -Inf;

fprintf('[INICIO] Split -> 11p=%d MHz | CV2X=%d MHz\n', ...
    State.spec.MHz_11p, State.spec.MHz_cv2x);

%% 4) Bucle principal
while State.t < cfg.tEnd
    %% 4.1 Inyección del escenario en [t, t+dt)
    sel = (Msgs.t >= State.t) & (Msgs.t < State.t + cfg.dt);
    step11_CAM = 0; step11_DENM = 0; stepcv_CAM = 0; stepcv_DENM = 0;
    if any(sel)
        stepMsgs = Msgs(sel,:);
        % actualizar posición de emisores
        for r=1:height(stepMsgs)
            n = double(stepMsgs.txNode(r));
            Nodes(n).pos = [stepMsgs.x(r), stepMsgs.y(r)];
        end
        % encolar paquetes
        for r=1:height(stepMsgs)
            n   = double(stepMsgs.txNode(r));
            tec = Nodes(n).tec;
            isDEN = (stepMsgs.type(r)=="DENM");

            % DCC: reduce CAM cuando hay alta ocupación global
            if useDCC
                occMean = (occEWMA_11 + occEWMA_cv)/2;
                if ~isDEN && occMean > dccThresh && rand < dccDropP
                    continue;
                end
            end

            pkt = struct('id',stepMsgs.id(r), ...
                         'type',char(stepMsgs.type(r)), ...
                         'tec', tec, ...
                         'lenB', 300*(stepMsgs.type(r)=="CAM") + ...
                                 700*(stepMsgs.type(r)=="DENM"), ...
                         't_enq',State.t, ...
                         'txNode',n);
            Queues(n).pkts = [Queues(n).pkts; pkt]; %#ok<AGROW>

            if strcmp(tec,'11p')
                if isDEN, step11_DENM = step11_DENM + 1; else, step11_CAM = step11_CAM + 1; end
            else
                if isDEN, stepcv_DENM = stepcv_DENM + 1; else, stepcv_CAM = stepcv_CAM + 1; end
            end
        end
        % medias suavizadas de llegadas
        arrEWMA_11_CAM  = (1-betaArr)*arrEWMA_11_CAM  + betaArr*(step11_CAM/cfg.dt);
        arrEWMA_11_DENM = (1-betaArr)*arrEWMA_11_DENM + betaArr*(step11_DENM/cfg.dt);
        arrEWMA_cv_CAM  = (1-betaArr)*arrEWMA_cv_CAM  + betaArr*(stepcv_CAM/cfg.dt);
        arrEWMA_cv_DENM = (1-betaArr)*arrEWMA_cv_DENM + betaArr*(stepcv_DENM/cfg.dt);
    end

    % Señal de evento para reserva dinámica en 11p (si tu MAC la usa)
    tot11 = max(1e-6, arrEWMA_11_CAM + arrEWMA_11_DENM);
    cfg.event11p = (arrEWMA_11_DENM / tot11) > 0.20;

    %% 4.2 MACs
    [tx11p, ~] = mac80211p_step(Queues, Nodes, State, cfg);
    [txcvx, ~] = macCV2X_step (Queues, Nodes, State, cfg);
    txAll = align_tx([tx11p(:); txcvx(:)]);

    %% 4.3 Desencolar transmitidos
    for k=1:numel(txAll)
        iNode = double(txAll(k).txNode);
        if iNode>=1 && iNode<=numel(Queues) && ~isempty(Queues(iNode).pkts)
            idx = find([Queues(iNode).pkts.id] == txAll(k).id, 1, 'first');
            if ~isempty(idx)
                Queues(iNode).pkts(idx) = [];
            end
        end
    end

    %% 4.4 Canal (carga -> PER) y HARQ para CV2X
    [n11, ncv] = count_by_tec(txAll);
    cap11 = cfg.cap_pps_10MHz * (State.spec.MHz_11p  / 10);
    capcv = cfg.cap_pps_10MHz * (State.spec.MHz_cv2x / 10);
    L11 = n11 / max(cap11*cfg.dt, eps);
    Lcv = ncv / max(capcv*cfg.dt, eps);
    PER11 = 1 - exp(-alpha11 * L11);
    PERcv = 1 - exp(-alphaCV * Lcv);

    occEWMA_11 = (1-betaOcc)*occEWMA_11 + betaOcc*min(1,L11);
    occEWMA_cv = (1-betaOcc)*occEWMA_cv + betaOcc*min(1,Lcv);

    %% 4.4b Construcción TX/RX (con foco VRU @50m)
    for k=1:numel(txAll)
        p = txAll(k);
        posTx = Nodes(double(p.txNode)).pos;

        % busco receptores potenciales a 100 m y 50 m
        has100 = false;
        has50  = false;
        for r=1:cfg.numNodes
            if r==p.txNode, continue; end
            d = norm(Nodes(r).pos - posTx);
            if d <= cfg.telemetry.dmax_PDR
                has100 = true;
            end
            if d <= cfg.telemetry.dmax_VRU
                has50 = true;
            end
            if has100 && has50
                break;
            end
        end

        % marcar si este paquete pertenece al evento VRU
        isVRU = strcmp(p.type,'DENM') && ...
                (State.t >= cfgScn.tVRU(1)) && (State.t < cfgScn.tVRU(2));

        % TX
        TX(end+1) = struct( ... %#ok<AGROW>
            'id',   uint64(p.id), ...
            'type', char(p.type), ...
            'tec',  char(p.tec), ...
            't_tx', State.t, ...
            'hasRx100', has100, ...
            'hasRx50',  has50, ...
            'isVRU',    isVRU);

        % RX (resultado canal)
        if strcmp(p.tec,'cv2x')
            % HARQ (3 intentos)
            e1 = rand < PERcv;
            e2 = rand < (0.5*PERcv + 0.5*PERcv*e1);
            e3 = rand < (0.5*PERcv + 0.5*PERcv*(e1|e2));
            ok = ~(e1 & e2 & e3);
        else
            ok = rand > PER11;
        end

        RX(end+1) = struct( ... %#ok<AGROW>
            'id',   uint64(p.id), ...
            'type', char(p.type), ...
            'tec',  char(p.tec), ...
            't_tx', State.t, ...
            't_rx', State.t, ...
            'ok',   ok, ...
            'dist', cfg.telemetry.dmax_PDR, ...
            'isVRU', isVRU);
    end

    %% 4.5 Cálculo de PDR (incluye VRU) y registro split
    if State.t >= tNextTel
        t0 = max(0, State.t - cfg.telemetry.window_s);

        T(end+1) = State.t; %#ok<AGROW>

        % (opcionales pero útiles para el controlador)
        P11c(end+1) = pdr_of(TX,RX,'11p','CAM', t0, State.t);   %#ok<AGROW>
        P11d(end+1) = pdr_of(TX,RX,'11p','DENM',t0, State.t);   %#ok<AGROW>
        PCVc(end+1) = pdr_of(TX,RX,'cv2x','CAM', t0, State.t);  %#ok<AGROW>
        PCVd(end+1) = pdr_of(TX,RX,'cv2x','DENM',t0, State.t);  %#ok<AGROW>

        % Aim-5: PDR específico VRU (DENM @50m)
        P11d_VRU(end+1) = pdr_vru_of(TX,RX,'11p',  t0, State.t); %#ok<AGROW>
        PCVd_VRU(end+1) = pdr_vru_of(TX,RX,'cv2x', t0, State.t); %#ok<AGROW>

        TS(end+1)   = State.t;  %#ok<AGROW>
        S11(end+1)  = State.spec.MHz_11p;   %#ok<AGROW>
        SCV(end+1)  = State.spec.MHz_cv2x;  %#ok<AGROW>
        tNextTel    = tNextTel + cfg.telemetry.update_s;
    end

    %% 4.6 Controlador (se sigue usando PDR general como feedback)
    if State.t >= tNextCtrl
        p11 = pickNonNaN(lastVal(P11d), lastVal(P11c)); if ~isfinite(p11), p11 = 0; end
        pcv = pickNonNaN(lastVal(PCVd), lastVal(PCVc)); if ~isfinite(pcv), pcv = 0; end
        def11 = max(0, ctrl.targetPDR - p11);
        defcv = max(0, ctrl.targetPDR - pcv);

        dem11 = arrEWMA_11_CAM + gammaDENM*arrEWMA_11_DENM;
        demcv = arrEWMA_cv_CAM + gammaDENM*arrEWMA_cv_DENM;

        w11 = (dem11+1e-6) * (0.5+0.5*occEWMA_11) * (1 + kDeficit*def11);
        wcv = (demcv+1e-6) * (0.5+0.5*occEWMA_cv) * (1 + kDeficit*defcv);

        % Emergencia: si PDR_DENM cae <0.90, fijar 18/2
        p11_den   = lastVal(P11d);  
        pcv_den   = lastVal(PCVd);
        canChange = (State.t - lastChange_t) >= ctrl.minGap_s;
        if (isfinite(p11_den) && p11_den < 0.90 && canChange)
            State.spec.MHz_11p  = ctrl.emergencyTo;
            State.spec.MHz_cv2x = 20 - ctrl.emergencyTo;
            lastChange_t = State.t;
            tNextCtrl = State.t + ctrl.period_s;
        elseif (isfinite(pcv_den) && pcv_den < 0.90 && canChange)
            State.spec.MHz_11p  = 20 - ctrl.emergencyTo;
            State.spec.MHz_cv2x = ctrl.emergencyTo;
            lastChange_t = State.t;
            tNextCtrl = State.t + ctrl.period_s;
        else
            % Reparto proporcional a pesos
            totalMHz = 20;
            target11 = totalMHz * w11 / max(w11+wcv, eps);
            target11 = min(max(target11, ctrl.minMHz), ctrl.maxMHz);
            q = ctrl.stepQuantMHz;
            target11 = q * round(target11 / q);

            if canChange
                cur11 = State.spec.MHz_11p;
                delta = target11 - cur11;
                if abs(delta) >= q
                    if abs(delta) >= ctrl.snapThreshMHz
                        step = sign(delta) * min(ctrl.maxStepMHz, abs(delta));
                    else
                        step = sign(delta) * q;
                    end
                    new11 = cur11 + step;
                    new11 = min(max(new11, ctrl.minMHz), ctrl.maxMHz);
                    new11 = q * round(new11 / q);
                    newcv = totalMHz - new11;

                    if new11 ~= State.spec.MHz_11p
                        State.spec.MHz_11p  = new11;
                        State.spec.MHz_cv2x = newcv;
                        lastChange_t = State.t;
                    end
                end
            end
            tNextCtrl = tNextCtrl + ctrl.period_s;
        end
    end

    %% 4.7 Tiempo
    State.t = State.t + cfg.dt;
end

%% 5) Gráfica Aim-5: PDR(VRU) DENM @50m
figure; hold on; grid on;
title('PDR (VRU) @50 m');
plot(T,P11d_VRU,'-','DisplayName','11p DENM (VRU)');
plot(T,PCVd_VRU,'-','DisplayName','C-V2X DENM (VRU)');
yline(0.80,'r--','0.80','DisplayName','Valor umbral');
ylim([0 1]); xlabel('t (s)'); ylabel('PDR (VRU)');
legend('Location','southwest');

fprintf('\n[FIN] Split final -> 11p=%d MHz | CV2X=%d MHz\n', ...
    State.spec.MHz_11p, State.spec.MHz_cv2x);

end

%% ===== Auxiliares =====
function tx = align_tx(tx)
tmpl = struct('id',uint64(0),'type','','tec','','txNode',uint32(0),'t_tx',0.0);
if isempty(tx), tx = tmpl([]); return; end
if iscell(tx), tx = [tx{:}]; end
if ~isstruct(tx), tx = tmpl([]); return; end
out = tmpl([]); out(end+1:numel(tx),1) = tmpl; %#ok<AGROW>
for k=1:numel(tx)
    if isfield(tx(k),'id'),     out(k).id     = uint64(tx(k).id);    end
    if isfield(tx(k),'type'),   out(k).type   = char(tx(k).type);    end
    if isfield(tx(k),'tec'),    out(k).tec    = char(tx(k).tec);     end
    if isfield(tx(k),'txNode'), out(k).txNode = uint32(tx(k).txNode);end
    if isfield(tx(k),'t_tx'),   out(k).t_tx   = double(tx(k).t_tx);  end
end
tx = out(:);
end

function [n11, ncv] = count_by_tec(txAll)
if isempty(txAll), n11=0; ncv=0; return; end
tecC = {txAll.tec};
n11 = sum(strcmp(tecC,'11p'));
ncv = sum(strcmp(tecC,'cv2x'));
end

function p = pdr_of(TX,RX,tec,typ,t0,t1)
if isempty(TX), p=NaN; return; end
selTX = strcmp({TX.tec},tec) & strcmp({TX.type},typ) & ...
        [TX.t_tx]>=t0 & [TX.t_tx]<t1 & [TX.hasRx100];
if ~any(selTX), p=NaN; return; end
denIds = unique([TX(selTX).id]);
selRX = strcmp({RX.tec},tec) & strcmp({RX.type},typ) & ...
        [RX.t_tx]>=t0 & [RX.t_tx]<t1 & [RX.ok];
okIds  = unique([RX(selRX).id]);
p = numel(intersect(denIds, okIds)) / numel(denIds);
p = min(max(double(p),0),1);
end

% Aim-5: PDR específico para VRU (DENM @50m)
function p = pdr_vru_of(TX,RX,tec,t0,t1)
if isempty(TX), p=NaN; return; end

selTX = strcmp({TX.tec},tec) & ...
        strcmp({TX.type},'DENM') & ...
        [TX.t_tx] >= t0 & [TX.t_tx] < t1 & ...
        [TX.hasRx50] & [TX.isVRU];

if ~any(selTX), p=NaN; return; end

denIds = unique([TX(selTX).id]);

selRX = strcmp({RX.tec},tec) & ...
        strcmp({RX.type},'DENM') & ...
        [RX.t_tx] >= t0 & [RX.t_tx] < t1 & ...
        [RX.ok] & [RX.isVRU];

okIds = unique([RX(selRX).id]);

p = numel(intersect(denIds, okIds)) / numel(denIds);
p = min(max(double(p),0),1);
end

function v = lastVal(x)
if isempty(x), v = NaN; else, v = x(end); end
end

function x = pickNonNaN(a,b)
if ~isfinite(a), x = b; else, x = a; end
end
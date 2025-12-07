function main_latency_dynamic()


clc; close all;

%% 1) Escenario
[Msgs, ~, ~] = escenario();
Msgs = sortrows(Msgs,'t');

%% 2) Parámetros base
cfg.dt   = 0.01;
cfg.tEnd = ceil(max(Msgs.t));
cfg.numNodes = max(double(Msgs.txNode));

cfg.telemetry.window_s = 2;
cfg.telemetry.update_s = 0.25;
cfg.telemetry.dmax = 100;

cfg.cap_pps_10MHz = 300;

% Espectro total 20 MHz (arranque 10/10)
State.spec.MHz_11p  = 10;
State.spec.MHz_cv2x = 10;

% Modelo de canal
alpha11 = 0.4;
alphaCV = 0.3;

% Controlador
ctrl.period_s      = 0.25;
ctrl.minGap_s      = 0.75;
ctrl.minMHz        = 1;
ctrl.maxMHz        = 19;
ctrl.stepQuantMHz  = 2;
ctrl.maxStepMHz    = 10;
ctrl.snapThreshMHz = 8;

% Objetivos Aim-2
ctrl.latTarget  = 0.100;   % 100 ms
ctrl.jittTarget = 0.040;   % 40 ms

ctrl.kLat  = 6.0;
ctrl.kJitt = 4.0;

ctrl.emergencyTo = 18;
ctrl.emergLatTol = 0.140;  % 140 ms
ctrl.emergJitTol = 0.060;  % 60 ms

gammaDENM = 4.0;
betaArr   = 0.3;
betaOcc   = 0.3;

cfg.slot_cv2x = 0.05;

useDCC    = true;
dccThresh = 0.55;
dccDropP  = 0.70;

%% 3) Estado, nodos, colas
State.t  = 0;
State.dt = cfg.dt;

Nodes = repmat(struct('pos',[0 0],'vel',[0 0],'tec','11p'), cfg.numNodes, 1);
for i=1:cfg.numNodes
    if rand < 0.5, Nodes(i).tec = '11p'; else, Nodes(i).tec = 'cv2x'; end
end
Queues = repmat(struct('pkts',[]), cfg.numNodes, 1);

% Log de encolado (para latencia extremo a extremo)
EnqLog = struct('id',uint64([]),'t_enq',[]); EnqLog = EnqLog([]);

% Telemetría TX/RX para latencia
TX = struct('id',uint64([]),'type','','tec','', ...
            't_dep',[],'t_enq',[],'okEligible',false); TX = TX([]);
RX = struct('id',uint64([]),'type','','tec','', ...
            't_dep',[],'t_rx',[],'ok',false);          RX = RX([]);

% Buffers lat/jitter y split
tNextTel = 0;
T  = [];
L95_11 = []; L95_CV = [];
J95_11 = []; J95_CV = [];
TS = []; S11 = []; SCV = [];

% Para control
occEWMA_11 = 0; occEWMA_cv = 0;
arrEWMA_11_CAM = 0; arrEWMA_11_DENM = 0;
arrEWMA_cv_CAM = 0; arrEWMA_cv_DENM = 0;

tNextCtrl    = 0;
lastChange_t = -Inf;

fprintf('[INICIO] Split -> 11p=%d MHz | C-V2X=%d MHz\n', ...
    State.spec.MHz_11p, State.spec.MHz_cv2x);

%% 4) Bucle principal
while State.t < cfg.tEnd
    %% 4.1 Inyección del escenario
    sel = (Msgs.t >= State.t) & (Msgs.t < State.t + cfg.dt);
    step11_CAM = 0; step11_DENM = 0; stepcv_CAM = 0; stepcv_DENM = 0;
    if any(sel)
        stepMsgs = Msgs(sel,:);
        % actualizar posiciones
        for r=1:height(stepMsgs)
            n = double(stepMsgs.txNode(r));
            Nodes(n).pos = [stepMsgs.x(r), stepMsgs.y(r)];
        end
        % encolar
        for r=1:height(stepMsgs)
            n   = double(stepMsgs.txNode(r));
            tec = Nodes(n).tec;
            isDEN = (stepMsgs.type(r)=="DENM");

            % DCC sobre CAM
            if useDCC
                occMean = (occEWMA_11 + occEWMA_cv)/2;
                if ~isDEN && occMean > dccThresh && rand < dccDropP
                    continue;
                end
            end

            pkt = struct( ...
                'id',   stepMsgs.id(r), ...
                'type', char(stepMsgs.type(r)), ...
                'tec',  tec, ...
                'lenB', 300*(stepMsgs.type(r)=="CAM") + ...
                        700*(stepMsgs.type(r)=="DENM"), ...
                't_enq', State.t, ...
                'txNode', n);

            Queues(n).pkts = [Queues(n).pkts; pkt]; %#ok<AGROW>

            % log de encolado
            EnqLog(end+1) = struct('id',uint64(stepMsgs.id(r)), ...
                                   't_enq',State.t); %#ok<AGROW>

            % demanda
            if strcmp(tec,'11p')
                if isDEN, step11_DENM = step11_DENM + 1;
                else,      step11_CAM  = step11_CAM  + 1;
                end
            else
                if isDEN, stepcv_DENM = stepcv_DENM + 1;
                else,      stepcv_CAM  = stepcv_CAM  + 1;
                end
            end
        end

        arrEWMA_11_CAM  = (1-betaArr)*arrEWMA_11_CAM  + betaArr*(step11_CAM/cfg.dt);
        arrEWMA_11_DENM = (1-betaArr)*arrEWMA_11_DENM + betaArr*(step11_DENM/cfg.dt);
        arrEWMA_cv_CAM  = (1-betaArr)*arrEWMA_cv_CAM  + betaArr*(stepcv_CAM/cfg.dt);
        arrEWMA_cv_DENM = (1-betaArr)*arrEWMA_cv_DENM + betaArr*(stepcv_DENM/cfg.dt);
    end

    % bandera de evento para 11p (más reserva a DENM)
    tot11 = max(1e-6, arrEWMA_11_CAM + arrEWMA_11_DENM);
    cfg.event11p = (arrEWMA_11_DENM / tot11) > 0.20;

    %% 4.2 MACs
    [tx11p, ~] = mac80211p_step(Queues, Nodes, State, cfg);
    [txcvx, ~] = macCV2X_step (Queues, Nodes, State, cfg);
    txAll = align_tx([tx11p(:); txcvx(:)]);

    %% 4.3 Desencolar transmitidos
    for k = 1:numel(txAll)
        iNode = double(txAll(k).txNode);
        if iNode>=1 && iNode<=numel(Queues) && ~isempty(Queues(iNode).pkts)
            idx = find([Queues(iNode).pkts.id] == txAll(k).id, 1, 'first');
            if ~isempty(idx)
                Queues(iNode).pkts(idx) = [];
            end
        end
    end

    %% 4.4 Canal y tiempos RX
    [n11, ncv] = count_by_tec(txAll);
    cap11 = cfg.cap_pps_10MHz * (State.spec.MHz_11p  / 10);
    capcv = cfg.cap_pps_10MHz * (State.spec.MHz_cv2x / 10);
    L11 = n11 / max(cap11*cfg.dt, eps);
    Lcv = ncv / max(capcv*cfg.dt, eps);
    PER11 = 1 - exp(-alpha11 * L11);
    PERcv = 1 - exp(-alphaCV * Lcv);

    occEWMA_11 = (1-betaOcc)*occEWMA_11 + betaOcc*min(1,L11);
    occEWMA_cv = (1-betaOcc)*occEWMA_cv + betaOcc*min(1,Lcv);

    tau_11 = 0.010;   % 10 ms
    tau_cv = 0.020;   % 20 ms

    for k = 1:numel(txAll)
        p   = txAll(k);
        idk = uint64(p.id);

        % tiempo de encolado
        idxE = find([EnqLog.id]==idk, 1, 'first');
        if isempty(idxE)
            t_enq = State.t;
        else
            t_enq = EnqLog(idxE).t_enq;
        end

        t_dep = State.t;

        if strcmp(p.tec,'cv2x')
            ok  = rand > PERcv;
            t_rx = t_dep + tau_cv;
        else
            ok  = rand > PER11;
            t_rx = t_dep + tau_11;
        end

        TX(end+1) = struct( ...
            'id', idk, ...
            'type', char(p.type), ...
            'tec',  char(p.tec), ...
            't_dep', t_dep, ...
            't_enq', t_enq, ...
            'okEligible', true); %#ok<AGROW>

        RX(end+1) = struct( ...
            'id',   idk, ...
            'type', char(p.type), ...
            'tec',  char(p.tec), ...
            't_dep', t_dep, ...
            't_rx',  t_rx, ...
            'ok',   ok); %#ok<AGROW>
    end

    %% 4.5 Latencia/Jitter por ventana
    if State.t >= tNextTel
        t0 = max(0, State.t - cfg.telemetry.window_s);

        [L95_11_cur, J95_11_cur] = lat_jit95_of(TX,RX,'11p','DENM', t0, State.t);
        [L95_CV_cur, J95_CV_cur] = lat_jit95_of(TX,RX,'cv2x','DENM', t0, State.t);

        T(end+1)      = State.t;        %#ok<AGROW>
        L95_11(end+1) = L95_11_cur;     %#ok<AGROW>
        L95_CV(end+1) = L95_CV_cur;     %#ok<AGROW>
        J95_11(end+1) = J95_11_cur;     %#ok<AGROW>
        J95_CV(end+1) = J95_CV_cur;     %#ok<AGROW>

        TS(end+1)   = State.t;          %#ok<AGROW>
        S11(end+1)  = State.spec.MHz_11p;   %#ok<AGROW>
        SCV(end+1)  = State.spec.MHz_cv2x;  %#ok<AGROW>

        tNextTel = tNextTel + cfg.telemetry.update_s;
    end

    %% 4.6 Controlador de split
    if State.t >= tNextCtrl
        L11v = lastVal(L95_11); J11v = lastVal(J95_11);
        LCVv = lastVal(L95_CV); JCVv = lastVal(J95_CV);

        defL11 = max(0, L11v - ctrl.latTarget);
        defLCV = max(0, LCVv - ctrl.latTarget);
        defJ11 = max(0, J11v - ctrl.jittTarget);
        defJCV = max(0, JCVv - ctrl.jittTarget);

        dem11 = arrEWMA_11_CAM + gammaDENM*arrEWMA_11_DENM;
        demcv = arrEWMA_cv_CAM + gammaDENM*arrEWMA_cv_DENM;

        w11 = (dem11+1e-6) * (0.5+0.5*occEWMA_11) * ...
              (1 + ctrl.kLat*(defL11/ctrl.latTarget) + ...
                   ctrl.kJitt*(defJ11/ctrl.jittTarget));
        wcv = (demcv+1e-6) * (0.5+0.5*occEWMA_cv) * ...
              (1 + ctrl.kLat*(defLCV/ctrl.latTarget) + ...
                   ctrl.kJitt*(defJCV/ctrl.jittTarget));

        canChange = (State.t - lastChange_t) >= ctrl.minGap_s;
        if canChange
            % Emergencias
            if (L11v > ctrl.emergLatTol || J11v > ctrl.emergJitTol) && L11v>0
                State.spec.MHz_11p  = ctrl.emergencyTo;
                State.spec.MHz_cv2x = 20 - ctrl.emergencyTo;
                lastChange_t = State.t;
                fprintf('[t=%.2fs] EMERG 11p -> %d/%d (L95_11=%.3f, J95_11=%.3f)\n',...
                    State.t, State.spec.MHz_11p, State.spec.MHz_cv2x, L11v, J11v);
            elseif (LCVv > ctrl.emergLatTol || JCVv > ctrl.emergJitTol) && LCVv>0
                State.spec.MHz_11p  = 20 - ctrl.emergencyTo;
                State.spec.MHz_cv2x = ctrl.emergencyTo;
                lastChange_t = State.t;
                fprintf('[t=%.2fs] EMERG CV2X -> %d/%d (L95_CV=%.3f, J95_CV=%.3f)\n',...
                    State.t, State.spec.MHz_11p, State.spec.MHz_cv2x, LCVv, JCVv);
            else
                % Reparto normal
                totalMHz = 20;
                target11 = totalMHz * w11 / max(w11+wcv, eps);
                target11 = min(max(target11, ctrl.minMHz), ctrl.maxMHz);
                q        = ctrl.stepQuantMHz;
                target11 = q * round(target11 / q);
                targetcv = totalMHz - target11;

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
                        fprintf('[t=%.2fs] Split -> 11p=%d | C-V2X=%d (L95_11=%.3f, L95_CV=%.3f)\n', ...
                            State.t, new11, newcv, L11v, LCVv);
                    end
                end
            end
        end
        tNextCtrl = tNextCtrl + ctrl.period_s;
    end

    %% 4.7 Tiempo
    State.t = State.t + cfg.dt;
end

%% 5) Gráficas
figure; tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

% Latencia P95
nexttile; hold on; grid on;
title('Latencia P95 (DENM)');
plot(T,1e3*L95_11,'-','DisplayName','11p');
plot(T,1e3*L95_CV,'-','DisplayName','C-V2X');
yline(100,'r--','100 ms','DisplayName','Valor umbral');
ylabel('ms'); xlabel('t (s)');
legend('Location','southwest');

% Jitter P95
nexttile; hold on; grid on;
title('Jitter P95 (DENM)');
plot(T,1e3*J95_11,'-','DisplayName','11p');
plot(T,1e3*J95_CV,'-','DisplayName','C-V2X');
yline(40,'r--','40 ms','DisplayName','Valor umbral');
ylabel('ms'); xlabel('t (s)');
legend('Location','southwest');

% Split
%nexttile; hold on; grid on;
%title('Split de espectro (MHz)');
%stairs(TS,S11,'LineWidth',1.2,'DisplayName','11p MHz');
%stairs(TS,SCV,'LineWidth',1.2,'DisplayName','C-V2X MHz');
%ylim([0 20]); xlabel('t (s)'); ylabel('MHz');
%legend('Location','southwest');

% ---- Resumen numérico con helper seguro ----
fprintf('\n[Última ventana] Lat95: 11p=%s, C-V2X=%s | Jit95: 11p=%s, C-V2X=%s\n', ...
    fmt_ms(L95_11), fmt_ms(L95_CV), fmt_ms(J95_11), fmt_ms(J95_CV));

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

function [L95, J95] = lat_jit95_of(TX,RX,tec,typ,t0,t1)
L95 = NaN; J95 = NaN;
if isempty(TX) || isempty(RX), return; end

maskTX = strcmp({TX.tec},tec) & strcmp({TX.type},typ) & ...
         [TX.t_dep] >= t0 & [TX.t_dep] < t1 & [TX.okEligible];
if ~any(maskTX), return; end
TXw = TX(maskTX);

latencies = [];
for k = 1:numel(TXw)
    idk = TXw(k).id;
    maskRX = [RX.id]==idk & strcmp({RX.tec},tec) & ...
             strcmp({RX.type},typ) & [RX.ok];
    idx = find(maskRX,1,'first');
    if isempty(idx), continue; end
    t_enq = TXw(k).t_enq;
    t_rx  = RX(idx).t_rx;
    lat   = t_rx - t_enq;
    if lat >= 0
        latencies(end+1) = lat; %#ok<AGROW>
    end
end

if isempty(latencies), return; end
L95 = pct(latencies,95);
med = median(latencies);
J95 = pct(abs(latencies - med),95);
end

function v = lastVal(x)
if isempty(x), v = NaN; else, v = x(end); end
end

function p = pct(x,q)
x = sort(x(:));
if isempty(x), p = NaN; return; end
n = numel(x);
r = (q/100)*(n-1) + 1;
k = floor(r); d = r-k;
if k>=n
    p = x(n);
else
    p = x(k) + d*(x(k+1)-x(k));
end
end

function s = fmt_ms(vec)
% Devuelve texto "—" o "XX.X ms" con el último valor del vector
if isempty(vec) || isnan(vec(end))
    s = '—';
else
    s = sprintf('%.1f ms', 1e3*vec(end));
end
end

function main_latency_static()
% MAIN_LATENCY_STATIC
% Igual que main_pdr_static (estático 10/10 MHz), pero la telemetría
% calcula Latencia P95 y Jitter P95 para DENM (Aim-2).

clc; close all;

%% 1) Escenario (igual que PDR)
[Msgs, ~, ~] = escenario();
need = {'t','id','type','txNode','x','y'};
miss = setdiff(need, Msgs.Properties.VariableNames);
if ~isempty(miss), error('Faltan columnas en Msgs: %s', strjoin(miss,', ')); end
Msgs = sortrows(Msgs,'t');

%% 2) Parámetros (idénticos a PDR salvo tau_proc)
cfg.dt   = 0.01;
cfg.tEnd = ceil(max(Msgs.t));
cfg.numNodes = max(double(Msgs.txNode));
cfg.telemetry.window_s = 2;      % misma ventana que PDR
cfg.telemetry.update_s = 0.25;
cfg.telemetry.dmax = 100;    % vecinos a 100 m
cfg.cap_pps_10MHz      = 220;

% Reparto estático 10/10
State.spec.MHz_11p  = 10;
State.spec.MHz_cv2x = 10;

% Modelo canal (mismo que PDR)
alpha11 = 0.9;    % sensibilidad 11p carga->PER
alphaCV = 0.6;    % sensibilidad C-V2X

% Retardo fijo de proceso/stack
tau_proc = 0.010; % 10 ms

%% 3) Estado, nodos, colas (idéntico a PDR)
State.t  = 0; 
State.dt = cfg.dt;

Nodes = repmat(struct('pos',[0 0],'vel',[0 0],'tec','11p'), cfg.numNodes, 1);
for i=1:cfg.numNodes
    if rand < 0.5, Nodes(i).tec = '11p'; else, Nodes(i).tec = 'cv2x'; end
end
Queues = repmat(struct('pkts',[]), cfg.numNodes, 1);

% Buffers telemetría
TX = struct('id',uint64([]),'type','','tec','', 't_tx',[],'t_enq',[],'hasRx100',false); TX = TX([]);
RX = struct('id',uint64([]),'type','','tec','', 't_tx',[],'t_enq',[],'t_rx',[],'ok',false,'dist',[]); RX = RX([]);

% Series para pintar
tNext = 0; T=[]; L95_11=[]; L95_CV=[]; J95_11=[]; J95_CV=[];

%% 4) Bucle principal
while State.t < cfg.tEnd
    % 4.1 Inyectar timeline
    sel = (Msgs.t >= State.t) & (Msgs.t < State.t + cfg.dt);
    if any(sel)
        stepMsgs = Msgs(sel,:);
        for r=1:height(stepMsgs)
            n = double(stepMsgs.txNode(r));
            Nodes(n).pos = [stepMsgs.x(r), stepMsgs.y(r)];
        end
        for r=1:height(stepMsgs)
            n   = double(stepMsgs.txNode(r));
            tec = Nodes(n).tec;
            Queues(n).pkts = [Queues(n).pkts; struct( ...
                'id',   stepMsgs.id(r), ...
                'type', char(stepMsgs.type(r)), ...
                'tec',  tec, ...
                'lenB', 300*(stepMsgs.type(r)=="CAM") + 700*(stepMsgs.type(r)=="DENM"), ...
                't_enq',State.t, ...
                'txNode',n )]; %#ok<AGROW>
        end
    end

    % 4.2 MACs
    [tx11p, ~] = mac80211p_step(Queues, Nodes, State, cfg);
    [txcvx, ~] = macCV2X_step (Queues, Nodes, State, cfg);
    txAll = align_tx([tx11p(:); txcvx(:)]);

    % 4.3 Carga y PER
    [n11, ncv] = count_by_tec(txAll);
    cap11 = cfg.cap_pps_10MHz * (State.spec.MHz_11p  / 10);
    capcv = cfg.cap_pps_10MHz * (State.spec.MHz_cv2x / 10);
    L11 = n11 / max(cap11*cfg.dt, eps);
    Lcv = ncv / max(capcv*cfg.dt, eps);
    PER11 = 1 - exp(-alpha11 * L11);
    PERcv = 1 - exp(-alphaCV * Lcv);

    % 4.4 Construir TX/RX (añade latencia y t_enq)
    for k=1:numel(txAll)
        p = txAll(k);
        iNode = double(p.txNode);

        % recuperar t_enq
        t_enq = State.t;
        if iNode>=1 && iNode<=numel(Queues) && ~isempty(Queues(iNode).pkts)
            idx = find([Queues(iNode).pkts.id] == p.id, 1, 'first');
            if ~isempty(idx), t_enq = Queues(iNode).pkts(idx).t_enq; end
        end

        % comprobar vecinos a 100m
        has = false;
        posTx = Nodes(iNode).pos;
        for r=1:cfg.numNodes
            if r==iNode, continue; end
            if norm(Nodes(r).pos - posTx) <= cfg.telemetry.dmax
                has = true; break;
            end
        end

        TX(end+1) = struct('id',uint64(p.id),'type',char(p.type),'tec',char(p.tec), ...
                           't_tx',State.t,'t_enq',t_enq,'hasRx100',has); %#ok<AGROW>

        perThis = (strcmp(p.tec,'11p'))*PER11 + (strcmp(p.tec,'cv2x'))*PERcv;
        ok = rand > perThis;
        t_rx = State.t + tau_proc;  % retardo fijo de proceso

        RX(end+1) = struct('id',uint64(p.id),'type',char(p.type),'tec',char(p.tec), ...
                           't_tx',State.t,'t_enq',t_enq,'t_rx',t_rx,'ok',ok,'dist',cfg.telemetry.dmax); %#ok<AGROW>
    end

    % 4.5 Desencolar transmitidos
    for k=1:numel(txAll)
        iNode = double(txAll(k).txNode);
        if iNode>=1 && iNode<=numel(Queues) && ~isempty(Queues(iNode).pkts)
            idx = find([Queues(iNode).pkts.id] == txAll(k).id, 1, 'first');
            if ~isempty(idx), Queues(iNode).pkts(idx) = []; end
        end
    end

    % 4.6 Métricas por ventana
    if State.t >= tNext
        t0 = max(0, State.t - cfg.telemetry.window_s);
        T(end+1)    = State.t; %#ok<AGROW>
        [l95, j95]  = lat_jit_p95(TX,RX,'11p','DENM',t0,State.t);
        L95_11(end+1) = l95; %#ok<AGROW>
        J95_11(end+1) = j95; %#ok<AGROW>
        [l95, j95]  = lat_jit_p95(TX,RX,'cv2x','DENM',t0,State.t);
        L95_CV(end+1) = l95; %#ok<AGROW>
        J95_CV(end+1) = j95; %#ok<AGROW>
        tNext = tNext + cfg.telemetry.update_s;
    end

    % 4.7 Avanza tiempo
    State.t = State.t + cfg.dt;
end

%% 5) Gráficas
figure; tiledlayout(1,1,'Padding','compact','TileSpacing','compact');

% Latencia P95
nexttile; hold on; grid on; title('Latencia P95 (DENM) (modelo estático (10/10))');
plot(T, 1e3*L95_11,'-','DisplayName','11p');
plot(T, 1e3*L95_CV,'-','DisplayName','C-V2X');
ylabel('ms'); xlabel('t (s)'); legend('Location','southwest');

% Resumen consola
fprintf('[Última ventana] L95: 11p=%s | CV2X=%s  ||  J95: 11p=%s | CV2X=%s\n', ...
    fmt_ms(L95_11), fmt_ms(L95_CV), fmt_ms(J95_11), fmt_ms(J95_CV));
end

%% ===== Auxiliares =====
function s = fmt_ms(v)
if isempty(v) || isnan(v(end))
    s = '—';
else
    s = sprintf('%.1f ms', 1e3*v(end));
end
end

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

function [L95, J95] = lat_jit_p95(TX,RX,tec,typ,t0,t1)
if isempty(TX), L95=NaN; J95=NaN; return; end
selTX = strcmp({TX.tec},tec) & strcmp({TX.type},typ) & [TX.t_tx]>=t0 & [TX.t_tx]<t1 & [TX.hasRx100];
if ~any(selTX), L95=NaN; J95=NaN; return; end
idsDen = unique([TX(selTX).id]);
selOK = strcmp({RX.tec},tec) & strcmp({RX.type},typ) & [RX.t_tx]>=t0 & [RX.t_tx]<t1 & [RX.ok];
idsOK = unique([RX(selOK).id]);
ids   = intersect(idsDen, idsOK);
if isempty(ids), L95=NaN; J95=NaN; return; end

lat = zeros(numel(ids),1);
for k=1:numel(ids)
    r = find([RX.id]==ids(k), 1, 'last');
    lat(k) = RX(r).t_rx - RX(r).t_enq;
end
L95 = prctile(lat,95);
med = median(lat);
J95 = prctile(abs(lat - med), 95);
end
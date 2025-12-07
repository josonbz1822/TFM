function main_pdr_static()
% MAIN_PDR_STATIC
% Conecta escenario.m con mac80211p_step.m y macCV2X_step.m
% y grafica el PDR @100 m (CAM y DENM) en modo estático 10/10 MHz.

clc; close all;

%% 1) Cargar escenario (timeline de mensajes)
[Msgs, Veh, cfgScn] = escenario(); %#ok<ASGLU>
if ~istable(Msgs), error('escenario() debe devolver una tabla Msgs.'); end
need = {'t','id','type','txNode','x','y'};
miss = setdiff(need, Msgs.Properties.VariableNames);
if ~isempty(miss)
    error('Faltan columnas en Msgs: %s', strjoin(miss,', '));
end
Msgs = sortrows(Msgs,'t');

%% 2) Parámetros mínimos (estático 10/10)
cfg.dt   = 0.01;
cfg.tEnd = ceil(max(Msgs.t));
cfg.numNodes = max(double(Msgs.txNode));
cfg.telemetry.window_s = 6;     % ventana PDR
cfg.telemetry.update_s = 0.5;   % refresco
cfg.telemetry.dmax_PDR = 100;   % KPI a 100 m

% reparto estático del espectro (lo usan los MACs)
State.spec.MHz_11p  = 10;
State.spec.MHz_cv2x = 10;
cfg.cap_pps_10MHz   = 220;      % capacidad de referencia canal (para PER)

% PER simple en función de carga (puedes ajustar)
alpha11 = 0.9; alphaCV = 0.6;

%% 3) Estado, nodos y colas
State.t  = 0; 
State.dt = cfg.dt;

% cada nodo usa tecnología fija 50/50
Nodes = repmat(struct('pos',[0 0],'vel',[0 0],'tec','11p'), cfg.numNodes, 1);
for i=1:cfg.numNodes
    if rand < 0.5, Nodes(i).tec = '11p'; else, Nodes(i).tec = 'cv2x'; end
end

% prepara colas de paquetes
Queues = repmat(struct('pkts',[]), cfg.numNodes, 1);

% ===== FIX: buffers telemetría correctamente inicializados como struct vacío =====
TX = struct('id',uint64([]), 'type','','tec','', 't_tx',[], 'hasRx100',false); TX = TX([]);  % 0x0 struct
RX = struct('id',uint64([]), 'type','','tec','', 't_tx',[], 't_rx',[], 'ok',false, 'dist',[]); RX = RX([]); % 0x0 struct
% =================================================================================

% vectores para plot
tNext = 0; T=[]; P11c=[]; P11d=[]; PCVc=[]; PCVd=[];

%% 4) Bucle principal
while State.t < cfg.tEnd
    % 4.1 Inyectar mensajes del escenario en [t, t+dt)
    sel = (Msgs.t >= State.t) & (Msgs.t < State.t + cfg.dt);
    if any(sel)
        stepMsgs = Msgs(sel,:);
        % actualiza posición del emisor
        for r=1:height(stepMsgs)
            n = double(stepMsgs.txNode(r));
            Nodes(n).pos = [stepMsgs.x(r), stepMsgs.y(r)];
        end
        % encola mensaje en la tecnología fija del nodo
        for r=1:height(stepMsgs)
            n   = double(stepMsgs.txNode(r));
            tec = Nodes(n).tec; % '11p' o 'cv2x'
            pkt = struct('id',stepMsgs.id(r), ...
                         'type',char(stepMsgs.type(r)), ...
                         'tec', tec, ...
                         'lenB', 300*(stepMsgs.type(r)=="CAM") + 700*(stepMsgs.type(r)=="DENM"), ...
                         't_enq',State.t, ...
                         'txNode',n);
            Queues(n).pkts = [Queues(n).pkts; pkt]; %#ok<AGROW>
        end
    end

    % 4.2 MACs: quién transmite en este paso
    [tx11p, ~] = mac80211p_step(Queues, Nodes, State, cfg);
    [txcvx, ~] = macCV2X_step (Queues, Nodes, State, cfg);
    txAll = align_tx([tx11p(:); txcvx(:)]);  % normaliza a struct con campos id,type,tec,txNode,t_tx

    % 4.3 Desencolar (borra el paquete transmitido por id)
    for k=1:numel(txAll)
        iNode = double(txAll(k).txNode);
        if iNode>=1 && iNode<=numel(Queues) && ~isempty(Queues(iNode).pkts)
            idx = find([Queues(iNode).pkts.id] == txAll(k).id, 1, 'first');
            if ~isempty(idx), Queues(iNode).pkts(idx) = []; end
        end
    end

    % 4.4 Canal abstracto: carga -> PER -> RX (éxito/fracaso)
    [n11, ncv] = count_by_tec(txAll);
    cap11 = cfg.cap_pps_10MHz * (State.spec.MHz_11p  / 10);
    capcv = cfg.cap_pps_10MHz * (State.spec.MHz_cv2x / 10);
    L11 = n11 / max(cap11*cfg.dt, eps);
    Lcv = ncv / max(capcv*cfg.dt, eps);
    PER11 = 1 - exp(-alpha11 * L11);
    PERcv = 1 - exp(-alphaCV * Lcv);

    % Construir TX (denominador) y RX (numerador)
    for k=1:numel(txAll)
        p = txAll(k);
        posTx = Nodes(double(p.txNode)).pos;
        has = false;
        for r=1:cfg.numNodes
            if r==p.txNode, continue; end
            if norm(Nodes(r).pos - posTx) <= cfg.telemetry.dmax_PDR
                has = true; break;
            end
        end
        TX(end+1) = struct('id',uint64(p.id),'type',char(p.type),'tec',char(p.tec), ...
                           't_tx',State.t,'hasRx100',has); %#ok<AGROW>
        perThis = (strcmp(p.tec,'11p'))*PER11 + (strcmp(p.tec,'cv2x'))*PERcv;
        ok = rand > perThis;
        RX(end+1) = struct('id',uint64(p.id),'type',char(p.type),'tec',char(p.tec), ...
                           't_tx',State.t,'t_rx',State.t,'ok',ok,'dist',cfg.telemetry.dmax_PDR); %#ok<AGROW>
    end

    % 4.5 PDR por ventana
    if State.t >= tNext
        t0 = max(0, State.t - cfg.telemetry.window_s);
        T(end+1)    = State.t; %#ok<AGROW>
        P11c(end+1) = pdr_of(TX,RX,'11p','CAM', t0, State.t);   %#ok<AGROW>
        P11d(end+1) = pdr_of(TX,RX,'11p','DENM',t0, State.t);   %#ok<AGROW>
        PCVc(end+1) = pdr_of(TX,RX,'cv2x','CAM', t0, State.t);  %#ok<AGROW>
        PCVd(end+1) = pdr_of(TX,RX,'cv2x','DENM',t0, State.t);  %#ok<AGROW>
        tNext = tNext + cfg.telemetry.update_s;
    end

    % 4.6 Avanzar tiempo
    State.t = State.t + cfg.dt;
end

%% 5) Gráfica y resumen
figure; hold on; grid on; title('PDR @100m (modelo estático (10/10))');
plot(T,P11c,'-','DisplayName','11p CAM');
plot(T,P11d,'-','DisplayName','11p DENM');
plot(T,PCVc,'-','DisplayName','C-V2X CAM');
plot(T,PCVd,'-','DisplayName','C-V2X DENM');
ylim([0 1]); xlabel('t (s)'); ylabel('PDR');
legend('Location','southwest');

fprintf('\n[Última ventana] 11p: CAM=%s DENM=%s | CV2X: CAM=%s DENM=%s\n', ...
    fmt_pct(P11c), fmt_pct(P11d), fmt_pct(PCVc), fmt_pct(PCVd));
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
selTX = strcmp({TX.tec},tec) & strcmp({TX.type},typ) & [TX.t_tx]>=t0 & [TX.t_tx]<t1 & [TX.hasRx100];
if ~any(selTX), p=NaN; return; end
denIds = unique([TX(selTX).id]);
selRX = strcmp({RX.tec},tec) & strcmp({RX.type},typ) & [RX.t_tx]>=t0 & [RX.t_tx]<t1 & [RX.ok];
okIds  = unique([RX(selRX).id]);
p = numel(intersect(denIds, okIds)) / numel(denIds);
p = min(max(double(p),0),1);
end

function s = fmt_pct(vec)
if isempty(vec) || isnan(vec(end)), s = '—';
else, s = sprintf('%.1f%%', 100*vec(end)); end
end
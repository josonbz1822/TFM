function [txList, occ] = mac80211p_step(Queues, Nodes, State, cfg)
% MAC80211P_STEP  — Emulación mínima de 802.11p (CSMA/CA abstracto)
% Selecciona transmisiones "oportunistas" por paso de simulación limitadas
% por una capacidad proporcional al ancho de banda 11p.
% Prioriza DENM > CAM y reparte entre nodos (máx. 1 por nodo y paso).
%
% Entradas mínimas:
%   Queues(i).pkts(j): struct con id,type,tec('11p'),t_enq,txNode,lenB
%   State.dt, State.t, State.spec.MHz_11p
%   cfg.cap_pps_10MHz (opc, por defecto 220)
%
% Salidas:
%   txList: array de struct con campos id,type,tec,txNode,t_tx
%   occ   : estimación de ocupación usada este paso (0..1)

% ---- parámetros y defensas ----
if ~isfield(State,'dt'), error('State.dt requerido'); end
MHz = 10; 
if isfield(State,'spec') && isfield(State.spec,'MHz_11p')
    MHz = max(1, double(State.spec.MHz_11p));
end
cap_pps_10 = 220;
if exist('cfg','var') && isfield(cfg,'cap_pps_10MHz'), cap_pps_10 = cfg.cap_pps_10MHz; end

% capacidad de "intentos" este paso
capStep = max(1, floor(cap_pps_10 * (MHz/10) * State.dt));

% ---- recolectar candidatos 11p (máx. 1 por nodo) ----
cands = [];  % [nodeIdx, pktIdx, isDENM]
for i = 1:numel(Queues)
    if isempty(Queues(i).pkts), continue; end
    % prioriza DENM sobre CAM en la cabeza de la cola de 11p
    idxD = find(strcmp({Queues(i).pkts.type},'DENM') & strcmp({Queues(i).pkts.tec},'11p'), 1, 'first');
    idxC = find(strcmp({Queues(i).pkts.type},'CAM')  & strcmp({Queues(i).pkts.tec},'11p'),  1, 'first');
    if ~isempty(idxD)
        cands = [cands; i, idxD, 1]; %#ok<AGROW>
    elseif ~isempty(idxC)
        cands = [cands; i, idxC, 0]; %#ok<AGROW>
    end
end

nCand = size(cands,1);
if nCand == 0
    txList = struct('id',{},'type',{},'tec',{},'txNode',{},'t_tx',{});
    occ    = 0;
    return;
end

% ---- ordenar por prioridad (DENM primero) y desempatar aleatorio ----
prio = cands(:,3);
% barajar dentro de cada prioridad
ordD = find(prio==1); ordC = find(prio==0);
ordD = ordD(randperm(numel(ordD)));
ordC = ordC(randperm(numel(ordC)));
order = [ordD; ordC];

% seleccionar hasta capacidad
selCount = min(capStep, nCand);
chosen = cands(order(1:selCount), :);

% ---- construir txList ----
txList = repmat(struct('id',uint64(0),'type','','tec','','txNode',uint32(0),'t_tx',0.0), selCount, 1);
for k = 1:selCount
    i = chosen(k,1); j = chosen(k,2);
    pk = Queues(i).pkts(j);
    if ~strcmp(pk.tec,'11p'), continue; end
    txList(k).id     = uint64(pk.id);
    txList(k).type   = char(pk.type);
    txList(k).tec    = '11p';
    txList(k).txNode = uint32(i);
    txList(k).t_tx   = State.t;
end

% ---- ocupación estimada (cuántos querían / cuántos caben) ----
occ = min(1, nCand / max(1, capStep));
end
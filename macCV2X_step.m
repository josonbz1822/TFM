function [txList, occ] = macCV2X_step(Queues, Nodes, State, cfg)


% ---- parámetros y defensas ----
if ~isfield(State,'t') || ~isfield(State,'dt')
    error('State.t y State.dt requeridos');
end
MHz = 10;
if isfield(State,'spec') && isfield(State.spec,'MHz_cv2x')
    MHz = max(1, double(State.spec.MHz_cv2x));
end
slot = 0.1;           % 100 ms
if exist('cfg','var') && isfield(cfg,'slot_cv2x'), slot = cfg.slot_cv2x; end
pResel = 0.2;
if exist('cfg','var') && isfield(cfg,'pResel'), pResel = cfg.pResel; end
cap_pps_10 = 220;
if exist('cfg','var') && isfield(cfg,'cap_pps_10MHz'), cap_pps_10 = cfg.cap_pps_10MHz; end

% ---- estado semipersistente por nodo (offset de slot) ----
persistent spsOffset   % vector tamaño numel(Queues): offset en [0,slot)
if isempty(spsOffset) || numel(spsOffset) ~= numel(Queues)
    spsOffset = rand(numel(Queues),1) * slot; % offset aleatorio inicial
end

% ¿estamos en borde de slot? (cada nodo usa su propio offset)
% criterio: si (t - offset) está muy cerca de múltiplo de slot
epsEdge = State.dt + 1e-9;

% ---- selección por nodo (máx. 1 pkt, prioriza DENM) ----
sel = []; % [nodeIdx, pktIdx]
nTx = 0;
for i = 1:numel(Queues)
    if isempty(Queues(i).pkts), continue; end

    % ¿toca su borde de slot ahora?
    if ~is_in_slot_edge(State.t, spsOffset(i), slot, epsEdge)
        continue;
    end

    % selección head-of-line priorizando DENM (solo paquetes cv2x)
    idxD = find(strcmp({Queues(i).pkts.type},'DENM') & strcmp({Queues(i).pkts.tec},'cv2x'), 1, 'first');
    idxC = find(strcmp({Queues(i).pkts.type},'CAM')  & strcmp({Queues(i).pkts.tec},'cv2x'),  1, 'first');
    if ~isempty(idxD)
        sel = [sel; i, idxD]; %#ok<AGROW>
    elseif ~isempty(idxC)
        sel = [sel; i, idxC]; %#ok<AGROW>
    end

    % reselección ocasional del offset (SB-SPS reselection)
    if rand < pResel
        spsOffset(i) = rand * slot;
    end
end

nSel = size(sel,1);
if nSel == 0
    txList = struct('id',{},'type',{},'tec',{},'txNode',{},'t_tx',{});
    occ    = 0;
    return;
end

% ---- construir lista de TX ----
txList = repmat(struct('id',uint64(0),'type','','tec','','txNode',uint32(0),'t_tx',0.0), nSel, 1);
for k = 1:nSel
    i = sel(k,1); j = sel(k,2);
    pk = Queues(i).pkts(j);
    if ~strcmp(pk.tec,'cv2x'), continue; end
    txList(k).id     = uint64(pk.id);
    txList(k).type   = char(pk.type);
    txList(k).tec    = 'cv2x';
    txList(k).txNode = uint32(i);
    txList(k).t_tx   = State.t;  % instante de borde de slot
end

% ---- ocupación aproximada del slot actual ----
capSlot = cap_pps_10 * (MHz/10) * slot;
occ     = min(1, nSel / max(1, capSlot));
end

% ===== helpers locales =====
function tf = is_in_slot_edge(t, offset, slot, epsEdge)
% devuelve true si (t - offset) ≈ m*slot (con tolerancia epsEdge)
x = t - offset;
if x < -epsEdge
    tf = false;
else
    m = round(x / slot);
    tf = abs(x - m*slot) <= epsEdge;
end
end

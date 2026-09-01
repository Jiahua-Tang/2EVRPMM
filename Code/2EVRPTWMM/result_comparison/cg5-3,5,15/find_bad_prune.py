import re

# ---- 1) both runs' final solutions -----------------------------------------
local_routes = [
    [4,20,4],[4,16,13,4],[6,17,15,6],[6,9,22,21,6],[4,19,18,10,14,4],[6,11,23,12,6],
]
cluster_routes = [
    [4,20,4],[4,16,13,4],[6,21,22,10,14,4],[6,9,11,23,6],[6,17,15,6],[4,19,18,12,6],
]

def build_facts(routes):
    together = set()
    start_of = {}
    end_of = {}
    start_count = {}
    for r in routes:
        sat_start, sat_end = r[0], r[-1]
        custs = r[1:-1]
        for c in custs:
            start_of[c] = sat_start
            end_of[c] = sat_end
        for i in range(len(custs)):
            for j in range(i+1, len(custs)):
                together.add(frozenset((custs[i], custs[j])))
        start_count[sat_start] = start_count.get(sat_start, 0) + 1
    total_routes = len(routes)
    return dict(together=together, start_of=start_of, end_of=end_of,
                start_count=start_count, total_routes=total_routes)

local_facts = build_facts(local_routes)
cluster_facts = build_facts(cluster_routes)

def consistent(node_type, args, kind, facts):
    # returns True/False/None(unknown)
    if node_type == 'cc':
        a, b = args
        if a not in facts['start_of'] or b not in facts['start_of']:
            return None
        tog = frozenset((a, b)) in facts['together']
        return tog if kind == 'must' else (not tog)
    if node_type == 'pc':  # start-parking-customer
        p, c = args
        if c not in facts['start_of']:
            return None
        match = facts['start_of'][c] == p
        return match if kind == 'must' else (not match)
    if node_type == 'epc':  # end-parking-customer
        p, c = args
        if c not in facts['end_of']:
            return None
        match = facts['end_of'][c] == p
        return match if kind == 'must' else (not match)
    if node_type == 'satfleet':
        s, bound = args  # bound = (floor_val, ceil_val)
        floor_v, ceil_v = bound
        cnt = facts['start_count'].get(s, 0)
        if kind == 'must':   # lower bound >= ceil
            return cnt >= ceil_v
        else:                # upper bound <= floor
            return cnt <= floor_v
    if node_type == 'totroutes':
        floor_v, ceil_v = args
        cnt = facts['total_routes']
        if kind == 'must':
            return cnt >= ceil_v
        else:
            return cnt <= floor_v
    return None

# ---- 2) parse a log file, rebuild the B&P tree -----------------------------
re_display = re.compile(r"^Display selected node (\d+) in level (\d+), parent node (\d+): from \d+ nodes(?:, current upper bound = (.+))?$")
re_cc      = re.compile(r"^Branch on combination customer-customer: \((\d+), (\d+)\)$")
re_pc      = re.compile(r"^Branch on combination parking-customer: \((\d+), (\d+)\)$")
re_epc     = re.compile(r"^Branch on combination end-parking-customer: \((\d+), (\d+)\)$")
re_satfleet= re.compile(r"^Branch on satellite (\d+) fleet: (\d+) / (\d+)$")
re_totroutes=re.compile(r"^Branch on total number of 2e routes:\s*([\d.]+), ([\d.]+)$")
re_child   = re.compile(r"^Solve child node (\d+)$")
re_lp      = re.compile(r"^LP result of column generation: ([\d.eE+-]+)$")
re_prune   = re.compile(r"^([\d.eE+-]+), Exceed Upper Bound, prune$")
re_integer = re.compile(r"^Integer solution found$")

def parse(path):
    lines = open(path, encoding="utf-8", errors="replace").read().splitlines()
    node_parent = {0: None}
    node_constraint = {}  # node_id -> (node_type, args, kind)
    prunes = []

    current_selected = None
    current_ub = None
    pending = None  # ('cc'/'pc'/'epc', args) or ('satfleet', (s,floor,ceil)) or ('totroutes',(floor,ceil))
    pending_children = []

    i = 0
    n = len(lines)
    while i < n:
        line = lines[i].strip()
        m = re_display.match(line)
        if m:
            nid, lvl, parent = int(m.group(1)), int(m.group(2)), int(m.group(3))
            current_selected = nid
            current_ub = m.group(4)
            if nid not in node_parent:
                node_parent[nid] = parent
            i += 1
            continue
        m = re_cc.match(line)
        if m:
            pending = ('cc', (int(m.group(1)), int(m.group(2))))
            pending_children = []
            i += 1
            continue
        m = re_pc.match(line)
        if m:
            pending = ('pc', (int(m.group(1)), int(m.group(2))))
            pending_children = []
            i += 1
            continue
        m = re_epc.match(line)
        if m:
            pending = ('epc', (int(m.group(1)), int(m.group(2))))
            pending_children = []
            i += 1
            continue
        m = re_satfleet.match(line)
        if m:
            pending = ('satfleet', (int(m.group(1)), int(m.group(2)), int(m.group(3))))
            pending_children = []
            i += 1
            continue
        m = re_totroutes.match(line)
        if m:
            pending = ('totroutes', (float(m.group(1)), float(m.group(2))))
            pending_children = []
            i += 1
            continue
        m = re_child.match(line)
        if m:
            cid = int(m.group(1))
            pending_children.append(cid)
            kind = 'must' if len(pending_children) == 1 else 'forbidden'
            node_parent[cid] = current_selected
            ptype = pending[0]
            if ptype == 'satfleet':
                s, floor_v, ceil_v = pending[1]
                node_constraint[cid] = ('satfleet', (s, (floor_v, ceil_v)), kind)
            elif ptype == 'totroutes':
                floor_v, ceil_v = pending[1]
                node_constraint[cid] = ('totroutes', (floor_v, ceil_v), kind)
            else:
                node_constraint[cid] = (ptype, pending[1], kind)
            i += 1
            continue
        m = re_prune.match(line)
        if m:
            lb = float(m.group(1))
            prunes.append((current_selected, lb, current_ub))
            i += 1
            continue
        i += 1

    return node_parent, node_constraint, prunes

def ancestry(node_parent, node_constraint, nid):
    chain = []
    cur = nid
    while cur is not None and cur != 0:
        if cur in node_constraint:
            chain.append((cur, *node_constraint[cur]))
        cur = node_parent.get(cur)
        if cur == 0:
            break
    return list(reversed(chain))

def label(node_type, args, kind):
    if node_type == 'cc':
        return f"customers {args} must be {'TOGETHER' if kind=='must' else 'APART'}"
    if node_type == 'pc':
        return f"route serving customer {args[1]} must {'START' if kind=='must' else 'NOT start'} at satellite {args[0]}"
    if node_type == 'epc':
        return f"route serving customer {args[1]} must {'END' if kind=='must' else 'NOT end'} at satellite {args[0]}"
    if node_type == 'satfleet':
        s, (fl, ce) = args
        return f"#routes from satellite {s} must be {'>= '+str(ce) if kind=='must' else '<= '+str(fl)}"
    if node_type == 'totroutes':
        fl, ce = args
        return f"total #2e routes must be {'>= '+str(ce) if kind=='must' else '<= '+str(fl)}"
    return "?"

def analyze(log_path, target_facts, target_cost, other_name):
    node_parent, node_constraint, prunes = parse(log_path)
    print(f"\n===== {log_path}: {len(node_parent)} nodes, {len(prunes)} 'Exceed Upper Bound' prunes =====")
    bad = []
    for nid, lb, ub in prunes:
        chain = ancestry(node_parent, node_constraint, nid)
        ok = True
        for (cid, ntype, args, kind) in chain:
            r = consistent(ntype, args, kind, target_facts)
            if r is False:
                ok = False
                break
        if ok:
            bad.append((nid, lb, ub, chain))
    bad.sort(key=lambda x: len(x[3]))
    print(f"{len(bad)} pruned nodes are consistent with {other_name}'s solution (cost {target_cost}).")
    for nid, lb, ub, chain in bad[:3]:
        print(f"\n--- node {nid} (depth {len(chain)}) ---")
        print(f"  reported cgLowerBound = {lb}   upperBound at prune time = {ub}")
        if lb > target_cost:
            print(f"  {lb} > {target_cost} -> CONTRADICTION (subtree provably contains a cheaper solution)")
        for (cid, ntype, args, kind) in chain:
            print(f"    node {cid}: {label(ntype, args, kind)}")

analyze("local.txt", cluster_facts, 328.36, "cluster (true optimal)")
analyze("cluster.txt", local_facts, 329.47, "local")

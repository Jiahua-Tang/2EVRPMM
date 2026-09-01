import re, sys

# ---- 1) local.txt's winning solution -> "together" pairs -------------------
local_routes_2e = [
    [28,30,21],[24,66,62],[6,34,27],[74,42],[38,17,56,39],[65,20,15,72],
    [71,9,46],[10,33,47],[57,51,60],[77,37,7,80],[19,40,69,53],[25,23],
    [61,70],[44,18,13],[64,29,49],[16,79],[12,76,31],[63,58,48,52],
    [78,11,35],[45,59,14],[22,75,43,26],[68,50,54,8],[32,55,67],[36,41,73],
]
together = set()
route_of = {}
for idx, r in enumerate(local_routes_2e):
    for c in r:
        route_of[c] = idx
    for i in range(len(r)):
        for j in range(i+1, len(r)):
            together.add(frozenset((r[i], r[j])))

def is_together(a, b):
    return frozenset((a, b)) in together

def is_consistent(a, b, kind):
    # both customers must actually be assigned somewhere in local's solution
    if a not in route_of or b not in route_of:
        return None  # can't judge (shouldn't happen, all customers routed)
    tog = is_together(a, b)
    if kind == 'must':
        return tog
    else:
        return not tog

# ---- 1b) cluster.txt's OWN final accepted solution -> "together" pairs -----
cluster_routes_2e = [
    [28,30,21],[45,59,14],[10,33,47],[24,66,62],[6,34,27],[57,51,60],
    [64,29,49],[38,17,56,39],[19,40,69,53],[77,37,7,80],[74,42],[71,9,46],
    [61,70],[12,58,52],[25,23],[16,79],[63,32,76,31],[44,18,13],
    [78,11,35],[22,75,43,26],[68,50,54,8],[48,55,67],[36,41,73],[65,20,15,72],
]
cluster_together = set()
cluster_route_of = {}
for idx, r in enumerate(cluster_routes_2e):
    for c in r:
        cluster_route_of[c] = idx
    for i in range(len(r)):
        for j in range(i+1, len(r)):
            cluster_together.add(frozenset((r[i], r[j])))

def is_together_cluster(a, b):
    return frozenset((a, b)) in cluster_together

# ---- 2) parse cluster.txt, rebuild the B&P tree ----------------------------
path = "cluster.txt"
lines = open(path, encoding="utf-8", errors="replace").read().splitlines()

node_parent = {0: None}
node_constraint = {}   # node_id -> (kind, a, b)
node_lb = {}           # node_id -> cgLowerBound at creation
node_level = {0: 0}

re_display = re.compile(r"^Display selected node (\d+) in level (\d+), parent node (\d+): from \d+ nodes, current upper bound = (.+)$")
re_branch  = re.compile(r"^Branch on combination customer-customer: \((\d+), (\d+)\)$")
re_child   = re.compile(r"^Solve child node (\d+)$")
re_lp      = re.compile(r"^LP result of column generation: ([\d.eE+-]+)$")
re_prune   = re.compile(r"^([\d.eE+-]+), Exceed Upper Bound, prune$")
re_integer = re.compile(r"^Integer solution found$")

current_selected = None
current_ub_at_select = None
pending_pair = None
pending_children = []  # list of child ids waiting for their LP result, in creation order
prunes = []            # (node_id, reported_lb, ub_at_time)
integers = []

i = 0
while i < len(lines):
    line = lines[i].strip()
    m = re_display.match(line)
    if m:
        nid, lvl, parent, ub = int(m.group(1)), int(m.group(2)), int(m.group(3)), m.group(4)
        current_selected = nid
        current_ub_at_select = ub
        node_level[nid] = lvl
        if nid not in node_parent:
            node_parent[nid] = parent
        i += 1
        continue
    m = re_branch.match(line)
    if m:
        pending_pair = (int(m.group(1)), int(m.group(2)))
        pending_children = []
        i += 1
        continue
    m = re_child.match(line)
    if m:
        cid = int(m.group(1))
        pending_children.append(cid)
        # look ahead for the LP result line (may not be immediately next due to blank lines)
        j = i + 1
        lb = None
        while j < len(lines) and j < i + 5:
            mm = re_lp.match(lines[j].strip())
            if mm:
                lb = float(mm.group(1))
                break
            if re_child.match(lines[j].strip()) or re_display.match(lines[j].strip()):
                break
            j += 1
        kind = 'must' if len(pending_children) == 1 else 'forbidden'
        node_parent[cid] = current_selected
        node_constraint[cid] = (kind, pending_pair[0], pending_pair[1])
        if lb is not None:
            node_lb[cid] = lb
        i += 1
        continue
    m = re_prune.match(line)
    if m:
        lb = float(m.group(1))
        prunes.append((current_selected, lb, current_ub_at_select))
        i += 1
        continue
    if re_integer.match(line):
        integers.append(current_selected)
        i += 1
        continue
    i += 1

print(f"Parsed {len(node_parent)} nodes, {len(prunes)} 'Exceed Upper Bound' prune events, {len(integers)} integer-solution leaves.")

def ancestry_constraints(nid):
    chain = []
    cur = nid
    while cur is not None and cur != 0:
        if cur in node_constraint:
            chain.append((cur, *node_constraint[cur]))
        cur = node_parent.get(cur)
        if cur == 0:
            break
    return list(reversed(chain))

# ---- 3) find pruned nodes whose accumulated constraints are consistent with
#         local's winning solution -------------------------------------------
bad = []
for nid, lb, ub in prunes:
    chain = ancestry_constraints(nid)
    consistent = True
    unknown = False
    for (cid, kind, a, b) in chain:
        r = is_consistent(a, b, kind)
        if r is None:
            unknown = True
        elif r is False:
            consistent = False
            break
    if consistent:
        bad.append((nid, lb, ub, chain, unknown))

bad.sort(key=lambda x: len(x[3]))  # shallowest first

print(f"\n{len(bad)} pruned nodes are consistent with local's 1173.73 solution (i.e. their subtree contains it).")
print("Shallowest such node = the branch that was pruned incorrectly:\n")

for nid, lb, ub, chain, unknown in bad[:5]:
    print(f"--- node {nid} (depth {len(chain)}) ---")
    print(f"  reported cgLowerBound = {lb}   upperBound at prune time = {ub}")
    print(f"  local's true solution cost in this subtree = 1173.73  -> {lb} > 1173.73 is a CONTRADICTION" if lb > 1173.73 else "  (bound is actually <= 1173.73, not a contradiction by itself)")
    print("  path of constraints from root, and whether CLUSTER's own accepted solution agrees:")
    split_node = None
    for (cid, kind, a, b) in chain:
        want = (kind == 'must')
        local_ok = is_together(a, b) == want
        cluster_ok = is_together_cluster(a, b) == want
        flag = "" if cluster_ok else "   <-- cluster's own final solution VIOLATES this (first split from node's path)" if split_node is None else "   <-- also violates"
        if not cluster_ok and split_node is None:
            split_node = cid
        print(f"    node {cid}: ({a},{b}) must be {'TOGETHER' if kind=='must' else 'APART'}"
              f"  -> local: {'ok' if local_ok else 'VIOLATES'}, cluster's own solution: {'ok' if cluster_ok else 'VIOLATES'}{flag}")
    if split_node is not None:
        print(f"  => cluster's actual accepted solution branches away from this path at node {split_node};")
        print(f"     i.e. it never explores past node {split_node} down the (would-be-correct) side, it took the sibling branch instead.")
    else:
        print("  => cluster's own final solution is ALSO fully consistent with this whole path (unexpected).")
    if unknown:
        print("  (NOTE: at least one customer in this chain was not found in local's route list)")
    print()

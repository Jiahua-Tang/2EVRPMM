import os
import random
import glob

DATA_DIR = ("/Users/jiahua/Library/CloudStorage/"
            "OneDrive-EcoledeManagementdeNormandie/"
            "2EVRPMM/Data/Instances/Data")

CUSTOMERS_FROM = 30
CUSTOMERS_TO   = 20
DELETE_N       = CUSTOMERS_FROM - CUSTOMERS_TO  # 10

def parse_instance(filepath):
    """Return (customers, others) where each element is a raw line string."""
    customers, others = [], []
    with open(filepath) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 6:          # customer: x y tw_a tw_b demand svc
                customers.append(line.rstrip("\n"))
            elif len(parts) > 0:         # satellite / depot
                others.append(line.rstrip("\n"))
            # empty lines are dropped; one trailing newline is re-added on write
    return customers, others

def write_instance(filepath, customers, others):
    with open(filepath, "w") as f:
        for line in customers:
            f.write(line + "\n")
        for line in others:
            f.write(line + "\n")
        f.write("\n")

pattern = os.path.join(DATA_DIR, f"*,{CUSTOMERS_FROM}.txt")
files = sorted(glob.glob(pattern))

if not files:
    raise FileNotFoundError(f"No files matching {pattern}")

print(f"Found {len(files)} instances with {CUSTOMERS_FROM} customers in:\n  {DATA_DIR}\n")

created = skipped = 0
for src in files:
    fname   = os.path.basename(src)
    dst_fname = fname.replace(f",{CUSTOMERS_FROM}.txt", f",{CUSTOMERS_TO}.txt")
    dst = os.path.join(DATA_DIR, dst_fname)

    customers, others = parse_instance(src)
    if len(customers) != CUSTOMERS_FROM:
        print(f"  SKIP {fname}: expected {CUSTOMERS_FROM} customers, found {len(customers)}")
        continue

    # Seed from the numeric id embedded in the filename (e.g. ce3-... → 3)
    # so results are deterministic and differ per instance family member
    try:
        id_part = fname.split("-")[0]          # e.g. "ce3"
        instance_num = int("".join(c for c in id_part if c.isdigit()))
    except ValueError:
        instance_num = 0
    rng = random.Random(instance_num * 137)    # spread seeds
    selected = sorted(rng.sample(range(len(customers)), CUSTOMERS_TO))
    new_customers = [customers[i] for i in selected]

    if os.path.exists(dst):
        print(f"  SKIP {dst_fname}  (already exists)")
        skipped += 1
        continue

    write_instance(dst, new_customers, others)
    print(f"  {dst_fname}  (new)  kept customers: {[i+1 for i in selected]}")
    created += 1

print(f"\nDone — {created} files written, {skipped} skipped.")

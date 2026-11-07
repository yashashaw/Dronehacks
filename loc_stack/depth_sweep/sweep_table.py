import json, glob
hdr = ["variant", "left", "L Hz", "Lgap", "depth", "D Hz", "VIO Hz", "VIOgap", "age p95", "age max"]
print("%-14s %-10s %6s %5s %-9s %6s %7s %6s %8s %8s" % tuple(hdr))
for f in sorted(glob.glob("*.json")):
    d = json.load(open(f)); L = d.get("left") or {}; D = d.get("depth") or {}; V = d.get("vio") or {}
    a = V.get("age_ms") or {}
    print("%-14s %-10s %6s %5s %-9s %6s %7s %6s %8s %8s" % (f[:-5], L.get("size"), L.get("rate_hz"), L.get("gaps_over_1p5"),
          D.get("size"), D.get("rate_hz"), V.get("rate_hz"), V.get("gaps_over_1p5"), a.get("p95"), a.get("max")))

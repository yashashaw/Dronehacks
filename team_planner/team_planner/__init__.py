import os

# numba writes its compiled code next to the source by default, which is read-only or recreated in some
# containers (then every start recompiles). Point it at the team workspace when that exists.
_cache = os.path.join(os.environ.get("TEAM_WS", "/root/team_ws"), ".numba_cache")
if "NUMBA_CACHE_DIR" not in os.environ:
    try:
        os.makedirs(_cache, exist_ok=True)
        os.environ["NUMBA_CACHE_DIR"] = _cache
    except OSError:
        pass

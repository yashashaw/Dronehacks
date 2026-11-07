"""Write one camera-pipeline variant (yaml + launch) for the depth/latency sweep.

    python3 make_variant.py NAME OUTDIR      (inside skybeta-stack)

Starts from the image's own stock oakd_vslam.yaml, so it always matches the driver.
'd_base' reproduces exactly what run_slam.sh's depth profile builds.
"""
import os, subprocess, sys, yaml

DEPTH_BASE = {
    'pipeline_gen.i_pipeline_type': 'depth',
    'stereo.i_publish_topic': True,
    'stereo.i_depth_preset': 'DEFAULT',
    'stereo.i_subpixel': False,
    'stereo.i_extended_disparity': False,   # as deployed (v3 calls it i_extended_disp; inert)
    'stereo.i_lr_check': True,
}
Q2 = {'left.i_max_q_size': 2, 'right.i_max_q_size': 2, 'stereo.i_max_q_size': 2}

VARIANTS = {
    'stock':      {},
    'd_base':     dict(DEPTH_BASE),
    'd_q2':       {**DEPTH_BASE, **Q2},
    'd_nolr':     {**DEPTH_BASE, 'stereo.i_lr_check': False},
    'd_noalign':  {**DEPTH_BASE, 'stereo.i_aligned': False},
    'd_960':      {**DEPTH_BASE, 'left.i_width': 960, 'left.i_height': 600,
                   'right.i_width': 960, 'right.i_height': 600},
    'd_20fps':    {**DEPTH_BASE, 'left.i_fps': 20.0, 'right.i_fps': 20.0},
    'd_fastd':    {**DEPTH_BASE, 'stereo.i_depth_preset': 'FAST_DENSITY'},
}
# combos are added on the command line as a+b (e.g. d_q2+d_nolr)


def overrides(name):
    o = {}
    for part in name.split('+'):
        if part not in VARIANTS:
            sys.exit(f'unknown variant {part}; have {sorted(VARIANTS)}')
        o.update(VARIANTS[part])
    return o


def main():
    name, out = sys.argv[1], sys.argv[2]
    share = subprocess.check_output(['ros2', 'pkg', 'prefix', '--share', 'oakd_vslam_bringup'],
                                    text=True).strip()
    cfg = yaml.safe_load(open(f'{share}/config/oakd_vslam.yaml'))
    params = next(iter(cfg.values()))['ros__parameters']
    for key, val in overrides(name).items():
        sec, k = key.split('.')
        params.setdefault(sec, {})[k] = val
    os.makedirs(out, exist_ok=True)
    safe = name.replace('+', '__')
    yp = f'{out}/{safe}.yaml'
    yaml.safe_dump(cfg, open(yp, 'w'), sort_keys=False)
    launch = open(f'{share}/launch/oakd_vslam.launch.py').read()
    old = "os.path.join(bringup_share, 'config', 'oakd_vslam.yaml')"
    if old not in launch:
        sys.exit('launch file has no config path to replace')
    lp = f'{out}/{safe}.launch.py'
    open(lp, 'w').write(launch.replace(old, repr(yp)))
    subprocess.check_call([sys.executable, '-m', 'py_compile', lp])
    print(lp)


if __name__ == '__main__':
    main()

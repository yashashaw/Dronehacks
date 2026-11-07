#!/usr/bin/env python3
"""Write an interactive 3D view of a walk_logger run: <run>/map.html (open it in a browser).

  python plot_walk.py runs/object1 --object 0.60,0.40

Shows the VIO path, the dwell points (your marks), the traced outline through the first four
dwells, and, if --object is given, the TRUE rectangle (your measured L x W) laid over the
traced one, anchored at corner 1 along the corner1->corner2 direction, so you can see the
shape and scale error at a glance. Drag to rotate, wheel to zoom. Standard library only.

This is a 3D trace of where the drone went, not a depth map of the scene: the stack runs the
stereo pair for cuVSLAM and publishes no depth or point cloud.
"""
import argparse
import json
import math
import os

import analyze_walk as aw


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def add(a, b):
    return [x + y for x, y in zip(a, b)]


def mul(a, k):
    return [x * k for x in a]


def unit(a):
    n = math.sqrt(sum(x * x for x in a))
    return [x / n for x in a] if n > 1e-9 else a


def true_rectangle(corners, length, width):
    """True L x W rectangle at corner 1, along corner1->corner2, in the traced plane."""
    c0, c1, c3 = corners[0], corners[1], corners[3]
    u = unit(sub(c1, c0))
    v = sub(c3, c0)
    v = unit(sub(v, mul(u, sum(a * b for a, b in zip(v, u)))))   # make v perpendicular to u
    # corner1->corner2 is the first measured edge: use whichever of L, W it is closer to
    d01 = math.dist(c0, c1)
    a, b = (length, width) if abs(d01 - length) <= abs(d01 - width) else (width, length)
    p1, p2, p3 = add(c0, mul(u, a)), add(add(c0, mul(u, a)), mul(v, b)), add(c0, mul(v, b))
    return [c0, p1, p2, p3, c0]


def build(run, obj, lm_radius=6.0):
    rows = aw.load_odom(run)
    dwells = aw.find_dwells(rows)
    step = max(1, len(rows) // 1500)
    path = [[r['x'], r['y'], r['z']] for r in rows[::step]]
    marks = [list(d['pos']) for d in dwells]
    data = {'path': path, 'marks': marks, 'traced': None, 'true': None, 'title': os.path.basename(run),
            'slam': None, 'lm': None}
    sp = os.path.join(run, 'slam_odom.csv')
    if os.path.exists(sp):
        import csv
        srows = list(csv.DictReader(open(sp)))
        data['slam'] = [[float(r['x']), float(r['y']), float(r['z'])] for r in srows[::max(1, len(srows) // 1500)]]
    lp = os.path.join(run, 'landmarks.csv')
    if os.path.exists(lp):
        pts = [tuple(map(float, l.split(','))) for l in open(lp) if l[0] not in '#x' and l.strip()]
        near = [q for q in pts if min(math.dist(q, c) for c in path[::20]) <= lm_radius]
        data['lm'] = [[round(v, 3) for v in q] for q in near[::max(1, len(near) // 6000)]]
        data['lm_info'] = f'{len(near)} of {len(pts)} landmarks within {lm_radius} m of the path'
    if obj and len(marks) >= 4:
        c = marks[:4]
        data['traced'] = c + [c[0]]
        data['true'] = true_rectangle(c, obj[0], obj[1])
        rep = aw.object_report(dwells, obj[0], obj[1])
        data['text'] = (f"edges {rep['edges_m']} m vs {rep['expected_edges_m']} m | "
                        f"scale {rep['scale_factor_measured_over_true']}x | "
                        f"angles {rep['corner_angles_deg']}")
    return data


PAGE = """<!doctype html><meta charset=utf-8><title>VIO trace</title>
<style>html,body{margin:0;height:100%;background:#0f1115;color:#d8dde6;font:13px system-ui}
canvas{display:block;width:100%;height:100%}#t{position:fixed;left:12px;top:10px;line-height:1.5}
b{color:#fff}</style><div id=t></div><canvas id=c></canvas>
<script>
const D=__DATA__;
const cv=document.getElementById('c'),g=cv.getContext('2d');
let yaw=-0.7,pitch=0.5,zoom=1,px=0,py=0,drag=null;
const all=[].concat(D.path,D.marks,D.traced||[],D.true||[],D.slam||[]);
const mn=[0,1,2].map(i=>Math.min(...all.map(p=>p[i]))),mx=[0,1,2].map(i=>Math.max(...all.map(p=>p[i])));
const ctr=mn.map((v,i)=>(v+mx[i])/2),span=Math.max(...mn.map((v,i)=>mx[i]-v),0.3);
function P(p){let x=p[0]-ctr[0],y=p[1]-ctr[1],z=p[2]-ctr[2];
 let x1=x*Math.cos(yaw)-y*Math.sin(yaw),y1=x*Math.sin(yaw)+y*Math.cos(yaw);
 let y2=y1*Math.cos(pitch)-z*Math.sin(pitch),z2=y1*Math.sin(pitch)+z*Math.cos(pitch);
 const s=Math.min(cv.width,cv.height)*0.42/span*zoom;return[cv.width/2+px+x1*s,cv.height/2+py-z2*s];}
function line(pts,col,w,dash){g.beginPath();pts.forEach((p,i)=>{const q=P(p);i?g.lineTo(q[0],q[1]):g.moveTo(q[0],q[1])});
 g.strokeStyle=col;g.lineWidth=w;g.setLineDash(dash||[]);g.stroke();g.setLineDash([]);}
function dot(p,col,r,label){const q=P(p);g.fillStyle=col;g.beginPath();g.arc(q[0],q[1],r,0,7);g.fill();
 if(label){g.fillStyle='#fff';g.fillText(label,q[0]+8,q[1]-6)}}
function draw(){cv.width=innerWidth*devicePixelRatio;cv.height=innerHeight*devicePixelRatio;g.font=14*devicePixelRatio+'px system-ui';
 const f=span/4,o=[ctr[0]-span/2,ctr[1]-span/2,mn[2]];
 for(let i=0;i<=4;i++){line([[o[0]+i*f,o[1],o[2]],[o[0]+i*f,o[1]+span,o[2]]],'#262b36',1);line([[o[0],o[1]+i*f,o[2]],[o[0]+span,o[1]+i*f,o[2]]],'#262b36',1)}
 line([[o[0],o[1],o[2]],[o[0]+f,o[1],o[2]]],'#e5534b',3);line([[o[0],o[1],o[2]],[o[0],o[1]+f,o[2]]],'#57ab5a',3);line([[o[0],o[1],o[2]],[o[0],o[1],o[2]+f]],'#539bf5',3);
 if(D.lm){g.fillStyle='rgba(120,170,255,0.55)';D.lm.forEach(p=>{const q=P(p);g.fillRect(q[0],q[1],2*devicePixelRatio,2*devicePixelRatio)})}
 if(D.slam)line(D.slam,'#bc8cff',2*devicePixelRatio);
 line(D.path,'#6e7681',1.5*devicePixelRatio);
 if(D.true)line(D.true,'#57ab5a',2.5*devicePixelRatio,[10,6]);
 if(D.traced)line(D.traced,'#f0883e',3*devicePixelRatio);
 D.marks.forEach((m,i)=>dot(m,'#f0883e',5*devicePixelRatio,String(i+1)));
 const L=[`<b>${D.title}</b>`,'grey: VIO path'+(D.slam?'  purple: SLAM path':'')+(D.lm?'  blue dots: SLAM landmarks':'')+(D.marks.length?'  orange: holds':'')+(D.true?'  green dashed: true rectangle':''),
  `axes (red x, green y, blue z up), grid cell = ${f.toFixed(2)} m`];if(D.text)L.push(D.text);if(D.lm_info)L.push(D.lm_info);
 document.getElementById('t').innerHTML=L.join('<br>');}
cv.onmousedown=e=>drag=[e.clientX,e.clientY,e.shiftKey];onmouseup=()=>drag=null;
onmousemove=e=>{if(!drag)return;const dx=e.clientX-drag[0],dy=e.clientY-drag[1];drag[0]=e.clientX;drag[1]=e.clientY;
 if(drag[2]){px+=dx*devicePixelRatio;py+=dy*devicePixelRatio}else{yaw+=dx*0.01;pitch=Math.max(-1.5,Math.min(1.5,pitch+dy*0.01))}draw()};
cv.onwheel=e=>{e.preventDefault();zoom*=e.deltaY<0?1.1:0.9;draw()};onresize=draw;draw();
</script>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--object', help='"LENGTH,WIDTH" metres')
    ap.add_argument('--lm-radius', type=float, default=6.0, help='drop landmarks farther than this from the path')
    a, _ = ap.parse_known_args()   # tolerate analyze_walk-only flags such as --skip
    obj = tuple(float(v) for v in a.object.split(',')) if a.object else None
    data = build(a.run, obj, a.lm_radius)
    out = os.path.join(a.run, 'map.html')
    with open(out, 'w') as f:
        f.write(PAGE.replace('__DATA__', json.dumps(data)))
    print('wrote', out)


if __name__ == '__main__':
    main()

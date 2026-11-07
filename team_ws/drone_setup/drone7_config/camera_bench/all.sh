#!/bin/bash
# Run every config back to back (~7 min). On the drone, in ~/cam_bench, stack stopped:
#   nohup bash all.sh > all.log 2>&1 &      results: <config>.out (RESULT {...} json line)
for c in A_stereo_1024 B1_depth_raw_640 B2_depth_rect_640 C_depth_rect_1024 D_depth_rect_1280 E_depth_raw_1024; do
  ./run.sh $c.yaml $c 40 > $c.out 2>&1; sleep 3
done
echo ALLDONE > all.done

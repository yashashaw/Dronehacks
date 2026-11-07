# Fixing drift: plan 3 vs plan 4

Written 2026-11-07, before the first real test flights. Numbers come from the sim
(`sim/HANDOFF.md`) and drone 7's bench tests (`loc_stack/facts/drone7/SUMMARY.md`).

## The problem

The drone knows where it is from VIO (camera + motion tracking), and VIO **drifts**:
- bench, hand-carried: read 2.11 m for a real 2.44 m (about 13 % short)
- sim: up to a 0.65 m jump during an in-place turn (run_016); the controller chased the
  bad estimate into a wall
- lanes leave only about ±0.2 m of room either side, so small errors mean touching

Other limits:
- **depth is off on the real drone.** The camera link is nearly full with the VIO images
  (`skybeta/src/oakd_vslam_bringup/config/oakd_vslam.yaml`). Lower resolution should make
  room for depth computed on the camera: about 61 MB/s at 640x400 vs a ~120-140 MB/s ceiling.
  Not yet tested on the drone.
- **depth can't see the net.** In sim it read 2.9-3.2 m with the net 2.14 m away. Solid walls
  are fine.
- PX4's built-in estimator (EKF2) has no input for "the wall is X m away", so depth can't be
  fed to it directly.

## Plan 4: map + wall checks

Plan the path from the course map **before the flight**. For each leg, the map says how far
the wall ahead should be when we're at the turn. In flight, stop and turn when the camera
reads that distance, not when the VIO estimate says we're there.

How it works:
1. Course map (`dronehacks-course-v3.obj`) -> grid -> A* -> simplify into straight legs.
   The corners are the turns.
2. For each leg, ray-cast in the map from the corner along the flight direction: distance to
   what's ahead, and whether it's a solid wall or the net.
3. In flight: fly the leg and read the centre of the depth image (a middle percentile, not the
   closest pixel; run_014 showed the closest pixels are noise).
   - solid wall ahead: stop when depth reads the expected distance. Drift is reset here.
   - net ahead: stop on the VIO estimate with a bigger margin.
4. Extras: an emergency brake if centre depth is very close, and a sanity check that lands if
   depth and VIO disagree by more than ~0.4 m.

On the sim route, legs 2, 3 and 4 end at solid walls (M2, barrier, M3) and can self-correct;
legs 1, 5 and 6 end at the net and can't.

**Pros**
- small change on top of the best sim run's logic (`team_flight_test`)
- planning happens before flight and is testable offline
- worst case is turning a bit early or late (capped), not flying on a wrong position
- no changes to PX4 or the VIO pipeline

**Cons**
- fixes drift only at walls; between walls we still trust VIO
- a sudden mid-leg jump isn't caught until the next wall
- no help on legs that end at the net

## Plan 3: our own estimator

Write our own position estimator on the Jetson that constantly compares camera wall distances
with the map, corrects the position, and feeds the result to PX4 through the existing vision
input (`/fmu/in/vehicle_visual_odometry`). It doesn't replace EKF2; it sits in front of it.

**Pros**
- fixes drift all the time, not just at walls
- catches mid-leg jumps whenever a wall is in view
- most general, best story for the judges

**Cons**
- much more work: estimation maths, tuning, ~60 ms camera delay
- must know *which* wall it's seeing; a wrong match moves the position confidently to the
  wrong place
- the net confuses it (depth sees through it to things not on the map)
- sits in the critical loop: a bug means the drone flies on a wrong position
- PX4 can reject or lurch on sudden corrections
- needs lots of flight time to tune

## The difference in one line

Plan 4 fixes drift **at the end of each leg**. Plan 3 fixes it **continuously**. Both use the
same ingredients: depth turned on, and expected wall distances from the map.

## If drift is terrible

Neither plan saves broken VIO (big jumps, lost tracking). Fix it at the source first. These
haven't been tried yet:
1. **IMU fusion is off** (`enable_imu_fusion: False` in the cuVSLAM launch). It's what holds the
   estimate steady in fast moves and turns.
2. `EKF2_EV_DELAY` is 70 ms in the params; 58 ms was measured on drone 7.
3. Slower turns with a pause afterwards (`team_flight_test` already does this).
4. Slower flight (less motion blur).
5. What the camera sees: texture and lighting. The rosbag records the images.
6. A loose Pixhawk data cable drops vision data mid-flight (see `skybeta/CLAUDE.md`).

Last resort: fly by walls only ("keep 0.5 m from the right wall until a wall is 0.5 m ahead,
turn left"), with the net from the map and a big margin.

## Order of work

1. Fly the `team_flight_test` profiles (`hover`, `yaw`, `out_back`) and measure real drift.
2. Turn on depth at lower resolution and check VIO stays at ~30 Hz.
3. Build plan 4.
4. Move to plan 3 only if plan 4 isn't enough.

| What the test flights show | Do |
|---|---|
| under ~0.15 m per leg, no jumps | plan 4 |
| bigger but smooth | plan 3, or plan 4 plus centring between side walls |
| sudden jumps or lost tracking | fix VIO first (list above) |

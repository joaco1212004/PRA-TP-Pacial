# custom_map

The cafe the ROSMASTER X3 drives around in, and the occupancy map recorded
inside it. A raw map to plan and follow paths on -- nothing here estimates a
pose.

## Running it

One command, on any machine:

```bash
ros2 launch custom_map cafe.launch.py
```

Gazebo opens in the cafe, `/map` is served from `maps/my_map.yaml`, and RViz
opens with the Map display and `map` as the fixed frame.

It picks the simulator that works where it is started. Gazebo Fortress on macOS
initialises ogre2 on a secondary thread, which Cocoa refuses, so there it has no
GUI and no rendering sensor -- `/scan` never publishes and nothing can be done
with a map. Gazebo Classic opens its window and its CPU raycast lidar needs no
rendering. So macOS gets Classic and everything else gets Fortress. Override
with `backend:=classic` or `backend:=fortress`; `rviz:=false` and `gui:=false`
also work.

On macOS, start it from inside the repository's `classic` pixi environment:

```bash
pixi run -e classic ros2 launch custom_map cafe.launch.py
```

## `map -> odom` is fixed on purpose

It is a static identity transform. The robot's pose in the map is therefore its
raw odometry, and it drifts. That is the point: the drift is visible against
`/ground_truth/odom`, and correcting it is the exercise. `calc_base` hangs off
`odom` for the same reason -- every frame shares one origin, and that origin is
the map's.

This is not what a robot that has to localise would do, and it only works
because nothing here consumes the TF tree for navigation. There is no AMCL, no
SLAM and no Nav2. Do not add one without removing the static transform first:
two publishers of `map -> odom` give `odom` two parents, and tf2 resolves that
by alternating between them.

## The two worlds

| | |
|---|---|
| `worlds/cafe.world` | Gazebo Fortress |
| `worlds/cafe_classic.world` | Gazebo Classic |

They differ only in the header. `cafe.world` declares `<physics type="ignored">`
and six `gz-sim-*` system plugins; Classic rejects the physics type and aborts
the load with *Unable to create physics engine*. Geometry, poses and model URIs
are identical -- keep them that way.

## Collision geometry

`models/custom_cafe` is the stock Gazebo cafe with the upper room made usable:
the doorway at `wall_3` opened, the wall above `wall_4` continued, and the floor
extended to reach both.

The stock model gives collision to the outer shell only, while the mesh draws a
counter, a back room and the upper room's partitions. That matters here because
the two simulators sense different things: Ignition's `gpu_lidar` traces the
rendered scene, Gazebo Classic's `ray` sensor traces collision. `my_map.pgm` was
recorded on the first, so on Classic a third of every scan used to fly straight
through walls the map draws.

37 generated boxes now cover that geometry. They are boxes rather than one mesh
collision because dartsim's ODE mesh wrapper segfaults building the trimesh --
gzserver dies on the first physics step inside
`dart::collision::detail::OdeMesh`. They are 1.5 m tall: enough to stop the
robot and to be seen by the lidar, low enough to stay under the mesh ceiling.

## Building

Only this package needs building; nothing in the rest of the workspace changes.

```bash
colcon build --packages-select custom_map
```

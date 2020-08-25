Path planning benchmark
=======================

To compare each path planning algorithm, we built a simple framework to easily compare each algorithm,
according to certain metrics. 

## Algorithms
- A*
- RRT* (basic version)
- RRT* (with path post processing)

## Test cases

We setup different situations, using the same environment, which all have different challenges for the path planning algorithm.

### Test A
Start: center \
Goal: the corner of the room, in line of sight

### Test B
Start: center \
Goal: the corner behing the small room

Works with A* (Incr=.4, UAV=.2) \
Works with RRT* (But maybe needs a biger UAV size) (Incr=.7)

### Test C
Start: corner \
Goal: the opposite corner, while still being in line of sight

Works with A* (Incr=.4, UAV=.2) \
Works with RRT* (Incr=.7)

### Test D
Start: center \
Goal: Inside the small room, with the door closer to the start

Works with A* (Incr=.4, UAV=.2) \
Works with RRT* (Incr=.7)

### Test E
Start: Furthest corner, behind the small rooms \
Goal: Same as #D

Works with A* (Incr=.4, UAV=.2) \
Works with RRT* (But needs maybe higher UAV size) (Incr=.7)

### Test F
Start: Same as #E \
Goal: On top of the furthest pillar \
This test needs to modify the max possible height

Works with A* (But maybe needs higher UAV size) (Incr=.4, UAV=.2) \
Works with RRT* (Incr=.7)

### Test G
Start: furthest corner from the main door \
Goal: Outside the main map

Works with A* (Incr=.4, UAV=.2) \
Works with RRT* sometimes (Incr=.7)
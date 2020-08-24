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

### Test C
Start: corner
Goal: the opposite corner, while still being in line of sight

### Test D
Start: center
Goal: Inside the small room, with the door closer to the start

### Test E
Start: Furthest corner, behind the small rooms
Goal: Same as #D

### Test F
Start: Same as #E
Goal: On top of the furthest pillar
This test needs to modify the max possible height

### Test G
Start: furthest corner from the main door
Goal: Outside the main map
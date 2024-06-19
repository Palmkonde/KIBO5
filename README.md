# KIBO5 TH_YumeTeam

Everything is in the Yourservice file. The main idea of pathfinding uses the **A*** algorithm, which follows these steps:

### Step by Step:
1.  We need to know our current position, and **Astrobee** provides it for us:

```java
Point nowKIBO = api.getRobotKinematics().getPosition(); // Get the current position
```

2.  For the very first check, we see if we can go straight from our current position to the destination (it's the fastest way because we don't have to reroute around obstacles):

```java
// Check if we can go
boolean canGo = true;
for (Box box : KOZ) {
    // Check if there's any KOZ obstructing the path between the two nodes
    if (isIntersectingBox(start, end, box)) {
        canGo = false;
    }
}
if (canGo) {
    path.add(end);
    return path;
}
``` 

3.  If we can't go straight, we use the A* algorithm. The first node is the current position of **Astrobee**, and the neighboring nodes are positions in 3D coordinates that are ±0.01 in x, y, z.
4.  Once we get the path, we can follow it. But it’s not an optimal path because our neighbors are ±0.01 in x, y, z. This means we can have many unnecessary nodes. Since **Astrobee** allows us to go straight to any node that doesn't pass through a KOZ box, we need to optimize the path a bit:

```java
// List path
List<Point3D> conPath = new ArrayList<>();
Point3D current = end;

while (current != null) {
    conPath.add(current);
    current = cameFrom.get(current);
}
Collections.reverse(conPath);

// Optimize path
Point3D ref = conPath.get(0);
for (int i = 1; i < conPath.size(); i++) {
    for (Box box : KOZ) {
        // We want as few nodes as possible, so
        // if the reference node can go straight to node i,
        // skip other nodes that waste time
        // unless ref can't go to node i
        // which means it can go to node i-1
        // update the reference node to i-1 and repeat until the end of the array
        if (isIntersectingBox(ref, conPath.get(i), box)) {
            ref = conPath.get(i - 1);
            path.add(new Point(ref.x, ref.y, ref.z));
            break;
        }
    }
}
return path;
```
5. That's all. We have the path that we want.

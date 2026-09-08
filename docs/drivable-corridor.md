# Map-independent adjacent-lane drivable corridors

The behavior path planner's common reference-path builder expands the road corridor using the
loaded RouteHandler's **legal, directional left/right routing edges**. No map name, lane ID,
absolute coordinate, fixed lane count or lane width is configured for this behavior.

A legal adjacent lane is no longer rejected in its entirety because its tapered entrance differs
from the route lane by more than 5 degrees or because their centerline separation changes by more
than 1 metre. Both opening and closing tapers keep their original map bounds. Curves and different
centerline sampling do not have to pass an additional whole-lane parallelism test.

## Conditions and safety boundaries

- Each added lane must share the same **oriented** boundary with the current outer lane and have
  a legal lane-change edge for the vehicle traffic rules. Mere proximity or adjacency across a
  forbidden boundary does not qualify. Directional permissions are not assumed symmetric.
- No-drivable lanes, shoulders, malformed/zero-area geometry, opposing boundary orientation and
  incompatible tagged intersection maneuvers are excluded. Traversal stops at a rejected lane;
  it cannot skip that lane to include one farther away. Visited IDs bound the traversal.
- Existing split/merge corridor selection is retained, and its outer edges are also extended
  through legal lateral connections. This supports lanes without a direct common predecessor.
- Lanelet and boundary IDs and geometry are preserved for merging with approved module output.
  The existing bound generator clips the original bounds to the reference path's forward and
  backward extent. A tapered nose remains narrow; it is not replaced with a full-width rectangle.
- Drivable area is available road surface, **not lane-change approval**. Vehicle footprint,
  obstacle margins, predicted collisions, route feasibility, stopping distances and RTC/module
  approval remain separate and unchanged. Merely displaying an adjacent lane does not guarantee
  that a maneuver can pass through its narrow portion.

The stricter completed-parallel geometry checks are still used by existing split **path-shifting**
heuristics; this change does not relax those trajectory-generation checks.

## Lane-change output and successor sections

The lane-change output builder uses the same `expandLaneletCorridor` policy on every
`DrivableLanes` section before overlap clipping and configured bound offsets. This includes
successor sections that previously retained only the selected target lane, even when the
reference-path portion already included its legal neighbors. Existing left, right and middle
lanes are preserved while expanding the outer edges; added intermediate lanes remain available
when the output is combined with the previous module's drivable area.

The common output hook covers both approved lane-change output and terminal lane-change output
while waiting for approval. This does not modify candidate generation, target-lane selection,
lane-departure validation, collision checks, velocity profiles or approval. It does not add a
fixed width across a taper or modify the map.

## Map requirements

Maps must encode shared boundaries consistently and expose the intended vehicle lane-change
permissions in their routing graph. For mixed solid/dashed markings, that includes the correct
directional permission and linestring orientation. Missing topology or prohibited crossings are
not repaired by inflating the drivable area. The map, bridge and traffic-light mapping are not
modified by this implementation.

This is common planner behavior for any correctly encoded loaded map, not a claim that every
external map or driving scenario has been validated. Existing configurable expansion offsets
and module-specific area policies are unchanged.

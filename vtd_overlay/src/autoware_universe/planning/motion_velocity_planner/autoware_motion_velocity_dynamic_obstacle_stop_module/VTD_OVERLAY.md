# VTD dynamic obstacle stop overlay

Upstream: Autoware Universe `0.52.0`, commit
`6e477c645efec33f7909095eea684474e97f5e3d`.

This package matches the installed module version. The local change admits
`ObjectClassification::UNKNOWN` alongside the existing six vehicle classes.
It does not rewrite incoming classifications or affect other modules.
Pedestrian-only and empty classifications remain excluded. All existing speed,
proximity, unavoidable-collision, opposing-heading and stop-duration conditions
are retained. Thus enabling UNKNOWN does not add general same-direction cut-in
handling or static-object stopping to this module.

The upstream classification test expectations are updated for this behavior.
Deployment builds use `BUILD_TESTING=OFF`; no regression suite or driving replay
is run as part of this change. Apply by rebuilding the image and restarting
Autoware, never by altering the running session.

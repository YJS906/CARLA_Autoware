# Selfcar VTD emergency braking

Vendored from autowarefoundation/autoware_universe tag 0.52.0 (control/autoware_vehicle_cmd_gate).
The opt-in immediate_emergency_stop parameter gives an active emergency-stop MRM priority over
pause/disengage stop-hold commands and applies emergency_acceleration immediately. Only longitudinal
comfort filters are bypassed during that emergency; lateral filters and normal driving remain intact.

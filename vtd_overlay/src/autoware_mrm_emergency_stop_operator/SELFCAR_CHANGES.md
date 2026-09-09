# Selfcar VTD emergency braking

Vendored from autowarefoundation/autoware_universe tag 0.52.0 (system/autoware_mrm_emergency_stop_operator).
The opt-in immediate_stop parameter applies target_acceleration on the first emergency command,
without integrating the jerk ramp. Default false preserves upstream behavior.

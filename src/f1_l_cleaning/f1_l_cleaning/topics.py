"""Explicit mappings: shared nodes use absolute pool topic/service names."""

POOL_ENDPOINTS = (
    "wall_observation", "wall_marker", "edge_target", "reset_observation",
    "map_pose", "command_preview", "status", "start", "pause", "cancel",
    "set_edge", "localization_status", "localization_ready",
    "set_initial_pose", "relocalize",
)
REMAPPINGS = [(f"/pool_cleaning/{name}", f"/l_cleaning/{name}")
              for name in POOL_ENDPOINTS]

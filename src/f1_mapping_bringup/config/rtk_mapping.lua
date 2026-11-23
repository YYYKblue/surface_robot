include "pavo2_2d.lua"
options.use_landmarks = true
-- GNSS outages must not hold the laser scan queue.
TRAJECTORY_BUILDER.collate_landmarks = false
-- Produce an optimized, serializable datum even on a short, slow survey.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.
POSE_GRAPH.optimize_every_n_nodes = 10
return options

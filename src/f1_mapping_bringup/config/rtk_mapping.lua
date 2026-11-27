include "pavo2_2d.lua"
-- Position constraints refer directly to the antenna. Its axes parallel base_link.
options.tracking_frame = "gnss_ant1"
-- Produce an optimized, serializable datum even on a short, slow survey.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.
POSE_GRAPH.optimize_every_n_nodes = 10
return options

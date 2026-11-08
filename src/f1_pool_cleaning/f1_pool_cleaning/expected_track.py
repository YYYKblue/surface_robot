"""Boat-centre targets; coverage contact intervals remain a separate contract."""


def cleaning_segments(pool, config, required=None):
    segments = []
    for edge in pool.edges:
        if required and edge.id not in required:
            continue
        contact_start = (required or {}).get(
            edge.id, (edge.clean_start_s_m, edge.clean_end_s_m))[0]
        start = contact_start - config.brush_x_m
        stop = edge.length_m - config.front_stop_m
        if stop < start:
            continue
        points = [[edge.start[i] + edge.tangent[i] * along
                   - edge.left_normal[i] * config.wall_target_m for i in (0, 1)]
                  for along in (start, stop)]
        segments.append(dict(edge_id=edge.id, points=points))
    return segments

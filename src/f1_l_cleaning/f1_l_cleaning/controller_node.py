"""L route adapter; all motion, lease and failsafe logic stays shared."""
from dataclasses import replace

from f1_pool_cleaning.controller_node import PoolController
from f1_pool_cleaning.ros_support import stamp_ns

from .route import build_l_route
from .topics import REMAPPINGS


class LController(PoolController):
    NODE_NAME = "l_controller"
    EDGE_TASK_DEFAULT = True

    def __init__(self, **kwargs):
        self._wall_after_turn_ns = 0
        # Remap even when the executable is run directly. Hardware /f1/*,
        # /map and TF remain shared, with one bridge and localization owner.
        kwargs["cli_args"] = [
            "--ros-args",
            *[arg for old, new in REMAPPINGS for arg in ("-r", f"{old}:={new}")],
            *kwargs.get("cli_args", []),
        ]
        super().__init__(**kwargs)

    def _build_pool(self):
        if not self.edge_task_enabled:
            raise ValueError("L cleaning requires edge_task_enabled=true")
        if self.default_edge_id != "edge_0":
            raise ValueError("L cleaning must start from edge_0 (A -> B)")
        if list(self.boundary_xy) != [0.0]:
            raise ValueError("use route_xy for the open L, not a closed boundary_xy")
        route_xy = self._declare_readonly("route_xy", [0.0])
        return build_l_route(route_xy, self.config)

    def _set_edge(self, request, response):
        if request.edge_id != "edge_0":
            response.success = False
            response.message = "L cleaning starts from edge_0; edge_1 is reached by the single transfer"
            return response
        return super()._set_edge(request, response)

    def _sync_mission_edge(self):
        previous = self.selected_edge
        super()._sync_mission_edge()
        if self.selected_edge is not previous:
            self._wall_after_turn_ns = self.get_clock().now().nanoseconds
            # Do not let a cached first-wall fit turn on the brush immediately
            # after switching edges. Perception also resets on target generation.
            if self.wall is not None:
                self.wall = replace(self.wall, valid=False, reason="waiting for post-turn target wall")

    def _wall(self, message):
        if stamp_ns(message.header.stamp) <= self._wall_after_turn_ns:
            return
        super()._wall(message)


def main(args=None):
    from f1_pool_cleaning.runner import run
    run(LController, args)

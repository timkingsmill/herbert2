# -------------------------------------------------------------------------
# -------------------------------------------------------------------------

import weakref

from abc import ABC
from rclpy.node import Node

# -------------------------------------------------------------------------
# -------------------------------------------------------------------------

class NodeDecorator(ABC):

    # ---------------------------------------------------------------------

    def __init__(self, node: Node):
        self._node_ref = weakref.ref(node)

    # ---------------------------------------------------------------------

    def create_publisher(self, *args, **kwargs):
        return self.node.create_publisher(*args, **kwargs)

    # ---------------------------------------------------------------------

    def create_subscription(self, *args, **kwargs):
        return self.node.create_subscription(*args, **kwargs)

    # ---------------------------------------------------------------------

    def get_logger(self):
        return self.node.get_logger()

    # ---------------------------------------------------------------------

    @property
    def node(self) -> Node:
        return self._node_ref()


# -------------------------------------------------------------------------
# -------------------------------------------------------------------------


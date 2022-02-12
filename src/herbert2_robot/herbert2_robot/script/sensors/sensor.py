import weakref
from abc import ABC
from abc import abstractmethod

from rclpy.node import Node

# --------------------------------------------------------------

class Sensor(ABC):

    # ----------------------------------------------------------

    def __init__(self, node: Node) -> None:
        self._node_ref = weakref.ref(node)

    # ----------------------------------------------------------

    @abstractmethod
    def update(self) -> None:
        pass

    # ----------------------------------------------------------

# --------------------------------------------------------------


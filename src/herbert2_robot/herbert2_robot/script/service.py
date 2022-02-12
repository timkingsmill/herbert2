from abc import ABC, abstractmethod
import weakref
from rclpy.node import Node

# -------------------------------------------------------------------

class Service(ABC):

    # -----------------------------------------------------

    def __init__(self, node: Node) -> None:
        self._node_ref = weakref.ref(node)

    # -----------------------------------------------------

    @abstractmethod
    def _execute_service(self, request, response) -> None:
        pass 

    # -----------------------------------------------------

# -------------------------------------------------------------------
# -------------------------------------------------------------------

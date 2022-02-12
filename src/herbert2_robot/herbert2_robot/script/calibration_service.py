from rclpy.node import Node
from herbert2_odrive_interfaces.srv import CalibrateODriveService

from .service import Service

from .herbert2_robot import ODriveDriver

# ------------------------------------------------------------------------

class CalibrationService(Service):

    def __init__(self, node: Node, controller: ODriveDriver) -> None:
        super().__init__(node)
        self._controller = controller
        self._srv = self._node_ref().create_service(CalibrateODriveService, 'calibrate_odrives', self._service_callback)

    # --------------------------------------------------------------------

    def _service_callback(self, request: CalibrateODriveService.Request, 
                                response: CalibrateODriveService.Response):
        self._execute_service(request, response)
        return response

    # --------------------------------------------------------------------

    def _execute_service(self, request: CalibrateODriveService.Request, 
                               response: CalibrateODriveService.Response) -> None:
        self._node_ref().get_logger().info('Executing the ODrive Calibration Service')
        self._controller.calibrate_axes()
        response._success = True
        response._message = 'Executed OOrive Calibration Service.'

# ------------------------------------------------------------------------
# ------------------------------------------------------------------------


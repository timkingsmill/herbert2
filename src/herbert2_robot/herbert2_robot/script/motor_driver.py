import weakref

from odrive import find_any

from rclpy.logging import get_logger
from rclpy.node import Node


# .................................................................................

class MotorDriver(): 

    # .............................................................................
    # Forward Class Definition
    class _ODrive:
        pass

    # .............................................................................

    _logger = get_logger('MotorDriver')
    _odrives = [_ODrive, _ODrive]

    # .............................................................................
    
    # .............................................................................

    def __init__(self, node: Node) -> None:
        super().__init__()
        """ Motor Driver constructor """
        self._node_ref = weakref.ref(node)
        self.get_logger().info('Initializing Herbert2 Motor Driver') 
        
        for index in [0, 1]:
            drive_name: str = f'odrv{index}'
            # Get the ODrive's serial number
            param = self._node_ref().declare_parameter(drive_name + '.serial_number', 0)
            self._odrives[index] = MotorDriver._ODrive(param.value)


    # .............................................................................

    def __getitem__(self, index):
        """
            Enables indexing: q[0] => q.w
        """
        return self.to_tuple()[index]

    # .............................................................................

    def __len__(self):
        """
            Enables the length function to work: len(q) => 3
        """
        return 3
    
    # .............................................................................

    def __iter__(self):
        """
            Enables iterating: for i in q: print(i)
        """
        for axis in (self.axis0, self.axis1, self.axis2):
            yield axis

    # .............................................................................

    @property
    def odrv0(self) -> _ODrive:
        return self._odrives[0]

    # .............................................................................

    @property
    def odrv1(self) -> _ODrive:
        return self._odrives[1]

    # .............................................................................

    @property
    def axis0(self):
        return self.odrv0.axis0

    # .............................................................................

    @property
    def axis1(self):
        return self.odrv0.axis1

    # .............................................................................

    @property
    def axis2(self):
        return self.odrv1.axis0

    # .............................................................................

    def get_logger(self):
        return self._logger

    # .............................................................................

    def to_dict(self):
        """Returns a dictionary"""
        return {'axis0': self.axis0, 'axis1': self.axis1, 'axis2': self.axis2}

    # .............................................................................

    def to_tuple(self):
        """
            Returns a tuple
        """
        return (self.axis0, self.axis1, self.axis2)

    # .............................................................................
    # .............................................................................

    class _ODrive:

        def __init__(self, serial_number: str):
            self._serial_number = serial_number
            self._logger = get_logger('MotorDriver._ODrive')
            self._odrive = find_any(serial_number = serial_number)
            self._logger.info(f'Found ODrive with serial number {serial_number}')

        # .............................................................................

        @property
        def odrv(self):
            return self._odrive

        # .............................................................................

        @property
        def axis0(self):
            return self._odrive.axis0

        # .............................................................................

        @property
        def axis1(self):
            return self._odrive.axis1

        # .............................................................................

# .................................................................................

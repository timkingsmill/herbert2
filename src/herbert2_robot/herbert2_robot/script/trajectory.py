# -----------------------------------------------------------------------------

import math
from typing import List
import numpy as np
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

class TrajectoryStep():

    def __init__(self):
        self.position = 0.0       # Position
        self.velocity = 0.0       # Velocity
        self.acceleration = 0.0   # Acceleration

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

class Trajectory():
    """
        Generates an acceleration-limited (AL) trajectory.
    """
    # ......................................................................................

    def __init__(self):
        super().__init__()

        self._initial_position: float = 0.0
        self._initial_velocity: float = 0.0

        # Maximum Acceleration (signed)
        self._max_acceleration: float = 0.0  
        # Maximum Deceleration (signed)
        self._max_deceleration: float = 0.0  
        # Maximum Velocity (signed)
        self._max_velocity: float = 0.0 

        # Time to accelerate to max_velocity
        self._acceleration_time: float = 0.0
        self._deceleration_time: float = 0.0
        self._coasting_time: float = 0.0
        self._total_time: float = 0.0

        self._acceleration_done: float = 0.0

    # ......................................................................................

    def plan_trajectory(self, goal: float, 
                              initial_position: float, 
                              initial_velocity: float, 
                              Vmax: float, Amax: float, Dmax: float) -> bool:
        """
            Plan a trajectory from start to goal with a given starting velocity.
        """
        self._goal: float = goal
        self._initial_position: float = initial_position
        self._initial_velocity: float = initial_velocity

        # Distance to travel
        distance: float = self._goal - self._initial_position  

        # Minimum stopping distance
        stopping_distance: float = (self._initial_velocity * self._initial_velocity) / (2.0 * Dmax) 

        # Minimum stopping displacement
        stopping_displacement: float = np.sign(self._initial_velocity) * stopping_distance

        # Sign of coast velocity (if any)
        # The sign of the velocity during the cruising phase.
        sign: float = np.sign(distance - stopping_displacement)

        # To ensure that the trajectory is time-optimal, we set the
        # acceleration, deceleration, and cruising velocity values at the
        # maximum values
        #
        # Maximum Acceleration (signed)
        self._max_acceleration: float = sign * Amax  
        # Maximum Deceleration (signed)
        self._max_deceleration: float = -sign * Dmax  
        # Maximum Velocity (signed) Coasting speed.
        self._max_velocity: float = sign * Vmax;  

        # If we start with a speed faster than coasting speed, 
        # we need to decelerate instead of accelerate.
        if ((sign * self._initial_velocity) > (sign * self._max_velocity)): 
            print('Applying the handbreak!')
            self._max_acceleration = -sign * Amax
            #self._max_deceleration = sign * Amax

        # Calculate the time to accelerate/decelerate to/from max_velocity (cruise speed)
        self._acceleration_time = (self._max_velocity - self._initial_velocity) / self._max_acceleration
        self._deceleration_time = (-self._max_velocity) / self._max_deceleration

        # Integral of velocity ramps over the full accel and decel times to get
        # minimum displacement required to reach cuising speed.
        min_displacement: float = (self._acceleration_time * (self._max_velocity + self._initial_velocity) / 2.0) \
                                + (self._deceleration_time * (self._max_velocity) / 2.0)   

        # Calculate the time to coast to the minimum displacement
        # Are we displacing enough to reach cruising speed?
        if ((sign * distance) < (sign * min_displacement)) :
            # Short move calculates a triangle profile. No coasting time
            print("Short Move:")
            # Calculate the maximum velocity to reach the minimum displacement
            self._max_velocity: float = sign * math.sqrt((self._max_deceleration * self._initial_velocity**2) + (2.0 * self._max_acceleration * self._max_deceleration * distance) / (self._max_deceleration - self._max_acceleration)) 
            print("Max Velocity: ", self._max_velocity)
            self._acceleration_time = max(0.0, (self._max_velocity - self._initial_velocity) / self._max_acceleration)
            self._deceleration_time = max(0.0, (-self._max_velocity) / self._max_deceleration)
            self._coasting_time = 0.0
        else:
            print("Long Move:")
            self._coasting_time = (distance - min_displacement) / self._max_velocity 

        self._total_time = self._acceleration_time + self._coasting_time + self._deceleration_time
        
        # We only know acceleration (Ar and Dr), so we integrate to create
        # the velocity and position curves we should do this once at the 
        # start of the move, not for each call of time thru the evaluate method.
        
        # Position at the end of the acceleration phase
        self._acceleration_done = self._initial_position                                   \
                                + (self._initial_velocity * self._acceleration_time)       \
                                + (0.5 * self._max_acceleration * self._acceleration_time * self._acceleration_time)    
        return True

    # ......................................................................................
    
    def evaluate(self, time: float) -> TrajectoryStep:
        """
            Step Function for the trajectory.
            Outputs a jerk-limited (JL) trajectory.
        """
        step: TrajectoryStep = TrajectoryStep()
        
        if (time < 0.0):
            #print("Initial Condition")
            step.position = self._initial_position
            step.velocity = self._initial_velocity
            step.acceleration = 0.0

        elif (time < self._acceleration_time): 
            # Acceleration stage
            #print("Accelerating")   
            step.position = self._initial_position + self._initial_velocity * time + 0.5 * self._max_acceleration * time * time
            step.velocity = self._initial_velocity + self._max_acceleration * time
            step.acceleration = self._max_acceleration

        elif (time < (self._acceleration_time + self._coasting_time)):
            # Coasting stage
            #print("Coasting")   
            step.position = self._acceleration_done + self._max_velocity * (time - self._acceleration_time)
            step.velocity = self._max_velocity
            step.acceleration = 0.0

        elif (time < (self._acceleration_time + self._coasting_time + self._deceleration_time)):
            # Deceleration stage 
            #print("Decelerating")   
            remaining_time = time - self._total_time
            step.position = self._goal + (0.5 * self._max_deceleration * remaining_time * remaining_time)
            step.velocity = self._max_deceleration * remaining_time
            step.acceleration = self._max_deceleration

        elif (time >= self._total_time):
            #print("Final Condition")
            step.position = self._goal
            step.velocity = 0.0
            step.acceleration = 0.0

        else:
            raise ValueError("time = {} is outside of considered range".format(time))
    
        return step
    
    # ......................................................................................

    @property
    def total_time(self) -> float:
        return self._total_time

    # ......................................................................................

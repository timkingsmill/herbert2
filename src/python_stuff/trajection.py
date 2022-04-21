# -----------------------------------------------------------------------------

import math
from re import S
from typing import List
import numpy as np
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
"""
# A sign function where input 0 has positive sign (not 0)
def sign_hard(value: float) -> float:
    if (value >= 0.0):
        return 1.0
    else:
        return -1.0
"""
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

class TrajectoryStep():

    def __init__(self):
        self.position = 0.0     # Position
        self.velocity = 0.0    # Velocity
        self.acceleration = 0.0   # Acceleration


# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

class Trajectory():
    """
        Generates an acceleration-limited (AL) trajectory.
    """


    # ............................................................

    def __init__(self):
        super().__init__()
        
        self._debug_mode = False

        self._initial_position: float = 0.0
        self._initial_velocity: float = 0.0
        self._goal:             float = 0.0

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

        self._trajectory_complete: bool = False

        # ............................................................

    def plan_trajectory(self, goal: float, 
                              initial_position: float, 
                              initial_velocity: float, 
                              Vmax: float, Amax: float, Dmax: float) -> bool:
        """
        Plan a trajectory from start to goal with a given starting velocity.
        """
        self._trajectory_complete: bool = False

        if (self.debug_mode):
            print("DEBUG MODE")

        self._goal: float = goal
        self._initial_position: float = initial_position
        self._initial_velocity: float = initial_velocity

        if (self.debug_mode):
            print("Initial Position: {}".format(self._initial_position))
            print("Initial Velocity: {}".format(self._initial_velocity))
            print("Goal: {}".format(self._goal))

        # Distance to travel
        distance: float = self._goal - self._initial_position  
        if (self.debug_mode):
            print("Total Travel Distance: {}".format(distance))
        
        # Minimum stopping distance
        stopping_distance: float = (self._initial_velocity * self._initial_velocity) / (2.0 * Dmax) 
        if (self.debug_mode):
            print("Stopping Distance: {}".format(stopping_distance))

        # Minimum stopping displacement
        stopping_displacement: float = np.sign(self._initial_velocity) * stopping_distance
        if (self.debug_mode):
            print("Stopping Displacement: {}".format(stopping_displacement))

        # Sign of coast velocity (if any)
        # The sign of the velocity during the cruising phase.
        sign: float = np.sign(distance - stopping_displacement)
        if (self.debug_mode):
            print("Sign: {}".format(sign))

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
            #self._max_deceleration = sign * Dmax
        else:
            print('Applying the accelerator!')    
    
        if (self.debug_mode):
            print("Max Acceleration: {}".format(self._max_acceleration))
            print("Max Deceleration: {}".format(self._max_deceleration))
            print("Max Velocity:     {}".format(self._max_velocity))

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
            #print("Short Move:")
            
            # Calculate the maximum velocity to reach the minimum displacement
            self._max_velocity: float = sign * math.sqrt((self._max_deceleration * self._initial_velocity**2) + (2.0 * self._max_acceleration * self._max_deceleration * distance) / (self._max_deceleration - self._max_acceleration)) 
            #print("Max Velocity: ", self._max_velocity)
            
            self._acceleration_time = max(0.0, (self._max_velocity - self._initial_velocity) / self._max_acceleration)
            self._deceleration_time = max(0.0, (-self._max_velocity) / self._max_deceleration)
            self._coasting_time = 0.0
        else:
            #print("Long Move:")
            self._coasting_time = (distance - min_displacement) / self._max_velocity 

        self._total_time = self._acceleration_time + self._coasting_time + self._deceleration_time

        # We only know acceleration (Ar and Dr), so we integrate to create
        # the velocity and position curves we should do this once at the 
        # start of the move, not for each call of time thru the evaluate method.
        
        # Position at the end of the acceleration phase
        self._acceleration_done = self._initial_position                                   \
                                + (self._initial_velocity * self._acceleration_time)       \
                                + (0.5 * self._max_acceleration * self._acceleration_time * self._acceleration_time)      
        # Fill in the rest of the values used at evaluation-time
        #Xi_ = Xi;
        #Xf_ = Xf;
        #Vi_ = Vi;
        #yAccel_ = Xi + Vi*Ta_ + 0.5f*Ar_*SQ(Ta_); // pos at end of accel phase

        #print(f'Distance: {distance:.3f}   Stopping Distance: {stopping_distance:.3f}   Stopping Displacement: {stopping_displacement:.3f}   Max Acceleration: {self._max_acceleration:.3f}   Max Deceleration: {self._max_deceleration:.3f}   Max Velocity: {self._max_velocity:.3f}')


        return True

    # ............................................................

    def evaluate(self, time: float) -> TrajectoryStep:
        #
        #Step Function for the trajectory.
        #Outputs a jerk-limited (JL) trajectory.
        #
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
            self._trajectory_complete = True
        else:
            raise ValueError("time = {} is outside of considered range".format(time))
        return step

    # ............................................................

    def total_time(self) -> float:
        return self._total_time

    # ............................................................

    @property
    def debug_mode(self) -> bool:
        """
        Get debug mode on or off.
        """
        return self._debug_mode

   # ............................................................

    @debug_mode.setter
    def debug_mode(self, debug: bool) -> bool:
        """
        Set debug mode on or off.
        """
        self._debug_mode = debug
        return True

    # ............................................................

    @property
    def is_trajectory_complete(self) -> bool:
        return self._trajectory_complete

    # ............................................................

    def __str__(self) -> str:   
        text = """
        Trajectory:
            Initial Position:  {self._initial_position:.3f}  
            Target Position:   {self._goal:.3f} 
            Initial Velocity:  {self._initial_velocity:.3f}  
        Time:     
            Acceleration Time: {self._acceleration_time:.3f}
            Coasting Time:     {self._coasting_time:.3f}
            Deceleration Time: {self._deceleration_time:.3f}
            Total Time:        {self._total_time:.3f}
        """
        return text.format(self=self)

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

def run_trajectory_test(goal: float, 
                        initial_position: float, 
                        initial_velocity: float, 
                        Vmax: float, Amax: float, Dmax: float) -> None:
    print("Running Trajectory Test!")

    step_time_: float = 0.0
    overall_time_: float = 0.0

    #time_interval: float = 0.000125
    time_interval: float = 1.0
    ##time_interval: float = 0.01

    replan_counter_: int = 0

    time_array: List[float] = [] 

    position_array: List[float] = []
    velocity_array: List[float] = []
    acceleration_array: List[float] = []

    trajectory = Trajectory()

    _goal = goal
    _initial_position = initial_position 
    _initial_velocity = initial_velocity
    _max_velocity = Vmax
    _max_acceleration = Amax
    _max_deceleration = Dmax

    step_: TrajectoryStep = None

    if (trajectory.plan_trajectory(_goal, _initial_position, _initial_velocity, _max_velocity, _max_acceleration, _max_deceleration)):
        print("Planned Trajectory")
        print(trajectory)


    while (True):
        step_ = trajectory.evaluate(step_time_)
        print('Position: {0:.3f}   Velocity: {1:.3f}   Acceleration: {2:.3f}'.format(step_.position, step_.velocity, step_.acceleration))   

        # Populate the arrays for plotting.
        time_array.append(overall_time_)
        position_array.append(step_.position)
        velocity_array.append(step_.velocity * 10.0)
        acceleration_array.append(step_.acceleration * 100.0)

        # Replan the trajectory if the current step is halfway to the goal.
        if (replan_counter_ == 0) and (step_time_ >= trajectory.total_time() / 2.0):
            replan_counter_ += 1
            #print("Replaning Trajectory")
            trajectory.debug_mode = True

            step_time_ = 0.0
            _initial_position = step_.position
            _initial_velocity = step_.velocity
            _goal = step_.position + 100.0

            if (trajectory.plan_trajectory(_goal, _initial_position, _initial_velocity, _max_velocity, _max_acceleration, _max_deceleration)):
                print("Replanned Trajectory")
                print(trajectory)

        # Increment the timing variables.
        step_time_ += time_interval
        overall_time_ += time_interval

        # Finish the loop if we've reached the end of the trajectory.
        if (trajectory.is_trajectory_complete):
            break


    print("Trajectory Calculator Test Complete!")
    print(time_array) 

    plt.plot(time_array, position_array, label='Position')
    plt.plot(time_array, velocity_array, label='Velocity (x 10)')
    plt.plot(time_array, acceleration_array, label='Acceleration (x 100)')

    plt.legend()
    plt.show()



# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

def main(args=None):
    print("Trajectory Calculator!")

    run_trajectory_test(goal = 1000.0, initial_position = 0.0, initial_velocity = 0.0, Vmax = 20.0, Amax = 1.0, Dmax = 1.0)


    #run_trajectory_test(goal = 1000.0, initial_position = 500.0, initial_velocity = -10.0, Vmax = 20.0, Amax = 1.0, Dmax = 1.0)


    return 0

# -----------------------------------------------------------------------------

if __name__ == '__main__':
    main()

# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
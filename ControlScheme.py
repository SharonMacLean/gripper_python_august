import time
import json


# Object which allows the user to return required control actions for a variety of different controller types
class PythonController:
    def __init__(self):
        self.proportional_gain = None
        self.integral_gain = None
        self.derivative_gain = None
        self.setpoint = None  # Force Setpoint
        self.t_current = 0  # Time of current control update [time since epoch]
        self.t_minus_1 = 0  # Time of previous control update
        self.t_minus_2 = 0  # Time of 2 control updates ago
        self.error_current = 0  # Current control error
        self.error_t_minus_1 = 0  # Error 1 time step ago
        self.error_t_minus_2 = 0  # Error 2 time steps ago
        self.control_current = 0  # Current Control Input
        self.control_t_minus_1 = 0  # Control input 1 time step ago
        # Add any other required gain values/required variables to this list of attributes

    # Update control gains for "Senior Design" setup for a given control type (P or PI)
    def update_finger_gains(self, fingertype):
        if fingertype == "Rigid":
            self.proportional_gain = 0.000869
            self.integral_gain = 0.0087
            self.derivative_gain = 0
        elif fingertype == "Thin Convex":
            self.proportional_gain = 0.0017
            self.integral_gain = 0.0083
            self.derivative_gain = 0
        elif fingertype == "Thin Concave":
            self.proportional_gain = 0.0016
            self.integral_gain = 0.0080
            self.derivative_gain = 0
        elif fingertype == "Thick Concave":
            self.proportional_gain = 0.0018
            self.integral_gain = 0.0089
            self.derivative_gain = 0

    # Update the controller gain values
    def update_gains(self, kp=0, ki=0, kd=0):
        self.proportional_gain = kp
        self.integral_gain = ki
        self.derivative_gain = kd

    # Update the setpoint for controller
    def update_setpoint(self, setpoint):
        self.setpoint = setpoint

    # Implement Proportional control law
    def p_control(self, currentvalue):
        print("update p")

        self.control_current = (currentvalue - self.setpoint) * self.proportional_gain
        return self.control_current

    # Implement Proportional + Integral control law
    def pi_control(self, currentvalue):
        print("update pi")

        # Update time steps
        self.t_minus_1 = self.t_current
        self.t_current = time.perf_counter()  # Returns time elapsed since epoch [in seconds]

        # Update previous error values
        self.error_t_minus_1 = self.error_current
        self.error_current = currentvalue - self.setpoint

        # Update previous control input
        self.control_t_minus_1 = self.control_current

        # Calculate control output
        self.control_current = self.control_t_minus_1 + \
                               self.proportional_gain * (self.error_current - self.error_t_minus_1) + \
                               self.integral_gain * self.error_current * (self.t_minus_1 - self.t_current)

        return self.control_current

    # Implement Proportional + Derivative control law
    def pd_control(self, currentvalue):
        print("update pd")

        # Update time steps
        self.t_minus_1 = self.t_current
        self.t_current = time.perf_counter()

        # Update previous error values
        self.error_t_minus_1 = self.error_current
        self.error_current = currentvalue - self.setpoint

        # Calculate control output
        self.control_current = self.proportional_gain * self.error_current + \
                               self.derivative_gain * (self.error_current - self.error_t_minus_1) / (
                                           self.t_current - self.t_minus_1)

        return self.control_current

    # Implement Proportional + Integral + Derivative control law
    def pid_control(self, currentvalue):
        print("update pid")

        # Update time steps
        self.t_minus_2 = self.t_minus_1
        self.t_minus_1 = self.t_current
        self.t_current = time.perf_counter()  # Returns time elapsed since epoch [in seconds]

        # Update previous error values
        self.error_t_minus_2 = self.error_t_minus_1
        self.error_t_minus_1 = self.error_current
        self.error_current = currentvalue - self.setpoint

        # Update previous control input
        self.control_t_minus_1 = self.control_current

        # Calculate control output
        self.control_current = self.control_t_minus_1 + \
                               self.derivative_gain * ((self.error_current - self.error_t_minus_1) / (
                    self.t_current - self.t_minus_1) -
                                                       (self.error_t_minus_1 - self.error_t_minus_2) / (
                                                                   self.t_minus_1 - self.t_minus_2)) + \
                               self.proportional_gain * (self.error_current - self.error_t_minus_1) + \
                               self.integral_gain * self.error_current * (self.t_current - self.t_minus_1)

        return self.control_current

        # Reset values before entering control loop again (To avoid huge integral gain from previous times)

    # Reset previous times and errors upon starting a new control loop
    def new_control_loop(self):
        self.error_t_minus_2 = 0
        self.error_t_minus_1 = 0
        self.t_minus_2 = time.perf_counter()
        self.t_minus_1 = self.t_minus_2

        # Save all current controller parameters in a text file to be retrieved later

    # Save all controller attributes in a text file
    def save_controller_attributes_json(self, filename='Controller_Attributes_1.txt'):
        # Check if given filename also includes a '.txt' ending, if not add it
        if '.txt' not in filename:
            filename = filename + '.txt'

        # Get all attributes from instance, store in dictionary and save
        data_to_save = vars(self)
        for key in list(data_to_save.keys()):

            # Not a very general way of removing invalid json objects
            # json cannot store bytes type data, so dictionaries are not stored in json file
            # Since dictionaries of function/parameter bytes cannot be stored, lists won't be stored either
            curval = str(type(data_to_save[key]))
            if ('str' not in curval) and ('int' not in curval) and ('float' not in curval) and \
                    ('NoneType' not in curval):
                del data_to_save[key]

        with open(filename, 'w') as json_file:
            json.dump(obj=data_to_save, fp=json_file, skipkeys=True, indent=0)

        # This method will load all attributes saved in a json file and update instance attributes to thier values

    # Load all controller attributes from a text file
    def load_controller_attributes_json(self, filename='Controller_Attributes_1.txt'):

        # Check if given filename also includes a '.txt' ending, if not add it
        if '.txt' not in filename:
            filename = filename + '.txt'

        try:
            with open(file=filename) as f:
                saved_data_json = json.load(f)
        except FileNotFoundError:
            print("File not accessible")

        data_to_update = vars(self)

        for key in saved_data_json.keys():
            data_to_update[key] = saved_data_json[key]

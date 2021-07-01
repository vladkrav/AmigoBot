import logging
import time
import threading
import multiprocessing
import sys
from datetime import datetime
import re
import json
import traceback
import imp

import rospy
from std_srvs.srv import Empty
import cv2

# from user_functions import GUIFunctions, HALFunctions
from console2 import start_console, close_console
from hal import HAL
from gui import GUI
from shared.value import SharedValue

# The brain process class
class BrainProcess(multiprocessing.Process):
    def __init__(self, code, exit_signal):
        super(BrainProcess, self).__init__()

        # Initialize exit signal
        self.exit_signal = exit_signal

        # Function definitions for users to use
        self.hal = HAL()
        self.gui = GUI()

        # Time variables
        self.time_cycle = SharedValue('brain_time_cycle')
        self.ideal_cycle = SharedValue('brain_ideal_cycle')
        self.iteration_counter = 0

        # Get the sequential and iterative code
        # Something wrong over here! The code is reversing
        # Found a solution but could not find the reason for this
        self.teop = False
        self.key = None

        if(code[0] == "" and code[1] == "" and (code[2] == True or code[2] == False)):
            self.teop = code[2]
        elif(code[0] == "" and code[1] == "" and (code[2] != True or code[2] != False) and code[2] != ""):
            self.key = code[2]
        else:
            self.sequential_code = code[1]
            self.iterative_code = code[0]

        # initialize Teleoperation variables
        self.speedV = 0.3
        self.speedW = 0.5
        self.stop = 0
        self.flag = 0
        self.pattern_V = 'HAL.motors.sendV'
        self.pattern_W = 'HAL.motors.sendW'

    def KeyEvent(self, key):

        if(key == "w"):
            self.hal.motors.sendV(self.speedV)
        elif(key == "s"):
            self.hal.motors.sendV(self.stop)
            self.hal.motors.sendW(self.stop)
        elif(key == "d"):
            self.hal.motors.sendW(-self.speedW)
        elif(key == "a"):
            self.hal.motors.sendW(self.speedW)
        elif(key == "q"):
            self.speedV = self.speedV + 0.1 * self.speedV
            if(self.speedV >= 0.75):
                self.speedV = 0.75
        elif(key == "z"):
            self.speedV = self.speedV - 0.1 * self.speedV
            if(self.speedV <= 0):
                self.speedV = 0
        elif(key == "e"):
            self.speedW = self.speedW + 0.1 * self.speedW
            if(self.speedW >= 0.75):
                self.speedW = 0.75
        elif(key == "c"):
            self.speedW = self.speedW - 0.1 * self.speedW
            if(self.speedW <= 0):
                self.speedW = 0
        else:
            pass
        self.gui.update_gui()

    # Function to run to start the process
    def run(self):
        # Two threads for running and measuring
        self.measure_thread = threading.Thread(target=self.measure_frequency)
        self.thread = threading.Thread(target=self.process_code)

        self.measure_thread.start()
        self.thread.start()

        print("Brain Process Started!")

        self.exit_signal.wait()

    # The process function
    def process_code(self):
        # Redirect information to console
        start_console()

        # Reference Environment for the exec() function
        iterative_code, sequential_code = self.iterative_code, self.sequential_code
        
        # print(sequential_code)
        # print(iterative_code)
        
        # Whatever the code is, first step is to just stop!
        self.hal.sendV(0)
        self.hal.sendW(0)


        # The Python exec function
        # Run the sequential part
        gui_module, hal_module = self.generate_modules()
        if sequential_code != "":
            reference_environment = {"GUI": gui_module, "HAL": hal_module}
            exec(sequential_code, reference_environment)

        # Run the iterative part inside template
        # and keep the check for flag
        while not self.exit_signal.is_set():
            start_time = datetime.now()
            if(self.teop == True):
                self.KeyEvent(self.key)
                iterative_code = re.sub(self.pattern_V, '#', iterative_code)
                iterative_code = re.sub(self.pattern_W, '#', iterative_code)
            
            # Execute the iterative portion
            if iterative_code != "":
                exec(iterative_code, reference_environment)

            # Template specifics to run!
            finish_time = datetime.now()
            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            
            # Keep updating the iteration counter
            if(iterative_code == ""):
                self.iteration_counter = 0
            else:
                self.iteration_counter = self.iteration_counter + 1
        
            # The code should be run for atleast the target time step
            # If it's less put to sleep
            # If it's more no problem as such, but we can change it!
            time_cycle = self.time_cycle.get()

            if(ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)

        close_console()
        print("Current Thread Joined!")


    # Function to generate the modules for use in ACE Editor
    def generate_modules(self):
        # Define HAL module
        hal_module = imp.new_module("HAL")
        hal_module.HAL = imp.new_module("HAL")
        hal_module.HAL.motors = imp.new_module("motors")

        # Add HAL functions
        hal_module.HAL.getPose3d = self.hal.pose3d.getPose3d
        hal_module.HAL.motors.sendV = self.hal.motors.sendV
        hal_module.HAL.motors.sendW = self.hal.motors.sendW
        hal_module.HAL.getLaserData = self.hal.laser.getLaserData
        hal_module.HAL.getSonarData_0 = self.hal.sonar_0.getSonarData
        hal_module.HAL.getSonarData_1 = self.hal.sonar_1.getSonarData
        hal_module.HAL.getSonarData_2 = self.hal.sonar_2.getSonarData
        hal_module.HAL.getSonarData_3 = self.hal.sonar_3.getSonarData
        hal_module.HAL.getSonarData_4 = self.hal.sonar_4.getSonarData
        hal_module.HAL.getSonarData_5 = self.hal.sonar_5.getSonarData
        hal_module.HAL.getSonarData_6 = self.hal.sonar_6.getSonarData
        hal_module.HAL.getSonarData_7 = self.hal.sonar_7.getSonarData

        # Define GUI module
        gui_module = imp.new_module("GUI")
        gui_module.GUI = imp.new_module("GUI")

        # Add GUI functions
        gui_module.GUI.update = self.gui.update_gui
        gui_module.GUI.showParticles = self.gui.showParticles
        gui_module.GUI.showEstimatedPose = self.gui.showEstimatedPose

        # Adding modules to system
        # Protip: The names should be different from
        # other modules, otherwise some errors
        sys.modules["HAL"] = hal_module
        sys.modules["GUI"] = gui_module

        return gui_module, hal_module
            
    # Function to measure the frequency of iterations
    def measure_frequency(self):
        previous_time = datetime.now()
        # An infinite loop
        while not self.exit_signal.is_set():
            # Sleep for 2 seconds
            time.sleep(2)
            
            # Measure the current time and subtract from the previous time to get real time interval
            current_time = datetime.now()
            dt = current_time - previous_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            previous_time = current_time
            
            # Get the time period
            try:
            	# Division by zero
            	self.ideal_cycle.add(ms / self.iteration_counter)
            except:
            	self.ideal_cycle.add(0)
            
            # Reset the counter
            self.iteration_counter = 0


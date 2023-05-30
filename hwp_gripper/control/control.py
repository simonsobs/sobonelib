# Built-in python modules
import time as tm
import sys
import os

class Control:
    """
    The Control object is a class to package gripper controller operations

    Args:
    JXC (src.JXC): JXC object

    Attributes:
    log (src.Logging): logging object
    """
    def __init__(self, JXC):
        if JXC is None:
            raise Exception(
                'Control Error: Control() constructor requires a '
                'controller object')
        self._JXC = JXC

        # Timout and timestep
        self._tout = 25.0  # sec
        self._tstep = 0.1  # sec

        # Dictionary of pins to write for each step number
        # Binary input = step number + 1
        self.step_inputs = {
            "01": [self._JXC.IN0],
            "02": [self._JXC.IN1],
            "03": [self._JXC.IN0, self._JXC.IN1],
            "04": [self._JXC.IN2],
            "05": [self._JXC.IN0, self._JXC.IN2],
            "06": [self._JXC.IN1, self._JXC.IN2],
            "07": [self._JXC.IN0, self._JXC.IN1, self._JXC.IN2],
            "08": [self._JXC.IN3],
            "09": [self._JXC.IN0, self._JXC.IN3],
            "10": [self._JXC.IN1, self._JXC.IN3],
            "11": [self._JXC.IN0, self._JXC.IN1, self._JXC.IN3],
            "12": [self._JXC.IN2, self._JXC.IN3],
            "13": [self._JXC.IN0, self._JXC.IN2, self._JXC.IN3],
            "14": [self._JXC.IN1, self._JXC.IN2, self._JXC.IN3],
            "15": [self._JXC.IN0, self._JXC.IN1, self._JXC.IN2, self._JXC.IN3],
            "16": [self._JXC.IN4],
            "17": [self._JXC.IN0, self._JXC.IN4],
            "18": [self._JXC.IN1, self._JXC.IN4],
            "19": [self._JXC.IN0, self._JXC.IN1, self._JXC.IN4],
            "20": [self._JXC.IN2, self._JXC.IN4],
            "21": [self._JXC.IN0, self._JXC.IN2, self._JXC.IN4],
            "22": [self._JXC.IN1, self._JXC.IN2, self._JXC.IN4],
            "23": [self._JXC.IN0, self._JXC.IN1, self._JXC.IN2, self._JXC.IN4],
            "24": [self._JXC.IN3, self._JXC.IN4],
            "25": [self._JXC.IN0, self._JXC.IN3, self._JXC.IN4],
            "26": [self._JXC.IN1, self._JXC.IN3, self._JXC.IN4],
            "27": [self._JXC.IN0, self._JXC.IN1, self._JXC.IN3, self._JXC.IN4],
            "28": [self._JXC.IN2, self._JXC.IN3, self._JXC.IN4],
            "29": [self._JXC.IN0, self._JXC.IN2, self._JXC.IN3, self._JXC.IN4],
            "30": [self._JXC.IN1, self._JXC.IN2, self._JXC.IN3, self._JXC.IN4]}

        # Dictionary of pins to write for each step number
        # Binary input = step number + 1
        self.step_outputs = {
            "01": [self._JXC.OUT0],
            "02": [self._JXC.OUT1],
            "03": [self._JXC.OUT0, self._JXC.OUT1],
            "04": [self._JXC.OUT2],
            "05": [self._JXC.OUT0, self._JXC.OUT2],
            "06": [self._JXC.OUT1, self._JXC.OUT2],
            "07": [self._JXC.OUT0, self._JXC.OUT1, self._JXC.OUT2],
            "08": [self._JXC.OUT3],
            "09": [self._JXC.OUT0, self._JXC.OUT3],
            "10": [self._JXC.OUT1, self._JXC.OUT3],
            "11": [self._JXC.OUT0, self._JXC.OUT1, self._JXC.OUT3],
            "12": [self._JXC.OUT2, self._JXC.OUT3],
            "13": [self._JXC.OUT0, self._JXC.OUT2, self._JXC.OUT3],
            "14": [self._JXC.OUT1, self._JXC.OUT2, self._JXC.OUT3],
            "15": [self._JXC.OUT0, self._JXC.OUT1, self._JXC.OUT2,
                   self._JXC.OUT3],
            "16": [self._JXC.OUT4],
            "17": [self._JXC.OUT0, self._JXC.OUT4],
            "18": [self._JXC.OUT1, self._JXC.OUT4],
            "19": [self._JXC.OUT0, self._JXC.OUT1, self._JXC.OUT4],
            "20": [self._JXC.OUT2, self._JXC.OUT4],
            "21": [self._JXC.OUT0, self._JXC.OUT2, self._JXC.OUT4],
            "22": [self._JXC.OUT1, self._JXC.OUT2, self._JXC.OUT4],
            "23": [self._JXC.OUT0, self._JXC.OUT1, self._JXC.OUT2,
                   self._JXC.OUT4],
            "24": [self._JXC.OUT3, self._JXC.OUT4],
            "25": [self._JXC.OUT0, self._JXC.OUT3, self._JXC.OUT4],
            "26": [self._JXC.OUT1, self._JXC.OUT3, self._JXC.OUT4],
            "27": [self._JXC.OUT0, self._JXC.OUT1, self._JXC.OUT3,
                   self._JXC.OUT4],
            "28": [self._JXC.OUT2, self._JXC.OUT3, self._JXC.OUT4],
            "29": [self._JXC.OUT0, self._JXC.OUT2, self._JXC.OUT3,
                   self._JXC.OUT4],
            "30": [self._JXC.OUT1, self._JXC.OUT2, self._JXC.OUT3,
                   self._JXC.OUT4]}

        # Dictionary of Alarm OUTputs
        # Output 0, 1, 2, 3
        self.alarm_group = {}
        self.alarm_group["B"] = '0100'
        self.alarm_group["C"] = '0010'
        self.alarm_group["D"] = '0001'
        self.alarm_group["E"] = '0000'

    # ***** Public Methods *****
    def ON(self, log=[]):
        """ Turn the controller on """
        # Turn SVON on
        if not self._JXC.read(self._JXC.SVON):
            self._JXC.set_on(self._JXC.SVON)
        self._sleep()
        if not self._JXC.read(self._JXC.SVON):
            log.append("Failed to turn SVON on")
            return False, log
        else:
            log.append("SVON turned on in Control.ON()")

        # Turn off the brakes
        if (not self._JXC.read(self._JXC.BRAKE1) or
           not self._JXC.read(self._JXC.BRAKE2) or
           not self._JXC.read(self._JXC.BRAKE3)):
            _, log = self.BRAKE(False, log=log)
        self._sleep()
        if (not self._JXC.read(self._JXC.BRAKE1) or
           not self._JXC.read(self._JXC.BRAKE2) or
           not self._JXC.read(self._JXC.BRAKE3)):
            log.append("Failed to disengage brakes in Control.ON()")
            return False, log
        else:
            log.append("Disengaged brakes in Control.ON()")

        return True, log

    def OFF(self, log=[]):
        """ Turn the controller off """
        # Turn on the brakes
        if (self._JXC.read(self._JXC.BRAKE1) or
           self._JXC.read(self._JXC.BRAKE2) or
           self._JXC.read(self._JXC.BRAKE3)):
            _, log = self.BRAKE(True, log=log)
        self._sleep()
        if (self._JXC.read(self._JXC.BRAKE1) or
           self._JXC.read(self._JXC.BRAKE2) or
           self._JXC.read(self._JXC.BRAKE3)):
            log.append("Failed to engage brakes in Control.OFF()")
            return False, log
        else:
            log.append("Brakes engaged in Control.OFF()")

        # Turn SVON off
        if self._JXC.read(self._JXC.SVON):
            self._JXC.set_off(self._JXC.SVON)
        self._sleep()
        if self._JXC.read(self._JXC.SVON):
            log.append("Failed turn SVON off in Control.OFF()")
            return False, log
        else:
            log.append("SVON turned off in Control.OFF()")

        return True, log

    def HOME(self, log=[]):
        """ Home all actuators """
        # Make sure the motors are on
        result, log = self.ON(log=log)
        if not result:
            log.append("Control.HOME() aborted due to SVON not being ON")
            return False, log
        # Check SVRE
        if not self._is_powered():
            log.append("Control.HOME() aborted due to SVRE not being ON -- timeout")
            return False, log
        # Check for alarms
        if self._JXC.read(self._JXC.ALARM):
            log.append("Control.HOME() aborted due to an alarm being triggered")
            return False, log
        # Check for emergency stop
        if self._JXC.read(self._JXC.ESTOP):
            log.append("Control.HOME() aborted due to emergency stop being on")
            return False, log

        # Home the actuators
        self._JXC.set_on(self._JXC.SETUP)
        self._sleep()
        if not self._JXC.read(self._JXC.SETUP):
            log.append("Control.HOME() aborted due to failure to set SETUP to ON")
        if self._wait():
            log.append("'HOME' operation finished in Control.HOME()")
            # Engage the brake
            # self.BRAKE(state=True)
            self._JXC.set_off(self._JXC.SETUP)
            return True, log
        else:
            log.append("'HOME' operation failed in Control.HOME() due to timeout")
            # Engage the brake
            # self.BRAKE(state=True)
            self._JXC.set_off(self._JXC.SETUP)
            return False, log

    def STEP(self, step_num, axis_no=None, log=[]):
        """
        Execute specified step for the controller

        Args:
        step_num (int): step number
        axis_no (int): axis number (default is None, which enables all axes)
        """
        # Make sure the motor is turned on
        if not self.ON(log=log):
            log.append("Control.STEP() aborted due to SVON not being ON")
            return False, log
        # Check for valid step number
        step_num = "%02d" % (int(step_num))
        if step_num not in self.step_inputs.keys():
            log.append(
                "Control.STEP() aborted due to unrecognized "
                "step number %02d not an " % (step_num))
            return False, log
        # Check that the motors aren't moving
        if self._is_moving():
            log.append("Control.STEP() aborted due to BUSY being on")
            return False, log
        # Check that the motors are ready to move
        if not self._is_ready():
            log.append("Control.STEP() aborted due to SETON not being on")
            return False, log

        # Set the inputs
        for addr in self.step_inputs[step_num]:
            self._JXC.set_on(addr)
        self._sleep()
        for addr in self.step_inputs[step_num]:
            if not self._JXC.read(addr):
                log.append(
                    "Control.STEP() aborted due to failure to set addr %d "
                    "to TRUE for step no %d" % (int(addr), step_num))
                return False, log

        # Drive the motor
        self._JXC.set_on(self._JXC.DRIVE)
        self._sleep()
        if not self._JXC.read(self._JXC.DRIVE):
            log.append("Control.STEP() aborted due to failure to set DRIVE to ON")
            return False, log
        # Wait for the motors to stop moving
        if self._wait():
            log.append(
                "Control.STEP() operation finished for step %d"
                % (int(step_num)))
            timeout = False
        # Otherwise the operation times out
        else:
            log.append(
                "STEP operation for step no %02d in Control.STEP() failed "
                "due to timout" % (int(step_num)))
            timeout = True

        # Reset inputs
        for addr in self.step_inputs[step_num]:
            self._JXC.set_off(addr)
        for addr in self.step_inputs[step_num]:
            if self._JXC.read(addr):
                log.append(
                    "Failed to reset addr %d after STEP command in "
                    "Control.STEP() for step no %02d"
                    % (int(addr), int(step_num)))
        # Turn off the drive
        self._JXC.set_off(self._JXC.DRIVE)
        if self._JXC.read(self._JXC.DRIVE):
            log.append(
                "Failed to turn off DRIVE after STEP command in "
                "Control.STEP() for step no %02d"
                % (int(step_num)))

        return not timeout, log

    def HOLD(self, state=True, log=[]):
        """
        Turn on and off a HOLD of the motors

        Args:
        state (bool): True to turn HOLD on, False to turn it off
        """
        # Turn HOLD on
        if state is True:
            if self._is_moving():
                self._JXC.set_on(self._JXC.HOLD)
                self._sleep()
                if not self._JXC.read(self._JXC.HOLD):
                    log.append(
                        "Failed to apply HOLD to moving grippers in "
                        "Control.HOLD()")
                    return False, log
                else:
                    log.append("Applied HOLD to moving grippers")
                return True, log
            else:
                log.append("Cannot apply HOLD when grippers are not moving")
                self._JXC.set_off(self._JXC.HOLD)
                self._sleep()
                if self._JXC.read(self._JXC.HOLD):
                    log.append(
                        "Failed to turn HOLD off after failed HOLD "
                        "operation in Control.HOLD()")
                return False, log
        # Turn HOLD off
        elif state is False:
            self._JXC.set_off(self._JXC.HOLD)
            self._sleep()
            if self._JXC.read(self._JXC.HOLD):
                log.append(
                    "Failed to turn HOLD off after failed HOLD "
                    "operation in Control.HOLD()")
                return False, log
            else:
                log.append("HOLD set to off")
                return True, log
        # Cannot understand HOLD argument
        else:
            log.append(
                "Could not understand argument %s to Control.HOLD()"
                % (str(state)))
            return False, log

    def BRAKE(self, state=True, axis=None, log=[]):
        """
        Turn the motor brakes on or off

        Args:
        state (bool): brake states. True for on, False for off
        axis (1-3): axis on which to apply the brake (default is all)
        """
        # Check the inputs
        if axis is None:
            axes = range(3)
        else:
            if type(axis) is int and int(axis) > 0 and int(axis) < 4:
                axes = [axis - 1]
            else:
                log.append(
                    "Could not understand axis %s passed to "
                    "Control.BRAKE()" % (str(axis)))
                return False, log

        # Set the brakes
        brakes = [self._JXC.BRAKE1, self._JXC.BRAKE2, self._JXC.BRAKE3]
        for ax in axes:
            if state:  # yes, it's inverted logic
                self._JXC.set_off(brakes[ax])
                log.append(
                    "Turned on BRAKE for axis %d in Control.BRAKE()"
                    % (int(ax + 1)))
            else:
                self._JXC.set_on(brakes[ax])
                log.append(
                    "Turned off BRAKE for axis %d in Control.BRAKE()"
                    % (int(ax + 1)))
        self._sleep()

        # Check the execution
        ret = True
        for ax in axes:
            read_out = self._JXC.read(brakes[ax])
            if state:  # yes, it's inverted logic
                if read_out:
                    log.append(
                        "Failed to turn on BRAKE for axis %d in "
                        "Control.BRAKE()" % (int(ax + 1)))
                    ret *= False
                else:
                    log.append(
                        "Successfully turned on BRAKE for axis %d "
                        "in Control.BRAKE()" % (int(ax + 1)))
                    ret *= True
            else:
                if not read_out:
                    log.append(
                        "Failed to turn off BRAKE for axis %d in "
                        "Control.BRAKE()" % (int(ax + 1)))
                    ret *= False
                else:
                    log.append(
                        "Successfully turned off BRAKE for axis %d "
                        "in Control.BRAKE()" % (int(ax + 1)))
                    ret *= True

        return ret, log

    def EMG(self, state=True, axis=None, log=[]):
        if axis is None:
            axes = range(3)
        else:
            if type(axis) is int and int(axis) > 0 and int(axis) < 4:
                axes = [axis - 1]
            else:
                log.append(
                    "Could not understand axis %s passed to "
                    "Control.EMG()" % (str(axis)))
                return False, log

        emg_stop = [self._JXC.EMG1, self._JXC.EMG2, self._JXC.EMG3]
        for ax in axes:
            if state:
                self._JXC.set_on(emg_stop[ax])
                log.append(
                    "Turned on EMG for axis %d in Control.EMG()"
                    % (int(ax + 1)))
            else:
                self._JXC.set_off(emg_stop[ax])
                log.append(
                    "Turned off EMG for axis %d in Control.EMG()"
                    % (int(ax + 1)))
        self._sleep()

        ret = True
        for ax in axes:
            read_out = self._JXC.read(emg_stop[ax])
            if state:
                if not read_out:
                    log.append(
                        "Failed to turn on EMG for axis %d in "
                        "Control.EMG()" % (int(ax + 1)))
                    ret *= False
                else:
                    log.append(
                        "Successfully turned on EMG for axis %d "
                        "in Control.EMG()" % (int(ax + 1)))
                    ret *= True
            else:
                if read_out:
                    log.append(
                        "Failed to turn off EMG for axis %d in "
                        "Control.EMG()" % (int(ax + 1)))
                    ret *= False
                else:
                    log.append(
                        "Successfully turned off EMG for axis %d "
                        "in Control.EMG()" % (int(ax + 1)))
                    ret *= True

        return ret, log

    def RESET(self, log=[]):
        """ Reset the alarm """
        if self._is_alarm():
            # Toggle the RESET pin on
            self._JXC.set_on(self._JXC.RESET)
            self._sleep()
            if not self._JXC.read(self._JXC.RESET):
                log.append("Failed to turn on RESET pin in Control.RESET()")
                return False, log
            # Toggle the RESET pin off
            self._JXC.set_off(self._JXC.RESET)
            self._sleep()
            if self._JXC.read(self._JXC.RESET):
                log.append(
                    "Failed to turn off RESET pin in Control.RESET() "
                    "after RESET was performed")
                return False, log
            # Check whether the ALARM was reset
            if self._is_alarm():
                log.append(
                    "Failed to RESET ALARM state. ALARM may be immutable")
                return False, log
        else:
            log.append(
                "RESET operation ignored in Control.RESET(). "
                "No ALARM detected")
        return True, log

    def OUTPUT(self, log=[]):
        """ Read the OUTPUT pins """
        out0 = int(not self._JXC.read(self._JXC.OUT0))
        out1 = int(not self._JXC.read(self._JXC.OUT1))
        out2 = int(not self._JXC.read(self._JXC.OUT2))
        out3 = int(not self._JXC.read(self._JXC.OUT3))
        return str(out0), str(out1), str(out2), str(out3), log

    def INP(self, log=[]):
        """ Read the INP pins """
        self.ON()
        self._sleep(1.)
        out = int(not self._JXC.read(self._JXC.INP))
        return bool(out), log

    def ACT(self, axis, log=[]):
        if axis == 1:
            out = int(self._JXC.read(self._JXC.ACT1))
        if axis == 2:
            out = int(self._JXC.read(self._JXC.ACT2))
        if axis == 3:
            out = int(self._JXC.read(self._JXC.ACT3))
        log.append('Actuator {} is in state {}'.format(axis, out))
        return bool(out), log

    def STATUS(self, log=[]):
        """ Print the control status """
        log.append("CONTROL STATUS:")
        log.append("IN0 = %d" % (self._JXC.read(self._JXC.IN0)))
        log.append("IN1 = %d" % (self._JXC.read(self._JXC.IN1)))
        log.append("IN2 = %d" % (self._JXC.read(self._JXC.IN2)))
        log.append("IN3 = %d" % (self._JXC.read(self._JXC.IN3)))
        log.append("IN4 = %d" % (self._JXC.read(self._JXC.IN4)))
        log.append("\n")
        log.append("SETUP = %d" % (self._JXC.read(self._JXC.SETUP)))
        log.append("HOLD  = %d" % (self._JXC.read(self._JXC.HOLD)))
        log.append("DRIVE = %d" % (self._JXC.read(self._JXC.DRIVE)))
        log.append("RESET = %d" % (self._JXC.read(self._JXC.RESET)))
        log.append("SVON  = %d" % (self._JXC.read(self._JXC.SVON)))
        log.append("\n")
        log.append("OUT0 = %d" % (not self._JXC.read(self._JXC.OUT0)))
        log.append("OUT1 = %d" % (not self._JXC.read(self._JXC.OUT1)))
        log.append("OUT2 = %d" % (not self._JXC.read(self._JXC.OUT2)))
        log.append("OUT3 = %d" % (not self._JXC.read(self._JXC.OUT3)))
        log.append("OUT4 = %d" % (not self._JXC.read(self._JXC.OUT4)))
        log.append("\n")
        log.append("BUSY  = %d" % (not self._JXC.read(self._JXC.BUSY)))
        log.append("AREA  = %d" % (not self._JXC.read(self._JXC.AREA)))
        log.append("SETON = %d" % (not self._JXC.read(self._JXC.SETON)))
        log.append("INP   = %d" % (not self._JXC.read(self._JXC.INP)))
        log.append("SVRE  = %d" % (not self._JXC.read(self._JXC.SVRE)))
        log.append("ESTOP = %d" % (self._JXC.read(self._JXC.ESTOP)))
        log.append("ALARM = %d" % (self._JXC.read(self._JXC.ALARM)))
        log.append("\n")
        log.append("BRAKE1 = %d" % (not self._JXC.read(self._JXC.BRAKE1)))
        log.append("BRAKE2 = %d" % (not self._JXC.read(self._JXC.BRAKE2)))
        log.append("BRAKE3 = %d" % (not self._JXC.read(self._JXC.BRAKE3)))
        log.append("\n")
        log.append("EMG1 = %d" % (self._JXC.read(self._JXC.EMG1)))
        log.append("EMG2 = %d" % (self._JXC.read(self._JXC.EMG2)))
        log.append("EMG3 = %d" % (self._JXC.read(self._JXC.EMG3)))
        log.append("\n")
        return True, log

    def ALARM(self, log=[]):
        """ Print the alarm status """
        log.append("ALARM = %d" % (self._JXC.read(self._JXC.ALARM)))
        return self._is_alarm(), log

    def ALARM_GROUP(self, log=[]):
        """ Identify the alarm group """
        # ID the alarm group
        if self._is_alarm():
            outs = self.OUTPUT()
            output = ''.join(outs)
            for k in self.alarm_group.keys():
                if output == self.alarm_group[k]:
                    log.append("ALARM GROUP '%s' detected" % (k))
                    return k, log
                else:
                    continue
        # Otherwise no alarm
        else:
            log.append("Ignored Control.ALARM_GROUP(). No ALARM detected")
            return None, log

        # Alarm group not understood
        log.append("ALARM_GROUP id failed -- unknown output:")
        for i in range(4):
            log.append("OUT%d = %d" % (i, int(outs[i])))
        return None, log

    # ***** Private Methods ******
    def _sleep(self, time=None):
        """ Sleep for a specified amount of time """
        if time is None:
            tm.sleep(self._tstep)
        else:
            tm.sleep(time)
        return

    def _is_moving(self):
        """ Return whether the motors are moving """
        if not self._JXC.read(self._JXC.BUSY):
            return True
        else:
            return False

    def _is_ready(self):
        """ Returns whether the motors are ready to move """
        if not self._JXC.read(self._JXC.SETON):
            return True
        else:
            return False

    def _is_powered(self):
        """ Returns whether the motors are powered """
        t = 0.  # stopwatch
        while t < self._tout:
            if self._JXC.read(self._JXC.SVRE):
                self._sleep()
                t += self._tstep
                continue
            else:
                return True
        return False

    def _is_alarm(self):
        """ Returns whether an alarm is triggered """
        if self._JXC.read(self._JXC.ALARM):
            return True
        else:
            return False

    def _wait(self, stepNum=None, timeout=None):
        """ Function to wait for step_num to finish """
        if timeout is None:
            timeout = self._tout
        t = 0.  # stopwatch
        while t < timeout:
            if self._is_moving():
                self._sleep()
                t += self._tstep
                continue
            else:
                return True
        return False


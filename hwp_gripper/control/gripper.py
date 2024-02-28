# Built-in python modules
import numpy as np

class Gripper:
    """
    The Gripper object is used to control the gripper motors

    Args:
    control (src.Control): Control object
    """
    def __init__(self, control):
        self.CTL = control

        # Step dict of pushing operations
        self.steps_push = {}
        self.steps_push["01"] = (+0.1, +0.0, +0.0)
        self.steps_push["02"] = (+0.0, +0.1, +0.0)
        self.steps_push["03"] = (+0.0, +0.0, +0.1)
        self.steps_push["04"] = (+0.5, +0.0, +0.0)
        self.steps_push["05"] = (+0.0, +0.5, +0.0)
        self.steps_push["06"] = (+0.0, +0.0, +0.5)
        self.steps_push["07"] = (+1.0, +0.0, +0.0)
        self.steps_push["08"] = (+0.0, +1.0, +0.0)
        self.steps_push["09"] = (+0.0, +0.0, +1.0)

        # Step dict of all-motor operations
        self.steps_pos = {}
        self.steps_pos["10"] = (+0.1, +0.0, +0.0)
        self.steps_pos["11"] = (+0.0, +0.1, +0.0)
        self.steps_pos["12"] = (+0.0, +0.0, +0.1)
        self.steps_pos["13"] = (-0.1, +0.0, +0.0)
        self.steps_pos["14"] = (+0.0, -0.1, +0.0)
        self.steps_pos["15"] = (+0.0, +0.0, -0.1)
        self.steps_pos["16"] = (+0.5, +0.0, +0.0)
        self.steps_pos["17"] = (+0.0, +0.5, +0.0)
        self.steps_pos["18"] = (+0.0, +0.0, +0.5)
        self.steps_pos["19"] = (-0.5, +0.0, +0.0)
        self.steps_pos["20"] = (+0.0, -0.5, +0.0)
        self.steps_pos["21"] = (+0.0, +0.0, -0.5)
        self.steps_pos["22"] = (+1.0, +0.0, +0.0)
        self.steps_pos["23"] = (+0.0, +1.0, +0.0)
        self.steps_pos["24"] = (+0.0, +0.0, +1.0)
        self.steps_pos["25"] = (-1.0, +0.0, +0.0)
        self.steps_pos["26"] = (+0.0, -1.0, +0.0)
        self.steps_pos["27"] = (+0.0, +0.0, -1.0)
        self.steps_pos["28"] = (+5.0, +0.0, +0.0)
        self.steps_pos["29"] = (+0.0, +5.0, +0.0)
        self.steps_pos["30"] = (+0.0, +0.0, +5.0)

    # ***** Public Methods *****
    def ON(self, log=[]):
        """ Turn the controller on """
        return self.CTL.ON(log=log)

    def OFF(self, log=[]):
        """ Turn the controller off """
        return self.CTL.OFF(log=log)

    def MOVE(self, mode, dist, axis_no, log=[]):
        """
        Move a specified motor a specified distance

        Args:
        mode (str): 'POS' for positioning mode, 'PUSH' for pushing mode
        dist (float): distance to move the motor [mm]
        axis_no (int): axis to move (1-3)
        """

        # Execute steps
        steps, log = self._select_steps(mode, dist, axis_no, log=log)
        if steps is None:
            log.append(
                "MOVE aborted in Gripper.MOVE() due to no selected steps")
            return False, log
        for st in steps:
            result, log = self.CTL.STEP(st, axis_no, log=log)
            if result:
                continue
            else:
                log.append(
                    "MOVE aborted in Gripper.MOVE() due to "
                    "CTL.STEP() returning False")
                # self.INP()
                return False, log
        log.append(
            "MOVE in Gripper.MOVE() completed successfully")

        return self.INP(log=log)

    def HOME(self, log=[]):
        """ Home all motors """
        # Home all motors
        result, log = self.CTL.HOME(log=log)
        if result:
            log.append(
                "HOME operation in Gripper.HOME() completed")
            return True, log
        else:
            log.append(
                "HOME operation failed in Gripper.HOME() due to CTL.HOME() returning False")
            log.append(
                "Actuators may be at unknown positions due to failed home operation")
            return False, log

    def ALARM(self, log=[]):
        """ Return the ALARM state """
        return self.CTL.ALARM(log=log)

    def RESET(self, log=[]):
        """ Reset the ALARM """
        # Obtain the alarm group
        group, log = self.CTL.ALARM_GROUP(log=log)
        if group is None:
            log.append(
                "RESET aborted in Gripper.RESET() due to no detected alarm")
            return False, log
        elif group == "B" or group == "C":
            log.append(
                "Clearing Alarm group '%s' via a RESET." % (group))
            return self.CTL.RESET(log=log)
        elif group == "D":
            log.append(
                "Clearing Alarm group '%s' via a RESET" % (group))
            return self.CTL.RESET(log=log)
        elif group == "E":
            log.append(
                "RESET failed in Gripper.RESET() due to alarm group '%s' "
                "detected. Power cycle of controller and motors required"
                % (group))
            return False, log
        else:
            log.append(
                "RESET aborted in Gripper.RESET() due to unknown alarm group")
            return False, log

        if not self.ALARM():
            log.append("Alarm successfully reset")
            return True, log
        else:
            log.append(
                "RESET aborted in Gripper.RESET() due to unknown error")
            return False, log

    def INP(self, log=[]):
        """ Return control INP """
        outs, log = self.CTL.INP(log=log)
        # for i in range(3):
        #    log.append("INP%d = %d" % (i+1, outs[i]))
        return outs, log

    def ACT(self, axis, log=[]):
        outs, log = self.CTL.ACT(axis, log=log)
        return outs, log

    def STATUS(self, log=[]):
        """ Return control status """
        return self.CTL.STATUS(log=log)

    # ***** Helper Methods *****
    def _select_steps(self, mode, dist, axis_no, log=[]):
        """ Select the steps to move a specified motor """
        d = dist
        steps_to_do = []
        while abs(d) >= 0.1:  # mm
            # Check input mode
            if mode == 'PUSH':
                steps_to_check = self.steps_push
            elif mode == 'POS':
                steps_to_check = self.steps_pos
            else:
                log.append(
                    "Did not understand mode '%s' in "
                    "GRIPPER()._select_steps()" % (mode))
                return None, log
            # Loop over steps to construct move from largest to smallest
            for k in list(steps_to_check.keys())[::-1]:
                move_step = float(steps_to_check[k][axis_no-1])
                if np.round(move_step, decimals=1) == 0.0:
                    continue
                try:
                    div = np.round(float(d)/move_step, decimals=1)
                except ZeroDivisionError:
                    continue
                if div >= 1.0:
                    steps_to_do.append(k)
                    d = np.round(d-move_step, decimals=1)
                    break
                else:
                    continue
        return steps_to_do, log

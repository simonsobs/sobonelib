# Built-in python modules
import sys as sy

class Command:
    def __init__(self, GPR=None):
        """
        The Command object handles user-input commands from the
        command-line program

        Args:
        GPR (src.Gripper): Gripper object
        """
        if GPR is None:
            raise Exception(
                "Command error: No gripper object passed "
                "to Command() constructor")
        else:
            self.GPR = GPR

    def CMD(self, user_input):
        args = user_input.split(' ')
        cmd = args[0].upper()
        if cmd == 'HELP':
            self._help()
        elif cmd == 'ON':
            result, log = self.GPR.ON(log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'OFF':
            result, log = self.GPR.OFF(log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'BRAKE':
            result, log = self._brake(args, log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'EMG':
            result, log = self._emg(args, log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'MOVE':
            result, log = self._move(args, log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'HOME':
            result, log = self.GPR.HOME(log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'INP':
            result, log = self.GPR.INP(log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'ACT':
            result, log = self._act(args, log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'ALARM':
            result, log = self.GPR.ALARM(log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'ALARM_GROUP':
            result, log = self.GPR.ALARM_STATE(log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'RESET':
            result, log = self.GPR.RESET(log=[])
            self._print_log(log)
            print(result)
            return {'result': result, 'log': log}
        elif cmd == 'STATUS':
            result, log = self.GPR.STATUS(log=[])
            return {'result': result, 'log': log}
        elif cmd == 'EXIT':
            self.GPR.OFF(log=[])
            sy.exit(0)
        else:
            print(
                "Cannot understand command '{}'. "
                "Type 'HELP' for a list of commands.".format(user_input))

    # ***** Helper Functions *****
    def _help(self):
        print("\n*** Gripper Control: Command Menu ***")
        print("HELP = help menu (you're here right now)")
        print("ON = turn grippers (SVON) on")
        print("OFF = turn grippers (SVON) off")
        print("BRAKE ON  [axis number (1-3)] = Engage brake on given axis. "
              "If axis not provided, engage brake on all axes.")
        print("BRAKE OFF [axis number (1-3)] = Release brake on given axis. "
              "If axis not provided, release brake on all axes.")
        print("MOVE [mode 'PUSH' or 'POS'] [axis number (1-3)] [distance (mm)]"
              " = move axis a given distance. Minimum step size = 0.1 mm")
        print("HOME = home all axes, resetting their positions to 0.0 mm")
        print("INP = in position (positioning operation) or pushing (pushing "
              "operation) flag")
        print("ALARM = display alarm state")
        print("RESET = reset alarm")
        print("STATUS = display status of all JXC controller bits")
        print("EXIT = exit this program\n")
        return True

    def _print_log(self, log):
        try:
            for line in log:
                print(line)
        except:
            pass

    def _brake(self, args, log=[]):
        ON = None
        if not (len(args) == 2 or len(args) == 3):
            log.append(
                "Cannot understand 'BRAKE' arguments: %s"
                % (' '.join(args[1:])))
            log.append(
                "Usage: BRAKE ON/OFF [axis number (1-3)]")
            return False, log
        if args[1].upper() == 'ON':
            ON = True
        elif args[1].upper() == 'OFF':
            ON = False
        else:
            log.append(
                "Cannot understand 'BRAKE' argument: %s"
                % (args[1]))
            log.append(
                "Usage: BRAKE ON/OFF [axis number (1-3)]\n")
            return False, log
        if len(args) == 3:
            try:
                axis = int(args[2])
            except ValueError:
                log.append(
                    "Cannot understand 'BRAKE' argument: %s"
                    % (args[2]))
                log.append(
                    "Usage: BRAKE ON/OFF [axis number (1-3)]\n")
                return False, log
            if axis < 1 or axis > 3:
                log.append(
                    "Cannot understand 'BRAKE' argument: %s"
                    % (args[2]))
                log.append(
                    "Usage: BRAKE ON/OFF [axis number (1-3)]\n")
                return False, log
            else:
                _, log = self.GPR.CTL.BRAKE(state=ON, axis=axis, log=log)
        else:
            for i in range(3):
                _, log = self.GPR.CTL.BRAKE(state=ON, axis=i+1, log=log)
        return True, log

    def _emg(self, args, log=[]):
        ON = None
        if not (len(args) == 2 or len(args) == 3):
            log.append(
                "Cannot understand 'EMG' arguments: %s"
                % (' '.join(args[1:])))
            log.append(
                "Usage: EMG ON/OFF [axis number (1-3)]")
            return False, log
        if args[1].upper() == 'ON':
            ON = True
        elif args[1].upper() == 'OFF':
            ON = False
        else:
            log.append(
                "Cannot understand 'EMG' argument: %s"
                % (args[1]))
            log.append(
                "Usage: EMG ON/OFF [axis number (1-3)]\n")
            return False, log
        if len(args) == 3:
            try:
                axis = int(args[2])
            except ValueError:
                log.append(
                    "Cannot understand 'EMG' argument: %s"
                    % (args[2]))
                log.append(
                    "Usage: EMG ON/OFF [axis number (1-3)]\n")
                return False, log
            if axis < 1 or axis > 3:
                log.append(
                    "Cannot understand 'EMG' argument: %s"
                    % (args[2]))
                log.append(
                    "Usage: EMG ON/OFF [axis number (1-3)]\n")
                return False, log
            else:
                _, log = self.GPR.CTL.EMG(state=ON, axis=axis, log=log)
        else:
            for i in range(3):
                _, log = self.GPR.CTL.EMG(state=ON, axis=i+1, log=log)
        return True, log

    def _move(self, args, log=[]):
        if not len(args) == 4:
            log.append(
                "Cannot understand 'MOVE' argument: %s"
                % (' '.join(args[1:])))
            return False, log
        else:
            mode = str(args[1]).upper()
            if not (mode == 'PUSH' or mode == 'POS'):
                log.append(
                    "Cannot understand move mode '%s'. "
                    "Must be either 'PUSH' or 'POS'"
                    % (mode))
                return False, log
            try:
                axis = int(args[2])
            except ValueError:
                log.append(
                    "Cannot understand axis number = '%s'. "
                    "Must be an integer (1-3)." % (str(axis)))
                return False, log
            if axis == 1 or axis == 2 or axis == 3:
                try:
                    dist = float(args[3])
                    result, log = self.GPR.MOVE(mode, dist, axis, log=log)
                    return result, log
                except ValueError:
                    log.append(
                        "Cannot understand relative move distance '%s'. "
                        "Must be a float." % (str(dist)))
                    return False, log
            else:
                log.append(
                    "Cannot understand axis number '%d'. "
                    "Must be an integer (1-3)." % (axis))
                return False, log

    def _act(self, args, log=[]):
        if not len(args) == 2:
            log.append(
                "Cannot understand 'ACT' argument: %s"
                % (' '.join(args[1:])))
            return False, log
        else:
            try:
                axis = int(args[1])
            except ValueError:
                log.append(
                    "Cannot understand axis '%s'. "
                    "Must be an int." % (str(args[1])))
                return False
            result, log = self.GPR.ACT(axis, log=log)
            return result, log

all: control/run.sh pru/run.sh pru/BeagleboneGripper pru/Encoder1.bin pru/Limit1.bin clean

control/run.sh:
	@echo '#!/bin/bash' > control/run.sh
	@echo 'sudo python3 gripper_server.py' >> control/run.sh
	@chmod +x control/run.sh

pru/run.sh:
	@chmod +x pru/pinconfig
	@echo '#!/bin/bash' > pru/run.sh
	@echo 'sudo ./BeagleboneGripper Encoder1.bin Encoder2.bin Limit1.bin Limit2.bin' >> pru/run.sh
	@chmod +x pru/run.sh

pru/BeagleboneGripper: pru/BeagleboneGripper.c
	@gcc -o pru/BeagleboneGripper pru/BeagleboneGripper.c -lprussdrv
	@chmod +x pru/BeagleboneGripper

pru/Encoder1.bin: pru/GripperEncoder.c pru/Encoder.cmd
	@clpru pru/GripperEncoder.c -z pru/AM335x_PRU.cmd -o pru/GripperEncoder.out
	@hexpru pru/Encoder.cmd pru/GripperEncoder.out
	@rm GripperEncoder.obj

pru/Encoder.cmd:
	@echo '-b' > pru/Encoder.cmd
	@echo '-image' >> pru/Encoder.cmd
	@echo '' >> pru/Encoder.cmd
	@echo 'ROMS {' >> pru/Encoder.cmd
	@echo '  PAGE 0:' >> pru/Encoder.cmd
	@echo '    text: o = 0x0, l = 0x1000, files={pru/Encoder1.bin}' >> pru/Encoder.cmd
	@echo '  PAGE 1:' >> pru/Encoder.cmd
	@echo '    data: o = 0x0, l = 0x1000, files={pru/Encoder2.bin}' >> pru/Encoder.cmd
	@echo '}' >> pru/Encoder.cmd

pru/Limit1.bin: pru/LimitSwitches.c pru/Limit.cmd
	@clpru pru/LimitSwitches.c -z pru/AM335x_PRU.cmd -o pru/LimitSwitches.out
	@hexpru pru/Limit.cmd pru/LimitSwitches.out
	@rm LimitSwitches.obj

pru/Limit.cmd:
	@echo '-b' > pru/Limit.cmd
	@echo '-image' >> pru/Limit.cmd
	@echo '' >> pru/Limit.cmd
	@echo 'ROMS {' >> pru/Limit.cmd
	@echo '  PAGE 0:' >> pru/Limit.cmd
	@echo '    text: o = 0x0, l = 0x1000, files={pru/Limit1.bin}' >> pru/Limit.cmd
	@echo '  PAGE 1:' >> pru/Limit.cmd
	@echo '    data: o = 0x0, l = 0x1000, files={pru/Limit2.bin}' >> pru/Limit.cmd
	@echo '}' >> pru/Limit.cmd

clean:
	@rm pru/GripperEncoder.out
	@rm pru/Encoder.cmd
	@rm pru/LimitSwitches.out
	@rm pru/Limit.cmd

reset:
	@rm control/run.sh
	@rm pru/run.sh
	@rm pru/BeagleboneGripper
	@rm pru/Encoder1.bin
	@rm pru/Encoder2.bin
	@rm pru/Limit1.bin
	@rm pru/Limit2.bin

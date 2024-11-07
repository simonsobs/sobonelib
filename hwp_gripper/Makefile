all: reset run manager clean

run: hwp-gripper-control.service hwp-gripper-pru.service control/run.sh pru/run.sh
	@sudo mv hwp-gripper-pru.service /etc/systemd/system/hwp-gripper-pru.service
	@sudo mv hwp-gripper-control.service /etc/systemd/system/hwp-gripper-control.service
	@sudo systemctl enable hwp-gripper-pru.service
	@sudo systemctl enable hwp-gripper-control.service
	@sudo systemctl start hwp-gripper-pru.service
	@sudo systemctl start hwp-gripper-control.service

hwp-gripper-control.service:
	@sed -i -e 's/\r$$//' config.txt
	@echo '[Unit]' > hwp-gripper-control.service
	@echo 'Description=hwp gripper control process' >> hwp-gripper-control.service
	@echo '' >> hwp-gripper-control.service
	@echo '[Service]' >> hwp-gripper-control.service
	@echo Environment=\"$$(awk '/PRU_PORT/ {print}' config.txt)\" >> hwp-gripper-control.service
	@echo Environment=\"$$(awk '/CONTROL_PORT/ {print}' config.txt)\" >> hwp-gripper-control.service
	@echo 'ExecStart=${PWD}/control/run.sh' >> hwp-gripper-control.service
	@echo 'User=root' >> hwp-gripper-control.service
	@echo 'WorkingDirectory=${PWD}/control/' >> hwp-gripper-control.service
	@echo 'Restart=always' >> hwp-gripper-control.service
	@echo 'RestartSec=10s' >> hwp-gripper-control.service
	@echo '' >> hwp-gripper-control.service
	@echo '[Install]' >> hwp-gripper-control.service
	@echo 'WantedBy=multi-user.target' >> hwp-gripper-control.service

hwp-gripper-pru.service:
	@sed -i -e 's/\r$$//' config.txt
	@echo '[Unit]' > hwp-gripper-pru.service
	@echo 'Description=hwp gripper pru process' >> hwp-gripper-pru.service
	@echo '' >> hwp-gripper-pru.service
	@echo '[Service]' >> hwp-gripper-pru.service
	@echo Environment=\"$$(awk '/PRU_PORT/ {print}' config.txt)\" >> hwp-gripper-pru.service
	@echo 'ExecStart=${PWD}/pru/run.sh' >> hwp-gripper-pru.service
	@echo 'User=root' >> hwp-gripper-pru.service
	@echo 'WorkingDirectory=${PWD}/pru/' >> hwp-gripper-pru.service
	@echo 'Restart=always' >> hwp-gripper-pru.service
	@echo 'RestartSec=10s' >> hwp-gripper-pru.service
	@echo '' >> hwp-gripper-pru.service
	@echo '[Install]' >> hwp-gripper-pru.service
	@echo 'WantedBy=multi-user.target' >> hwp-gripper-pru.service

control/run.sh: control/gripper_server.py
	@echo '#!/bin/bash' > control/run.sh
	@echo '/usr/bin/python3 gripper_server.py' >> control/run.sh
	@chmod +x control/run.sh

pru/run.sh: pru/BeagleboneGripper pru/Encoder1.bin pru/Encoder2.bin pru/Limit1.bin pru/Limit2.bin
	@sed -i -e 's/\r$$//' pru/pinconfig
	@chmod +x pru/pinconfig
	@echo '#!/bin/bash' > pru/run.sh
	@echo './BeagleboneGripper Encoder1.bin Encoder2.bin Limit1.bin Limit2.bin' >> pru/run.sh
	@chmod +x pru/run.sh

pru/BeagleboneGripper: pru/BeagleboneGripper.c
	@gcc -o pru/BeagleboneGripper pru/BeagleboneGripper.c -lprussdrv
	@chmod +x pru/BeagleboneGripper

pru/Encoder1.bin pru/Encoder2.bin: pru/GripperEncoder.c pru/Encoder.cmd
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

pru/Limit1.bin pru/Limit2.bin: pru/LimitSwitches.c pru/Limit.cmd
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

manager: hwp-gripper-manager.service manager.sh
	@if ! dpkg -s netcat | grep Status | grep -q installed; then \
		echo "Installing netcat, this will take a while"; \
		sudo apt update; \
		sudo apt -y install netcat; \
	fi
	@sudo mv hwp-gripper-manager.service /etc/systemd/system/hwp-gripper-manager.service
	@sudo systemctl enable hwp-gripper-manager.service
	@sudo systemctl start hwp-gripper-manager.service

hwp-gripper-manager.service:
	@echo '[Unit]' > hwp-gripper-manager.service
	@echo 'Description=hwp gripper manager process' >> hwp-gripper-manager.service
	@echo '' >> hwp-gripper-manager.service
	@echo '[Service]' >> hwp-gripper-manager.service
	@echo 'ExecStart=nc -l -p 5656 -e ${PWD}/manager.sh' >> hwp-gripper-manager.service
	@echo 'User=root' >> hwp-gripper-manager.service
	@echo 'WorkingDirectory=${PWD}/' >> hwp-gripper-manager.service
	@echo 'Restart=always' >> hwp-gripper-manager.service
	@echo 'RestartSec=2s' >> hwp-gripper-manager.service
	@echo '' >> hwp-gripper-manager.service
	@echo '[Install]' >> hwp-gripper-manager.service
	@echo 'WantedBy=multi-user.target' >> hwp-gripper-manager.service

manager.sh:
	@echo '#!/bin/bash' > manager.sh
	@echo 'read MESSAGE' >> manager.sh
	@echo '' >> manager.sh
	@echo 'if [[ $$MESSAGE == "reset" ]]' >> manager.sh
	@echo 'then' >> manager.sh
	@echo '    systemctl stop hwp-gripper-pru.service' >> manager.sh
	@echo '    systemctl stop hwp-gripper-control.service' >> manager.sh
	@echo '    sleep 1' >> manager.sh
	@echo '    systemctl start hwp-gripper-pru.service' >> manager.sh
	@echo '    systemctl start hwp-gripper-control.service' >> manager.sh
	@echo '    echo "Success"' >> manager.sh
	@echo 'else' >> manager.sh
	@echo '    echo "Unrecognized command"' >> manager.sh
	@echo 'fi' >> manager.sh
	@chmod +x manager.sh

clean:
	@rm pru/GripperEncoder.out
	@rm pru/Encoder.cmd
	@rm pru/LimitSwitches.out
	@rm pru/Limit.cmd

reset:
	@rm -f control/run.sh
	@rm -f pru/run.sh
	@rm -f manager.sh
	@rm -f pru/BeagleboneGripper
	@rm -f pru/Encoder1.bin
	@rm -f pru/Encoder2.bin
	@rm -f pru/Limit1.bin
	@rm -f pru/Limit2.bin
	@sudo systemctl disable hwp-gripper-pru.service 2> /dev/null || /bin/true
	@sudo systemctl disable hwp-gripper-control.service 2> /dev/null || /bin/true
	@sudo systemctl disable hwp-gripper-manager.service 2> /dev/null || /bin/true
	@sudo systemctl stop hwp-gripper-pru.service 2> /dev/null || /bin/true
	@sudo systemctl stop hwp-gripper-control.service 2> /dev/null || /bin/true
	@sudo systemctl stop hwp-gripper-manager.service 2> /dev/null || /bin/true
	@sudo rm /etc/systemd/system/hwp-gripper-pru.service 2> /dev/null || /bin/true
	@sudo rm /etc/systemd/system/hwp-gripper-control.service 2> /dev/null || /bin/true
	@sudo rm /etc/systemd/system/hwp-gripper-manager.service 2> /dev/null || /bin/true

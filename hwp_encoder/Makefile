all: run.sh Beaglebone_Encoder_DAQ Encoder1.bin IRIG1.bin service clean

run.sh:
	@chmod +x pinconfig
	@echo '#!/bin/bash' > run.sh
	@echo './Beaglebone_Encoder_DAQ Encoder1.bin Encoder2.bin IRIG1.bin IRIG2.bin' >> run.sh
	@chmod +x run.sh

Beaglebone_Encoder_DAQ: Beaglebone_Encoder_DAQ.c
	@gcc -o Beaglebone_Encoder_DAQ Beaglebone_Encoder_DAQ.c -lprussdrv
	@chmod +x Beaglebone_Encoder_DAQ

Encoder1.bin: Encoder_Detection.c Encoder.cmd
	@clpru Encoder_Detection.c -z AM335x_PRU.cmd -o Encoder_Detection.out
	@hexpru Encoder.cmd Encoder_Detection.out

Encoder.cmd:
	@echo '-b' > Encoder.cmd
	@echo '-image' >> Encoder.cmd
	@echo '' >> Encoder.cmd
	@echo 'ROMS {' >> Encoder.cmd
	@echo '  PAGE 0:' >> Encoder.cmd
	@echo '    text: o = 0x0, l = 0x1000, files={Encoder1.bin}' >> Encoder.cmd
	@echo '  PAGE 1:' >> Encoder.cmd
	@echo '    data: o = 0x0, l = 0x1000, files={Encoder2.bin}' >> Encoder.cmd
	@echo '}' >> Encoder.cmd

IRIG1.bin: IRIG_Detection.c IRIG.cmd
	@clpru IRIG_Detection.c -z AM335x_PRU.cmd -o IRIG_Detection.out
	@hexpru IRIG.cmd IRIG_Detection.out

IRIG.cmd:
	@echo '-b' > IRIG.cmd
	@echo '-image' >> IRIG.cmd
	@echo '' >> IRIG.cmd
	@echo 'ROMS {' >> IRIG.cmd
	@echo '  PAGE 0:' >> IRIG.cmd
	@echo '    text: o = 0x0, l = 0x1000, files={IRIG1.bin}' >> IRIG.cmd
	@echo '  PAGE 1:' >> IRIG.cmd
	@echo '    data: o = 0x0, l = 0x1000, files={IRIG2.bin}' >> IRIG.cmd
	@echo '}' >> IRIG.cmd

service: hwp-encoder-pru.service
	@sudo mv hwp-encoder-pru.service /etc/systemd/system/hwp-encoder-pru.service
	@sudo systemctl enable hwp-encoder-pru.service
	@sudo systemctl start hwp-encoder-pru.service

hwp-encoder-pru.service:
	@echo '[Unit]' > hwp-encoder-pru.service
	@echo 'Description=hwp encoder pru process' >> hwp-encoder-pru.service
	@echo '' >> hwp-encoder-pru.service
	@echo '[Service]' >> hwp-encoder-pru.service
	@echo "ExecStart=${PWD}/run.sh" >> hwp-encoder-pru.service
	@echo "User=root" >> hwp-encoder-pru.service
	@echo "WorkingDirectory=${PWD}/" >> hwp-encoder-pru.service
	@echo 'Restart=always' >> hwp-encoder-pru.service
	@echo 'RestartSec=10s' >> hwp-encoder-pru.service
	@echo '' >> hwp-encoder-pru.service
	@echo '[Install]' >> hwp-encoder-pru.service
	@echo 'WantedBy=multi-user.target' >> hwp-encoder-pru.service

clean:
	@rm Encoder_Detection.out
	@rm Encoder_Detection.obj
	@rm Encoder.cmd
	@rm IRIG_Detection.out
	@rm IRIG_Detection.obj
	@rm IRIG.cmd

reset:
	@rm run.sh
	@rm Beaglebone_Encoder_DAQ
	@rm Encoder1.bin
	@rm Encoder2.bin
	@rm IRIG1.bin
	@rm IRIG2.bin
	@sudo systemctl disable hwp-encoder-pru.service
	@sudo systemctl stop hwp-encoder-pru.service
	@sudo rm /etc/systemd/system/hwp-encoder-pru.service

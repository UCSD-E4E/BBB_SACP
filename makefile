all:
	test -d build || mkdir build
	cp ./src/microstrainServer.py ./build/microstrainServer.py
	make bbb_cli bbb_ctrl uStrain

bbb_cli:
	g++ ./src/BBB_CLI.cpp -o ./build/BBB_CLI -std=gnu++0x -Ilibraries -lzmq -g

bbb_ctrl:
	g++ ./src/BBB_Control.cpp ./libraries/PWM.cpp ./libraries/Servo.cpp ./libraries/CRServo.cpp -o ./build/BBB_Control -std=gnu++0x -Ilibraries -lzmq -g

uStrain:
	g++ ./src/parse_stream.cpp -o ./build/parse_stream -std=gnu++0x -g -lzmq -Ilibraries

clean:
	rm -r ./build
	mkdir build

run:
	make all
	./build/BBB_CLI
	python ./build/microstrainServer.py
	./build/BBB_Control.cpp

testDebug:
	g++ ./src/test.cpp ./src/MPU9150.cpp -o ./build/test -std=gnu++0x -g -Wall
	gdb ./build/test
test:
	g++ ./src/test.cpp ./src/MPU9150.cpp -o ./build/test -std=gnu++0x -g -Wall
	./build/test

testAVRMPU:
	avr-gcc -mmcu=atmega328p ./test/testMPU9150.c ./libraries/*.c ./libraries/i2cmaster/twimaster.c -Ilibraries -Ilibraries/i2cmaster -o ./build/testAVRMPU.out -Wall -Os -std=gnu99 -Wl,-u,vfprintf -lprintf_flt -lm
	avr-objcopy -j .text -j .data -O ihex ./build/testAVRMPU.out ./build/testAVRMPU.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/testAVRMPU.hex:i 

testAVR:
	avr-gcc -mmcu=atmega328p ./test/testAVR.c ./libraries/*.c -Ilibraries -o ./build/testAVR.out -Wall -Os
	avr-objcopy -j .text -j .data -O ihex ./build/testAVR.out ./build/testAVR.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/testAVR.hex:i 
testAVRUART:
	avr-gcc -mmcu=atmega328p ./test/testAVRUART.c ./libraries/*.c -Ilibraries -o ./build/testAVRUART.out -Wall -Os
	avr-objcopy -j .text -j .data -O ihex ./build/testAVRUART.out ./build/testAVRUART.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/testAVRUART.hex:i 

avr_ctrl:
	avr-gcc -mmcu=atmega328p ./src/arduino_controller.c ./libraries/uart.c ./libraries/MPU9150.c ./libraries/i2cmaster/twimaster.c -Ilibraries -Ilibraries/i2cmaster -o ./build/avr_ctrl.out -Wall -Os -std=gnu99
	avr-objcopy -j .text -j .data -O ihex ./build/avr_ctrl.out ./build/avr_ctrl.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/avr_ctrl.hex:i

testFLOPS:
	avr-gcc -mmcu=atmega328p ./test/testFlops.c -o ./build/testFlops.out -Wall
	avr-objcopy -j .text -j .data -O ihex ./build/testFlops.out ./build/testFlops.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/testFlops.hex:i

testI2C:
	avr-gcc -mmcu=atmega328p ./test/testI2C.c ./libraries/uart.c ./libraries/i2cmaster/twimaster.c -Ilibraries -Ilibraries/i2cmaster -o ./build/twitest.out -Wall
	avr-objcopy -j .text -j .data -O ihex ./build/twitest.out ./build/twitest.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/twitest.hex:i

calibrate:
	avr-gcc -mmcu=atmega328p ./test/calibMPU.c ./libraries/uart.c ./libraries/i2cmaster/twimaster.c ./libraries/MPU9150.c -Ilibraries -Ilibraries/i2cmaster -o ./build/calibrate.out -Wall -std=gnu99
	avr-objcopy -j .text -j .data -O ihex ./build/calibrate.out ./build/calibrate.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/calibrate.hex:i

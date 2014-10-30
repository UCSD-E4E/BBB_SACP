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
	avr-gcc --mmcu=atmega328p ./test/testMPU9150.c ./libraries/MPU9150.c -Ilibraries -o ./build/testAVRMPU.out -Wall
	avr-objcopy -j .text .j .data -O ihex ./build/textAVRMPU.out ./build/textAVRMPU.hex


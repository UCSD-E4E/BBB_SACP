all:
	test -d build || mkdir build
	cp ./src/microstrainServer.py ./build/microstrainServer.py
	make bbb_cli bbb_ctrl

bbb_cli:
	g++ ./src/BBB_CLI.cpp -o ./build/BBB_CLI -std=gnu++0x -Ilibraries -lzmq

bbb_ctrl:
	g++ ./src/BBB_Control.cpp ./libraries/PWM.cpp ./libraries/Servo.cpp ./libraries/CRServo.cpp -o ./build/BBB_Control -std=gnu++0x -Ilibraries -lzmq -g

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

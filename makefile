.PHONY: program clean

#bbb_cli:
#	g++ ./src/BBB_CLI.cpp -o ./build/BBB_CLI -std=gnu++0x -Ilibraries -lzmq -g
#
#bbb_ctrl:
#	g++ ./src/BBB_Control.cpp ./libraries/PWM.cpp ./libraries/Servo.cpp ./libraries/CRServo.cpp -o ./build/BBB_Control -std=gnu++0x -Ilibraries -lzmq -g
#
#uStrain:
#	g++ ./src/parse_stream.cpp -o ./build/parse_stream -std=gnu++0x -g -lzmq -Ilibraries

clean:
	rm -r ./build
	mkdir build
	rm -r ./objdir
	mkdir objdir

#testDebug:
#	g++ ./src/test.cpp ./src/MPU9150.cpp -o ./build/test -std=gnu++0x -g -Wall
#	gdb ./build/test

#test:
#	g++ ./src/test.cpp ./src/MPU9150.cpp -o ./build/test -std=gnu++0x -g -Wall
#	./build/test

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

build/testI2C.out: test/testI2C.c libraries/uart.h libraries/i2cmaster.h
	avr-gcc -mmcu=atmega328p ./test/testI2C.c ./libraries/uart.c ./libraries/i2cmaster.S -Ilibraries -o ./build/testI2C.out -Wall
	cp ./build/testI2C.out ./build/linkobj.out

calibrate:
	avr-gcc -mmcu=atmega328p ./test/calibMPU.c ./libraries/uart.c ./libraries/i2cmaster/twimaster.c ./libraries/MPU9150.c -Ilibraries -Ilibraries/i2cmaster -o ./build/calibrate.out -Wall -std=gnu99
	avr-objcopy -j .text -j .data -O ihex ./build/calibrate.out ./build/calibrate.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/calibrate.hex:i

testQuat:
	avr-gcc -mmcu=atmega328p ./test/testQuat.c ./libraries/*.c ./libraries/i2cmaster/twimaster.c -Ilibraries -Ilibraries/i2cmaster -o ./build/testAVRMPU.out -Wall -Os -std=gnu99 -Wl,-u,vfprintf -lprintf_flt -lm
	avr-objcopy -j .text -j .data -O ihex ./build/testAVRMPU.out ./build/testAVRMPU.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/testAVRMPU.hex:i 

build/testMag.out: test/testMag.c libraries/MPU9150.h libraries/uart.h libraries/i2cwrap.h libraries/i2cmaster.h
	avr-gcc -mmcu=atmega328p ./test/testMag.c -o ./build/testMag.out -Ilibraries -Wall -std=gnu99 -Os ./libraries/uart.c ./libraries/i2cwrap.c ./libraries/i2cmaster.S
	cp ./build/testMag.out ./build/linkobj.out

objdir/MPU9150.o: libraries/MPU9150.c libraries/MPU9150.h libraries/i2cwrap.h libraries/i2cmaster.h
	avr-gcc -mmcu=atmega328p ./libraries/MPU9150.c -o ./objdir/MPU9150.o -c -Ilibraries -Wall -std=gnu99 -Os

objdir/i2cmaster.o: libraries/i2cmaster.S
	avr-gcc -mmcu=atmega328p ./libraries/i2cmaster.S -o ./objdir/i2cmaster.o -c -Ilibraries -Wall -std=gnu99 -Os

objdir/i2cwrap.o: libraries/i2cwrap.c libraries/i2cmaster.h libraries/i2cwrap.h
	avr-gcc -mmcu=atmega328p ./libraries/i2cwrap.c -o ./objdir/i2cwrap.o -c -Ilibraries -Wall -std=gnu99 -Os

objdir/uart.o: libraries/uart.c libraries/uart.h
	avr-gcc -mmcu=atmega328p ./libraries/uart.c -o ./objdir/uart.o -c -Ilibraries -Wall -std=gnu99 -Os

build/linkobj.out: objdir/
	avr-gcc ./objdir/*.o -o ./build/linkobj.out

build/hexfile.hex: build/linkobj.out
	avr-objcopy -j .text -j .data -O ihex ./build/linkobj.out ./build/hexfile.hex

program: build/hexfile.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/hexfile.hex:i

build/testAlg: test/testAlg.c libraries/uart.h
	avr-gcc  -mmcu=atmega328p ./test/testAlg.c -o build/testAlg -Wall -std=gnu99 -lm -Ilibraries ./libraries/uart.c -Wl,-u,vfprintf -lprintf_flt
	cp build/testAlg build/linkobj.out

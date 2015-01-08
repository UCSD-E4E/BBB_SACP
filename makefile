# Makefile for BBB_SACP
# 
# 

GCC_OPTS = -Wall -Os -std=gnu99 -fdata-sections -ffunction-sections -mmcu=atmega328p
LD_OPTS = -Wl,-u,vfprintf,--gc-sections -lprintf_flt -lm
ILIBS = -Ilibraries

.PHONY: program clean

clean:
	rm -r ./build
	mkdir build
	rm -r ./objdir
	mkdir objdir

build/testMPU9150.out: test/testMPU9150.c libraries/MPU9150.h libraries/uart.h libraries/i2cwrap.h libraries/MPU9150.c libraries/i2cmaster.h libraries/uart.c libraries/i2cwrap.c libraries/twimaster.c
	avr-gcc -mmcu=atmega328p ./test/testMPU9150.c ./libraries/MPU9150.c ./libraries/uart.c ./libraries/i2cwrap.c ./libraries/twimaster.c -Ilibraries -o ./build/testMPU9150.out -Wall -Os -std=gnu99 -Wl,-u,vfprintf -lprintf_flt -lm
	cp ./build/testMPU9150.out ./build/linkobj.out

build/testAVR.out: objdir/testAVR.o
	avr-gcc ./objdir/testAVR.o -o ./build/testAVR.out -Wl,-u,vfprintf -lm
	cp ./build/testAVR.out ./build/linkobj.out

objdir/testAVR.o: test/testAVR.c
	avr-gcc ./test/testAVR.c -c -o ./objdir/testAVR.o -Wall -Os -std=gnu99 -mmcu=atmega328p $(ILIBS) -fdata-sections -ffunction-sections

testAVR:
	avr-gcc ./test/testAVR.c -o ./build/testAVR.out -Wall -Os -std=gnu99 -mmcu=atmega328p -fdata-sections -ffunction-sections -Wl,-u,vfprintf,--gc-sections -lprintf_flt -lm
	cp ./build/testAVR.out ./build/linkobj.out


objdir/testAVRUART.o: test/testAVRUART.c
	avr-gcc ./test/testAVRUART.c -c -o objdir/testAVRUART.o $(ILIBS) -Wall -Os -std=gnu99 -mmcu=atmega328p

build/testAVRUART.out: objdir/testAVRUART.o objdir/uart.o
	avr-gcc objdir/testAVRUART.o objdir/uart.o -o build/testAVRUART.out -Wl,-u,vfprintf,--gc-sections -lprintf_flt -lm
	cp ./build/testAVRUART.out ./build/linkobj.out

testAVRUART:
	avr-gcc -mmcu=atmega328p ./test/testAVRUART.c ./libraries/uart.c -Ilibraries -o ./build/testAVRUART.out -Wall -Os -std=gnu99 -Wl,-u,vfprintf,--gc-sections -lprintf_flt -lm
	avr-objcopy -j .text -j .data -O ihex ./build/testAVRUART.out ./build/testAVRUART.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/testAVRUART.hex:i 

build/arduino_controller.out: objdir/arduino_controller.o objdir/uart.o objdir/MPU9150.o objdir/i2cwrap.o objdir/i2cmaster.o
	avr-gcc ./objdir/arduino_controller.o ./objdir/uart.o ./objdir/MPU9150.o ./objdir/i2cwrap.o ./objdir/i2cmaster.o -o ./build/arduino_controller.out $(LD_OPTS)
	cp ./build/arduino_controller.out ./build/linkobj.out

objdir/arduino_controller.o: src/arduino_controller.c libraries/uart.h libraries/MPU9150.h
	avr-gcc -c ./src/arduino_controller.c -o objdir/arduino_controller.o $(ILIBS) $(GCC_OPTS)

objdir/i2cwrap.o: libraries/i2cwrap.c libraries/i2cmaster.h libraries/i2cwrap.h
	avr-gcc -c ./libraries/i2cwrap.c $(ILIBS) $(GCC_OPTS) -o ./objdir/i2cwrap.o

testFLOPS:
	avr-gcc -mmcu=atmega328p ./test/testFlops.c -o ./build/testFlops.out -Wall
	avr-objcopy -j .text -j .data -O ihex ./build/testFlops.out ./build/testFlops.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/testFlops.hex:i

build/testI2C.out: test/testI2C.c libraries/uart.h libraries/i2cmaster.h
	avr-gcc -mmcu=atmega328p ./test/testI2C.c ./libraries/uart.c ./libraries/twimaster.c -Ilibraries -o ./build/testI2C.out -Wall
	cp ./build/testI2C.out ./build/linkobj.out


build/testMag.out: test/testMag.c libraries/MPU9150.h libraries/uart.h libraries/i2cwrap.h libraries/i2cmaster.h
	avr-gcc -mmcu=atmega328p ./test/testMag.c ./libraries/uart.c ./libraries/i2cwrap.c ./libraries/twimaster.c $(GCC_OPTS) $(ILIBS) -o ./build/testMag.out
	cp ./build/testMag.out ./build/linkobj.out

objdir/MPU9150.o: libraries/MPU9150.c libraries/MPU9150.h libraries/i2cwrap.h libraries/i2cmaster.h
	avr-gcc ./libraries/MPU9150.c -o ./objdir/MPU9150.o -c $(GCC_OPTS) $(ILIBS)

objdir/i2cmaster.o: libraries/twimaster.c
	avr-gcc -mmcu=atmega328p ./libraries/twimaster.c -o ./objdir/i2cmaster.o -c $(ILIBS) $(GCC_OPTS)

objdir/uart.o: libraries/uart.c libraries/uart.h
	avr-gcc -mmcu=atmega328p ./libraries/uart.c -o ./objdir/uart.o -c $(ILIBS) $(GCC_OPTS)

build/linkobj.out:
	avr-gcc ./objdir/*.o -o ./build/linkobj.out -Os

build/hexfile.hex: build/linkobj.out
	avr-objcopy -j .text -j .data -O ihex ./build/linkobj.out ./build/hexfile.hex

program: build/hexfile.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/hexfile.hex:i

build/testAlg: test/testAlg.c libraries/uart.h
	gcc ./test/testAlg.c -o build/testAlg -lm $(ILIBS) $(GCC_OPTS)
	cp build/testAlg build/linkobj.out

# testQuat:
# 	avr-gcc -mmcu=atmega328p ./test/testQuat.c ./libraries/*.c ./libraries/i2cmaster/twimaster.c -Ilibraries -Ilibraries/i2cmaster -o ./build/testAVRMPU.out -Wall -Os -std=gnu99 -Wl,-u,vfprintf -lprintf_flt -lm
# 	avr-objcopy -j .text -j .data -O ihex ./build/testAVRMPU.out ./build/testAVRMPU.hex
# 	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/testAVRMPU.hex:i 
# 	
# calibrate:
# 	avr-gcc -mmcu=atmega328p ./test/calibMPU.c ./libraries/uart.c ./libraries/i2cmaster/twimaster.c ./libraries/MPU9150.c -Ilibraries -Ilibraries/i2cmaster -o ./build/calibrate.out -Wall -std=gnu99
# 	avr-objcopy -j .text -j .data -O ihex ./build/calibrate.out ./build/calibrate.hex
# 	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/calibrate.hex:i
# 	
#bbb_cli:
#	g++ ./src/BBB_CLI.cpp -o ./build/BBB_CLI -std=gnu++0x -Ilibraries -lzmq -g
#
#bbb_ctrl:
#	g++ ./src/BBB_Control.cpp ./libraries/PWM.cpp ./libraries/Servo.cpp ./libraries/CRServo.cpp -o ./build/BBB_Control -std=gnu++0x -Ilibraries -lzmq -g
#
#uStrain:
#	g++ ./src/parse_stream.cpp -o ./build/parse_stream -std=gnu++0x -g -lzmq -Ilibraries
#	
#testDebug:
#	g++ ./src/test.cpp ./src/MPU9150.cpp -o ./build/test -std=gnu++0x -g -Wall
#	gdb ./build/test
#	
#test:
#	g++ ./src/test.cpp ./src/MPU9150.cpp -o ./build/test -std=gnu++0x -g -Wall
#	./build/test

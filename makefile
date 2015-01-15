# Makefile for BBB_SACP
# 
# 

AVR_GCC_OPTS = -Wall -Os -std=gnu99 -fdata-sections -ffunction-sections -mmcu=atmega328p -c
GCC_OPTS = -Wall -Os -std=gnu99 -fdata-sections -ffunction-sections -c
AVR_LD_OPTS = -Wl,-u,vfprintf,-u,vfscanf,--gc-sections -lprintf_flt -lscanf_flt -lm -mmcu=atmega328p
LD_OPTS = -Wl,--gc-sections -lm
ILIBS = -Ilibraries

.PHONY: program clean

clean:
	rm -r ./build
	mkdir build
	rm -r ./objdir
	mkdir objdir

build/calibMag.out: objdir/calibMag.o objdir/MPU9150.o objdir/uart.o objdir/i2cwrap.o objdir/i2cmaster.o
	avr-gcc -o build/calibMag.out objdir/calibMag.o objdir/MPU9150.o objdir/uart.o objdir/i2cwrap.o objdir/i2cmaster.o $(AVR_LD_OPTS)
	cp build/calibMag.out build/linkobj.out

objdir/calibMag.o: test/calibMag.c libraries/MPU9150.h libraries/uart.h libraries/i2cwrap.h libraries/MPU9150_reg.h libraries/i2cmaster.h
	avr-gcc ./test/calibMag.c -o objdir/calibMag.o $(ILIBS) $(AVR_GCC_OPTS)

build/testMPU9150.out: objdir/testMPU9150.o objdir/MPU9150.o objdir/uart.o objdir/i2cwrap.o objdir/i2cmaster.o objdir/vmath.o
	avr-gcc ./objdir/testMPU9150.o ./objdir/uart.o ./objdir/i2cwrap.o ./objdir/i2cmaster.o ./objdir/vmath.o ./objdir/MPU9150.o -o ./build/testMPU9150.out $(AVR_LD_OPTS)
	cp ./build/testMPU9150.out ./build/linkobj.out

objdir/testMPU9150.o: test/testMPU9150.c libraries/MPU9150.h libraries/uart.h libraries/i2cwrap.h libraries/i2cmaster.h libraries/MPU9150_reg.h
	avr-gcc ./test/testMPU9150.c -o objdir/testMPU9150.o $(ILIBS) $(AVR_GCC_OPTS)

build/testAVR.out: objdir/testAVR.o
	avr-gcc ./objdir/testAVR.o -o ./build/testAVR.out $(AVR_LD_OPTS)
	cp ./build/testAVR.out ./build/linkobj.out

objdir/testAVR.o: test/testAVR.c
	avr-gcc ./test/testAVR.c -o ./objdir/testAVR.o $(ILIBS) $(AVR_GCC_OPTS)

objdir/testAVRUART.o: test/testAVRUART.c
	avr-gcc ./test/testAVRUART.c -o objdir/testAVRUART.o $(ILIBS) $(AVR_GCC_OPTS)

build/testAVRUART.out: objdir/testAVRUART.o objdir/uart.o
	avr-gcc objdir/testAVRUART.o objdir/uart.o -o build/testAVRUART.out $(AVR_LD_OPTS)
	cp ./build/testAVRUART.out ./build/linkobj.out

build/arduino_controller.out: objdir/arduino_controller.o objdir/uart.o objdir/MPU9150.o objdir/i2cwrap.o objdir/i2cmaster.o objdir/vmath.o
	avr-gcc ./objdir/arduino_controller.o ./objdir/uart.o ./objdir/MPU9150.o ./objdir/i2cwrap.o ./objdir/i2cmaster.o ./objdir/vmath.o -o ./build/arduino_controller.out $(AVR_LD_OPTS)
	cp ./build/arduino_controller.out ./build/linkobj.out

objdir/arduino_controller.o: src/arduino_controller.c libraries/uart.h libraries/MPU9150.h
	avr-gcc -c ./src/arduino_controller.c -o objdir/arduino_controller.o $(ILIBS) $(AVR_GCC_OPTS)

objdir/i2cwrap.o: libraries/i2cwrap.c libraries/i2cmaster.h libraries/i2cwrap.h
	avr-gcc ./libraries/i2cwrap.c -o ./objdir/i2cwrap.o $(ILIBS) $(AVR_GCC_OPTS)

testFLOPS:
	avr-gcc -mmcu=atmega328p ./test/testFlops.c -o ./build/testFlops.out -Wall
	avr-objcopy -j .text -j .data -O ihex ./build/testFlops.out ./build/testFlops.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/testFlops.hex:i

build/testI2C.out: test/testI2C.c libraries/uart.h libraries/i2cmaster.h
	avr-gcc -mmcu=atmega328p ./test/testI2C.c ./libraries/uart.c ./libraries/twimaster.c -Ilibraries -o ./build/testI2C.out -Wall
	cp ./build/testI2C.out ./build/linkobj.out


build/testMag.out: objdir/testMag.o objdir/MPU9150.o objdir/uart.o objdir/i2cwrap.o objdir/i2cmaster.o
	avr-gcc ./objdir/testMag.o ./objdir/uart.o ./objdir/i2cwrap.o ./objdir/i2cmaster.o $(AVR_LD_OPTS) -o ./build/testMag.out
	cp ./build/testMag.out ./build/linkobj.out

objdir/testMag.o: test/testMag.c libraries/MPU9150.h libraries/uart.h libraries/i2cwrap.h libraries/i2cmaster.h
	avr-gcc ./test/testMag.c -o ./objdir/testMag.o $(AVR_GCC_OPTS) $(ILIBS)

objdir/MPU9150.o: libraries/MPU9150.c libraries/MPU9150.h libraries/i2cwrap.h libraries/i2cmaster.h libraries/vmath.h
	avr-gcc ./libraries/MPU9150.c -o ./objdir/MPU9150.o $(AVR_GCC_OPTS) $(ILIBS)

objdir/vmath.o: libraries/vmath.h
	avr-gcc ./libraries/vmath.c -o ./objdir/vmath.o $(AVR_GCC_OPTS) $(ILIBS)

objdir/i2cmaster.o: libraries/twimaster.c
	avr-gcc ./libraries/twimaster.c -o ./objdir/i2cmaster.o $(ILIBS) $(AVR_GCC_OPTS)

objdir/uart.o: libraries/uart.c libraries/uart.h
	avr-gcc ./libraries/uart.c -o ./objdir/uart.o $(ILIBS) $(AVR_GCC_OPTS)

build/linkobj.out:
	avr-gcc ./objdir/*.o -o ./build/linkobj.out $(AVR_LD_OPTS)

build/hexfile.hex: build/linkobj.out
	avr-objcopy -j .text -j .data -O ihex ./build/linkobj.out ./build/hexfile.hex

program: build/hexfile.hex
	avrdude -c arduino -p m328p -P /dev/ttyACM0 -U flash:w:./build/hexfile.hex:i

build/testAlg.out: ./build/linkobj.out test/testAlg.c libraries/vmath.c libraries/vmath.h
	gcc ./test/testAlg.c -o objdir/testAlg.o $(ILIBS) $(GCC_OPTS)
	gcc ./libraries/vmath.c -o objdir/vmath.o $(ILIBS) $(GCC_OPTS)
	gcc ./objdir/testAlg.o ./objdir/vmath.o -o build/testAlg.out $(LD_OPTS)

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

all:
	test -d build || mkdir build
	g++ ./src/BBB_Controller.cpp -o ./build/BBB_Controller -std=gnu++0x

clean:
	rm -r ./build
	mkdir build

run:
	make all
	./build/BBB_Controller

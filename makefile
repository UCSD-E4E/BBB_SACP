all:
	g++ ./src/BBB_Controller.cpp -o ./build/BBB_Controller -std=c++11

clean:
	rm -r ./build
	mkdir build

run:
	make all
	./build/BBB_Controller

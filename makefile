exe: main.o AAstar_algorithm.o grid_input.o
	g++ -o exe  main.o AAstar_algorithm.o grid_input.o

main.o: main.cpp
	g++ -c main.cpp -std=c++11

AAstar_algorithm.o: AAstar_algorithm.cpp
	g++ -c AAstar_algorithm.cpp -std=c++11

grid_input.o: grid_input.cpp
	g++ -c grid_input.cpp 

clean:
	rm -f *.o

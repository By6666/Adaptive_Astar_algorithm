exe: main.o Astar_algorithm.o grid_input.o
	g++ -o exe  main.o Astar_algorithm.o grid_input.o

main.o: main.cpp
	g++ -c main.cpp

Astar_algorithm.o: include/Astar_algorithm.cpp
	g++ -c include/Astar_algorithm.cpp

grid_input.o: include/grid_input.cpp
	g++ -c include/grid_input.cpp

clean:
	rm -f *.o

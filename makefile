desc_comp:desc_comp.o utility.o
	g++ desc_comp.o utility.o `pkg-config --libs opencv` -o desc_comp -L. -lglut -lGLU -lGL
utility.o desc_comp.o:utility.cpp desc_comp.cpp 
	gcc `pkg-config --cflags opencv` -g -c utility.cpp desc_comp.cpp

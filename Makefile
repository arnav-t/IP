CFLAGS = -g -Wall
OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

CC = g++
% : %.cpp
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)

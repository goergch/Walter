CXXFLAGS =	-O2 -g -Wall -fmessage-length=0

OBJS =		TrajectoryExecution.o

LIBS =

TARGET =	TrajectoryExecution.exe

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)

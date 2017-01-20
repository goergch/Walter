CC=gcc
CXX=g++
RM=rm -f

SRC=../src
LIB=./lib
LDLIBS=-lpthreads
OBJS=$(LIB)/TrajectoryExecution.o $(LIB)/SerialPort.o $(LIB)/RS232/rs232-linux.o $(LIB)/mongoose.o \
     $(LIB)/main.o $(LIB)/CortexController.o $(LIB)/CmdDispatcher.o\
     $(LIB)/BezierCurve.o $(LIB)/DenavitHardenbergParam.o $(LIB)/Kinematics.o $(LIB)/logger.o\
     $(LIB)/spatial.o $(LIB)/SpeedProfile.o $(LIB)/Trajectory.o $(LIB)/TrajectoryPlayer.o $(LIB)/Util.o \
     $(LIB)/ActuatorProperty.o $(LIB)/CommDef.o $(LIB)/core.o
INCLUDES=
CXX_FLAGS= -O1 -g2 -Wall -c -fmessage-length=0 

all: server

server: $(OBJS) 
	$(CXX) $(LDFLAGS) -o server $(OBJS) $(LDLIBS) 

%.o: $(SRC)/%.cpp $(SRC)/%.h
	$(CXX) -o $@ $(INCLUDES) $(CXX_FLAGS) $<

%.o:$(SRC)/%.cpp 
	$(CXX) -o $@ -g -c $(INCLUDES) $(CXX_FLAGS) $<

clean:
	$(RM) $(OBJS) server





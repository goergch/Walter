CC=gcc
CXX=g++
RM=rm -f

SRC=./src
LIB=./Debug
OPENCV_LIB=`pkg-config opencv --libs`

#LDLIBS=$(OPENCV_LIB)  -lboost_thread -lboost_atomic -lboost_system -lboost_date_time -lboost_filesystem
LDLIBS=

OBJS=$(LIB)/BotMain.o $(LIB)/ActuatorCtrlInterface.o $(LIB)/Util.o $(LIB)/SerialPort.o $(LIB)/CommDef.o 
INCLUDES=-I./src -I./src/util
CXX_FLAGS= -std=c++11 -O0 -g2 -Wall -c -fmessage-length=0 

all: snorre

Snorre: $(OBJS) 
	$(CXX) $(LDFLAGS) -o snorre $(OBJS) $(LDLIBS) 

$(LIB)/%.o: $(SRC)/%.cpp $(SRC)/%.h
	$(CXX) -o $@ $(INCLUDES) $(CXX_FLAGS) $<

$(LIB)/%.o:$(SRC)/%.cpp 
	$(CXX) -o $@ -g -c $(INCLUDES) $(CXX_FLAGS) $<

$(LIB)/%.o:$(SRC)/util/%.cpp 
	$(CXX) -o $@ -g -c $(INCLUDES) $(CXX_FLAGS) $<


clean:
	$(RM) $(OBJS) snorre.exe





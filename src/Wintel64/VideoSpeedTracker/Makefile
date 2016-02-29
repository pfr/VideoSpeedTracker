CXXFLAGS  = -Wall -Werror

ifeq ($(shell uname),Darwin)
# Go get opencv3:
#   brew install homebrew/science/opencv3
# Here's where that lands:
CPPFLAGS  = -I/usr/local/opt/opencv3/include
LDFLAGS   = -L/usr/local/opt/opencv3/lib

# clang requires this for in-class const initialization
CXXFLAGS += -std=c++11 
endif

LDLIBS    = -lopencv_core -lopencv_videoio -lopencv_highgui -lopencv_imgproc
OBJS      = Globals.o Projection.o Snapshot.o VehicleDynamics.o videoSpeedTracker.o

all: vst

clean:
	rm -f vst $(OBJS)

vst: $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS) $(LDLIBS)

include ../common/make.rules

OBJS = Globals.o Projection.o Snapshot.o VehicleDynamics.o videoSpeedTracker.o ../common/dirlist.o

all: vst

clean:
	rm -f vst $(OBJS)

vst: $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS) $(LDLIBS)

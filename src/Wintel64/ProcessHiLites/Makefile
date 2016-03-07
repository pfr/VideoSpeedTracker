include ../common/make.rules

OBJS = ProcessHilites.o ../common/dirlist.o

all: phl

clean:
	rm -f phl $(OBJS)

phl: $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS) $(LDLIBS)

shared: trivial.o
	g++ -shared -o libtrivial.so trivial.o

static: trivial.o
	ar rcs libtrivial.a trivial.o

%.o: %.cpp
	g++ -c -fPIC -o $@ $<

.PHONY: clean

clean:
	rm -f *.a *.o *.so a.out

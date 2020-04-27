CC=g++
CFLAGS="-Wall"

debug:clean
	$(CC) $(CFLAGS) -g -o control main.cc
stable:clean
	$(CC) $(CFLAGS) -o control main.cc
clean:
	rm -vfr *~ control

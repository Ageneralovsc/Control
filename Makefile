CC=gcc
CFLAGS="-Wall"

debug:clean
	$(CC) $(CFLAGS) -g -o control main.c
stable:clean
	$(CC) $(CFLAGS) -o control main.c
clean:
	rm -vfr *~ control

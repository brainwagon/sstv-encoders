#

CFLAGS=-I/opt/local/include -O -g
LDFLAGS=-L/opt/local/lib -g
LIBS=-lsndfile -ljpeg -lm

all: robot36 robot72 martin scotty pd180 bw

SRC=robot36.c robot72.c martin.c scotty.c pd180.c bw.c 
TARFILES=Makefile README.txt $(SRC)

bw: bw.o
	$(CC) -o $@ $(LDFLAGS) bw.o -lfftw3 -lsndfile -lm 

scotty: scotty.o
	$(CC) -o $@ $(LDFLAGS) scotty.o $(LIBS)

martin: martin.o
	$(CC) -o $@ $(LDFLAGS) martin.o $(LIBS)
	
robot72: robot72.o
	$(CC) -o $@ $(LDFLAGS) robot72.o $(LIBS)

robot36: robot36.o
	$(CC) -o $@ $(LDFLAGS) robot36.o $(LIBS)

pd180: pd180.o
	$(CC) -o $@ $(LDFLAGS) pd180.o $(LIBS)

tar:	
	tar cvzf sstv.tar.gz $(TARFILES)

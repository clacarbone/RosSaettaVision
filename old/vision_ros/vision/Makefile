#---------------------------------------#
#Executable program file (library)
PROGS     = libvision.so
#Object file
OBJS      = coordinates.o couples.o frame.o labelling.o lists.o log.o v4l2_capture.o simple_image.o 
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) -Wall -fpic -shared -c -o $@ $< -I../bin/include
#-------------------------------#

#linking library
$(PROGS): $(OBJS)
	$(CC) $(OBJS) -shared -o $(PROGS) -I../bin/include -lm -lgsl
#-------------------------------#

#Installation rule
install: $(PROGS)
	mv *.so ../bin/librerie
	cp *.h ../bin/include
	mkdir -p ../bin/exec/data/vision
	find data -name "*.txt" -exec ln -s '../../../../vision/{}' ../bin/exec/data/vision/ \;	
 
#-------------------------------#

#clean rule
clean:
	rm -f $(PROGS) *.o core
#---------#

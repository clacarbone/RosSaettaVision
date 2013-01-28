#---------------------------------------#
#Executable program file (library)
PROGS     = libcontroller.so
#Object file
OBJS      = controller.o
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
	$(CC) $(OBJS) -shared -o $(PROGS) -I../bin/include -lm
#-------------------------------#

#Installation rule
install: $(PROGS)
	mv *.so ../bin/librerie
	cp *.h ../bin/include
	mkdir -p ../bin/exec/data/controller
	find data -name "*.txt" -exec ln -s '../../../../controller/{}' ../bin/exec/data/controller/ \;	
 
#-------------------------------#

#clean rule
clean:
	rm -f $(PROGS) *.o core
#---------#

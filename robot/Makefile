#---------------------------------------#
#Executable program file (library)
PROGS     = librobot.so
#Object file
OBJS      = crdtransf.o robot.o 
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) -fpic -shared -c -o $@ $< -I../bin/include
#-------------------------------#

#linking library
$(PROGS): $(OBJS) 
	$(CC) $(OBJS) -O2 -shared -o $(PROGS) -I../bin/include -L../bin/librerie
#-------------------------------#

#Installation rule
install: $(PROGS)
	mv *.so ../bin/librerie
	cp *.h ../bin/include
	mkdir -p ../bin/exec/data/robot
	find data -name "*.txt" -exec ln -s '../../../../robot/{}' ../bin/exec/data/robot/ \;	
#-------------------------------#

#clean rule
clean:
	rm -f $(PROGS) *.o core
#---------#

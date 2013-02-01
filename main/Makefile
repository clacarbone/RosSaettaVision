#---------------------------------------#
#Executable program file
PROGS     = main
#Object file
OBJS      = main.o
#Installation Directory
INSTDIR = $(prefix)/usr/bin
#Compiler flags
#CFLAGS += `c_urg-config --cflags`
#Linker flags
#LDFLAGS+= -lm -lpthread -D_REENTRANT -lxbee -lrobotcore  -lconsole -lconsensus -lvision -lvision_laser -lvisionprocessing -lkalman -lparticle -lrobot_comm -lausiliarie -lrobot_sensors -lserial_comm -liphone -lpic2netus -lviapointservoing -lobstavoid -lhokuyomiddle `c_urg-config --libs` 
LDFLAGS+= -lm -lpthread -D_REENTRANT -lvision -lmathematics -lgsl -lgslcblas 
#---------------------------------------#

#main rule
all: $(PROGS)
#--------#

#replace rule for .c source files
.c.o:	
	$(CC) -fpic -shared -g -c $(CFLAGS) -o $@ $< -I../bin/include -L../bin/librerie
#-------------------------------#

#linking
$(PROGS): $(OBJS) 
	$(CC) $(OBJS) -I../bin/include -L../bin/librerie $(LDFLAGS) -o $(PROGS)
#-------------------------------#

#Installation rule
install: $(PROGS)
	mv $(PROGS) ../bin/exec/
	cp exec.sh ../bin/exec/
	cp comp.sh ../bin/exec/
	#cp *.txt ../bin/exec/
	mkdir -p ../bin/exec/data/main
	find data -name "*.txt" -exec ln -s '../../../../main/{}' ../bin/exec/data/main/ \;	
#-------------------------------#

#clean rule
clean:
	rm -f $(PROGS) *.o core
#---------#

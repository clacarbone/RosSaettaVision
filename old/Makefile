# Makefile able to recursively execute the Makefiles inside the subdirectories

#List of subdirectories
#SUBDIRS = ausiliarie netus2pic sensor xbee_lib vision vision_laser viapoint_servoing core hokuyo_laser o_avoidance consensus console kalman particle iphone vision_processing main
SUBDIRS = mathematics robot vision kalman kalman1 wifi controller main  

#Declaration of Phony targets  (cfr. http://www.gnu.org/software/automake/manual/make/Phony-Targets.html)
.PHONY: all Makefile.pre $(SUBDIRS) 

#Default all target
all: Makefile.pre $(SUBDIRS)

Makefile.pre:
	$(SHELL) $@

#Subdir target (make and install)
$(SUBDIRS):
	$(MAKE) -C $@ && $(MAKE) -C $@ install

#Clean target
clean:
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir clean; done
	$(RM) -rf bin/
	

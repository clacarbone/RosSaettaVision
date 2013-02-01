#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=clang
CCC=clang++
CXX=clang++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=CLang-Linux-x86
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/1472/v4l2_capture.o \
	${OBJECTDIR}/_ext/1472/labelling.o \
	${OBJECTDIR}/_ext/1472/log.o \
	${OBJECTDIR}/_ext/1472/lists.o \
	${OBJECTDIR}/_ext/1472/couples.o \
	${OBJECTDIR}/_ext/1472/coordinates.o \
	${OBJECTDIR}/_ext/1472/simple_image.o \
	${OBJECTDIR}/_ext/1472/frame.o


# C Compiler Flags
CFLAGS=-Wall

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-lm -ldl

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libVision.${CND_DLIB_EXT}

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libVision.${CND_DLIB_EXT}: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.c} -shared -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libVision.${CND_DLIB_EXT} -fPIC ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/_ext/1472/v4l2_capture.o: ../v4l2_capture.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1472
	${RM} $@.d
	$(COMPILE.c) -g -I../../mathematics -I../../robot -I.. -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1472/v4l2_capture.o ../v4l2_capture.c

${OBJECTDIR}/_ext/1472/labelling.o: ../labelling.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1472
	${RM} $@.d
	$(COMPILE.c) -g -I../../mathematics -I../../robot -I.. -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1472/labelling.o ../labelling.c

${OBJECTDIR}/_ext/1472/log.o: ../log.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1472
	${RM} $@.d
	$(COMPILE.c) -g -I../../mathematics -I../../robot -I.. -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1472/log.o ../log.c

${OBJECTDIR}/_ext/1472/lists.o: ../lists.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1472
	${RM} $@.d
	$(COMPILE.c) -g -I../../mathematics -I../../robot -I.. -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1472/lists.o ../lists.c

${OBJECTDIR}/_ext/1472/couples.o: ../couples.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1472
	${RM} $@.d
	$(COMPILE.c) -g -I../../mathematics -I../../robot -I.. -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1472/couples.o ../couples.c

${OBJECTDIR}/_ext/1472/coordinates.o: ../coordinates.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1472
	${RM} $@.d
	$(COMPILE.c) -g -I../../mathematics -I../../robot -I.. -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1472/coordinates.o ../coordinates.c

${OBJECTDIR}/_ext/1472/simple_image.o: ../simple_image.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1472
	${RM} $@.d
	$(COMPILE.c) -g -I../../mathematics -I../../robot -I.. -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1472/simple_image.o ../simple_image.c

${OBJECTDIR}/_ext/1472/frame.o: ../frame.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1472
	${RM} $@.d
	$(COMPILE.c) -g -I../../mathematics -I../../robot -I.. -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1472/frame.o ../frame.c

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libVision.${CND_DLIB_EXT}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc

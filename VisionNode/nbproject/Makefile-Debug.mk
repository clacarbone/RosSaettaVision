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
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
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
	${OBJECTDIR}/_ext/760813400/main.o


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
LDLIBSOPTIONS=-Wl,-rpath,../vision/Vision/dist/Debug/GNU-Linux-x86 -L../vision/Vision/dist/Debug/GNU-Linux-x86 -lVision -Wl,-rpath,../robot/Robot/dist/Debug/GNU-Linux-x86 -L../robot/Robot/dist/Debug/GNU-Linux-x86 -lRobot -Wl,-rpath,../mathematics/Mathematics/dist/Debug/CLang-Linux-x86 -L../mathematics/Mathematics/dist/Debug/CLang-Linux-x86 -lMathematics

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/visionnode

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/visionnode: ../vision/Vision/dist/Debug/GNU-Linux-x86/libVision.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/visionnode: ../robot/Robot/dist/Debug/GNU-Linux-x86/libRobot.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/visionnode: ../mathematics/Mathematics/dist/Debug/CLang-Linux-x86/libMathematics.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/visionnode: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.c} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/visionnode ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/_ext/760813400/main.o: ../main/main.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/760813400
	${RM} $@.d
	$(COMPILE.c) -g -I../mathematics -I../robot -I../vision -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/760813400/main.o ../main/main.c

# Subprojects
.build-subprojects:
	cd ../vision/Vision && ${MAKE}  -f Makefile CONF=Debug
	cd ../robot/Robot && ${MAKE}  -f Makefile CONF=Debug
	cd ../mathematics/Mathematics && ${MAKE}  -f Makefile CONF=Debug

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/visionnode

# Subprojects
.clean-subprojects:
	cd ../vision/Vision && ${MAKE}  -f Makefile CONF=Debug clean
	cd ../robot/Robot && ${MAKE}  -f Makefile CONF=Debug clean
	cd ../mathematics/Mathematics && ${MAKE}  -f Makefile CONF=Debug clean

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc

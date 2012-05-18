# DO NOT DELETE THIS LINE -- make depend depends on it.

CC = g++

#Compile and Link Flag
#OpenCV & OpenGL
OPENCVLIB = -lhighgui -lcvaux -lcv -lml -lcxcore
OPENGLLIB = -lGL -lGLU -lglut

#Open Scene Graph
OSGLIB = -losg -losgUtil -losgGA -losgViewer -losgText -losgDB -losgShadow -lOpenThreads
OSGINCPATH = /home/umakatsu/Download/OpenSceneGraph-3.0.1/include

#for loading 3ds file
3DSLIB = -l3ds

#boost library
BOOSTINC = -I/home/umakatsu/Download/boost_1_43_0

#OpenNL
OPENNLINC=-I/home/umakatsu/desktop/Lab/M2/OpenNL3.2.1/src
OPENNLLIB=-L/home/umakatsu/desktop/Lab/M2/OpenNL3.2.1/build/Linux-Release/binaries/lib -lnl

#COMPILEFLAGS = -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3 -fno-strict-aliasing
#LINKFLAGS = -L MY_CUSTOM_LINK_PATH -lGVars3 -lcvd $(3DSLIB)
COMPILEFLAGS = -I MY_CUSTOM_INCLUDE_PATH -I./include -I/usr/include -I/usr/include/opencv -I/usr/include/flycapture -D_LINUX -D_REENTRANT -Wall -O3 -march=nocona -pipe -fno-strict-aliasing -mfpmath=sse -fomit-frame-pointer -msse3 -fno-tree-vectorize -g -fpermissive -DNDEBUG -fPIC 
LINKFLAGS = -L MY_CUSTOM_LINK_PATH -L/usr/lib/octave-3.2.3 -L/usr/local/lib  -lGLEW -Wl,-rpath /usr/lib/octave-3.2.3 -lblas -llapack -lGVars3 -lcvd -lboost_serialization -loctave -lflycapture -lgslcblas  $(3DSLIB)

EIGENLIB = -lcholmod -lumfpack -lamd -lcamd -lccolamd  -lcolamd -lcxsparse -lblas -llapack
EIGENINC = -I/home/umakatsu/desktop/Lab/M1/mesh_decomp/CHOLMOD/Include -I/home/umakatsu/desktop/Lab/M1/mesh_decomp/eigen-eigen-3.0.3/unsupported -I/home/umakatsu/desktop/Lab/M1/mesh_decomp/eigen-eigen-3.0.3

######## Object file #########
SUBDIRS = ./MeshDecomposition ./Modelling
VPATH = $(SUBDIRS)

Models=Obj/Model3ds.o\
	Obj/Texture.o
	
Object = Obj/main.o\
	Obj/ViewingModel.o\
	$(Models)\
	Obj/LSCM.o\
	Obj/IndexedMesh.o
	
all: TextureTransfer
TextureTransfer: $(Object)
	cd Obj
	$(CC) -g -o TT $(Object) $(LINKFLAGS) $(OPENGLLIB) $(OPENCVLIB) $(EIGENLIB) $(OPENNLLIB)
	cd ..

Obj/%.o:%.cc
	$(CC) $< -o $@ -c $(COMPILEFLAGS) $(OPENCVINC) $(EIGENINC) $(OPENNLINC)

Obj/%.o:%.cpp
	$(CC) $< -o $@ -c $(COMPILEFLAGS) $(OPENCVINC) $(EIGENINC) $(OPENNLINC)

clean:
	rm Obj/*.o

depend:
	rm dependecies; touch dependencies
	makedepend -fdependencies $(INCLUDEFLAGS) *.cc *.h
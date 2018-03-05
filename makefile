GCC=g++
CFLAGS=`pkg-config opencv --cflags` 
#static libs
#LIBS=-L$(PWD)/libs -lopencv_core -lopencv_imgproc -lopencv_highgui -lz -lpthread -lglib-2.0
#dynamic libs
LIBS=`pkg-config opencv --libs` -lpthread
#----- add middle object file here -----#
OBJ=scan_window.o main.o hough.o tracker.o util.o cluster.o
OBJ_DIR=./obj/
SRC=./src/
EXE=detect

OBJS = $(addprefix $(OBJ_DIR), $(OBJ))

all: obj $(EXE)

$(OBJ_DIR)%.o: $(SRC)%.cpp
	$(GCC) -g -c $< -o $@

detect: $(OBJS)
	$(GCC) -g -o $(EXE) $(OBJS) $(CFLAGS) $(LIBS)

obj:
	mkdir -p $(OBJ_DIR)

.PHONY: clean
clean:
	rm -f ./${OBJS} ./${EXE}

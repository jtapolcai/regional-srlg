CPP  = g++ 
#-D LATEX_LOG 

OBJ  =  parse_srlg.o 
LINKOBJ  = $(OBJ)
LIBS =  
INCS =  
CXXINCS =  -U HTML_LOG -I .
BIN  = srlg-show
CXXFLAGS = $(CXXINCS)   -O3
CFLAGS = $(INCS)   -O3
RM = rm -f

all: srlg-show

clean: clean-custom
	${RM} $(OBJ) $(BIN)

$(BIN): $(OBJ)
	$(CPP) $(LINKOBJ) -o "srlg-show" $(LIBS)

 parse_srlg.o:  parse_srlg.cpp
	$(CPP) -c  parse_srlg.cpp -o  parse_srlg.o $(CXXFLAGS)

lemon/arg_parser.o: lemon/arg_parser.cc
	$(CPP) -c lemon/arg_parser.cc -o lemon/arg_parser.o $(CXXFLAGS)

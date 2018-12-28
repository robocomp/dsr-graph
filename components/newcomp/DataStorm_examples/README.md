How to compile.

g++  -fmax-errors=5 Writer.cpp /usr/include/DataStorm/DataStorm.cpp -o writer -lIce++11 -lDataStorm; ./writer;
g++  -fmax-errors=5 Reader.cpp /usr/include/DataStorm/DataStorm.cpp -o reader -lIce++11 -lDataStorm; ./reader;
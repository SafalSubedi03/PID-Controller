# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Iinclude

# Directories
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

# Executable name
TARGET = $(BIN_DIR)/PID_Controller.exe

# Source and object files
SRC = $(wildcard $(SRC_DIR)/*.cpp) main.cpp
OBJ = $(SRC:%.cpp=$(OBJ_DIR)/%.o)

# Default rule
all: $(TARGET)

# Link all object files
$(TARGET): $(OBJ)
	if not exist "$(BIN_DIR)" mkdir "$(BIN_DIR)"
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source files into object files
$(OBJ_DIR)/%.o: %.cpp
	if not exist "$(dir $@)" mkdir "$(dir $@)"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
clean:
	if exist "$(OBJ_DIR)" rmdir /s /q "$(OBJ_DIR)"
	if exist "$(BIN_DIR)" rmdir /s /q "$(BIN_DIR)"

# Run executable
run: $(TARGET)
	$(TARGET)

.PHONY: all clean run

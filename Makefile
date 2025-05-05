# Compiler and flags
NVCC    := nvcc
CXX     := g++
CFLAGS  := -O2 -I SDL/include -I ./include -Xlinker /SUBSYSTEM:CONSOLE
LDFLAGS := -L./SDL/build/Debug -lSDL3

# Source and target details
SRC     	:= $(wildcard src/*.cu) $(wildcard src/*.cpp)
OUT_DIR     := output
OUT_NAME    := out
TARGET      := $(OUT_DIR)/$(OUT_NAME).exe

.PHONY: all build run clean

# Default target
all: build

# Build target with extra message
build: $(TARGET)
	@echo "Build complete! The executable is located at $(TARGET)"

# Link final executable
$(TARGET): $(SRC) | $(OUT_DIR)
	$(NVCC) $(SRC) $(CFLAGS) $(LDFLAGS) -o $(TARGET)

# Create build directory if it doesn't exist (Windows and Unix compatible)
$(OUT_DIR):
	@if not exist $(OUT_DIR) mkdir $(OUT_DIR)

# Run the program
run: build
	@echo "Running $(TARGET)..."
	"$(TARGET)"

# Clean build files while keeping SDL3.dll
clean:
	@echo "Cleaning build folder (keeping SDL3.dll)..."
	del /Q "$(OUT_DIR)\$(OUT_NAME).*" 2>nul || true

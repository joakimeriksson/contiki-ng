# Makefile for C Event Kernel Benchmarks

CC = gcc
CFLAGS = -Wall -Wextra -O3 -std=c11 -I.
LDFLAGS = -lrt

SRC_DIR = src
EXAMPLES_DIR = examples
BUILD_DIR = build

# Source files
SRCS = $(SRC_DIR)/process.c $(SRC_DIR)/clock.c

# Object files
OBJS = $(BUILD_DIR)/process.o $(BUILD_DIR)/clock.o

# Targets
BENCHMARK = $(BUILD_DIR)/benchmark

.PHONY: all clean benchmark

all: $(BENCHMARK)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BENCHMARK): $(OBJS) $(EXAMPLES_DIR)/benchmark.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(OBJS) $(EXAMPLES_DIR)/benchmark.c -o $@ $(LDFLAGS)

benchmark: $(BENCHMARK)
	@echo "=== Running C Benchmark ==="
	@./$(BENCHMARK)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: help
help:
	@echo "C Event Kernel Build System"
	@echo "Usage:"
	@echo "  make                 - Build all targets"
	@echo "  make benchmark       - Build and run benchmark"
	@echo "  make clean           - Remove build artifacts"

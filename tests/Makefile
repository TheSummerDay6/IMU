# Host-side unit test Makefile
# Compiles tests with native GCC, stubbing out STM32 hardware dependencies.
CC      = gcc
CFLAGS  = -Wall -Wextra -Wno-unused-function -Wno-unused-variable -Wno-missing-braces -D__packed= -I. -I../Components -I../Components/Controller -I../Components/Devices -I../Bsp
LDFLAGS = -lm

TESTS = test_user_lib test_controller test_transfer_function

.PHONY: all clean run

all: $(TESTS)

test_user_lib: test_user_lib.c test_stubs.h main.h bsp_dwt.h arm_math.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

test_controller: test_controller.c test_stubs.h main.h bsp_dwt.h arm_math.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

test_transfer_function: test_transfer_function.c test_stubs.h main.h bsp_dwt.h arm_math.h
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

run: $(TESTS)
	@echo "=== Running all tests ===" && \
	./test_user_lib && \
	./test_controller && \
	./test_transfer_function && \
	echo "=== All tests passed ==="

clean:
	rm -f $(TESTS)

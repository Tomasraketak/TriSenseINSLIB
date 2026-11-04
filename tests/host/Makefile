# Host test: the real library + vendored INSLIB against a simulated TriSense.
# SPDX-License-Identifier: AGPL-3.0-or-later
SRC   := ../../src
BUILD := build
CFLAGS   := -O2 -Wall -Wextra -std=gnu11 -DTRISENSE_INSLIB_HOST_TEST -I$(SRC)
CXXFLAGS := -O2 -Wall -Wextra -std=gnu++17 -DTRISENSE_INSLIB_HOST_TEST -Istubs -I$(SRC)

TU   := $(wildcard $(SRC)/inslib_tu_*.c)
COBJ := $(patsubst $(SRC)/%.c,$(BUILD)/%.o,$(TU))

all: $(BUILD)/test
	./$(BUILD)/test

$(BUILD)/%.o: $(SRC)/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/test: $(COBJ) $(SRC)/TriSenseINS.cpp test_main.cpp stubs/TriSense.h stubs/Arduino.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $(SRC)/TriSenseINS.cpp test_main.cpp $(COBJ) -lm -o $@

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
.PHONY: all clean

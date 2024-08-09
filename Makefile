# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q		:= @
NULL	:= 2>/dev/null
endif

SRC_DIR     := src
INC_DIR     := include
BUILD_DIR   := build
BIN_DIR     := bin
BINARY      := $(BIN_DIR)/main
CFLAGS      := -g -I $(INC_DIR)
CC          := gcc

SRCS        := $(wildcard $(SRC_DIR)/*.c)
OBJS        := $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRCS))

all: $(BINARY)

# Define a helper macro for debugging make errors online
# you can type "make print-OPENCM3_DIR" and it will show you
# how that ended up being resolved by all of the included
# makefiles.
print-%:
	@echo $*=$($*)

$(BINARY): $(OBJS)
	$(Q)mkdir -p $(BIN_DIR)
	$(Q)$(CC) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(Q)mkdir -p $(BUILD_DIR)
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

clean:
	$(Q)rm -f $(BUILD_DIR)/*.o $(BINARY)

.PHONY: all clean print-%

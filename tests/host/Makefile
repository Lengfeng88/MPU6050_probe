# Host-side tests for App/ and the main.c glue.  Needs only gcc, make, python3 and git.
#
#   make -C tests/host test                  # everything
#   make -C tests/host units                 # App/ modules vs. the pre-refactor code
#   make -C tests/host glue                  # old main.c vs new main.c, same simulated sensor
#   make -C tests/host glue OLD_REV=<sha>    # which commit holds the pre-refactor main.c
#                                            # (default HEAD: run before committing the refactor)
CC      ?= gcc
ROOT    := $(abspath ../..)
B       := build
OLD_REV ?= HEAD
APP_SRC := $(wildcard $(ROOT)/App/*/*.c)

COMMON  := -std=gnu11 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer \
           -I$(ROOT)/App -Ihal_stub
STRICT  := -Wall -Wextra -Werror

.PHONY: test units glue clean FORCE
test: units glue

units: $(B)/test_units
	ASAN_OPTIONS=detect_leaks=0 $(B)/test_units

glue: $(B)/glue_old $(B)/glue_new
	python3 run_glue.py

$(B):
	mkdir -p $(B)

$(B)/test_units: test_units.c hal_stub/mock_hal.c $(APP_SRC) | $(B)
	$(CC) $(COMMON) $(STRICT) $^ -o $@

FORCE:
$(B)/main_old.c: FORCE | $(B)
	git -C $(ROOT) show $(OLD_REV):Core/Src/main.c > $@

$(B)/glue_old.c: gen_glue.py $(B)/main_old.c
	python3 gen_glue.py $(B)/main_old.c $@

$(B)/glue_new.c: gen_glue.py $(ROOT)/Core/Src/main.c | $(B)
	python3 gen_glue.py $(ROOT)/Core/Src/main.c $@

$(B)/glue_old: $(B)/glue_old.c hal_stub/mock_hal.c
	$(CC) $(COMMON) -Wall $^ -o $@

$(B)/glue_new: $(B)/glue_new.c hal_stub/mock_hal.c $(APP_SRC)
	$(CC) $(COMMON) -Wall -Wextra $^ -o $@

clean:
	rm -rf $(B)

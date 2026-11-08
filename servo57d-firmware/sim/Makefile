# Host build of the control core against the virtual drive, for both boards.
CFLAGS ?= -O2 -Wall -Wextra -std=gnu11
SRC = sim_main.c sim_plant.c ../ctrl/servo.c ../ctrl/can_proto.c ../ctrl/mt6816_proto.c
HDR = sim_plant.h sim_board.h ../ctrl/servo.h ../ctrl/servo_hw.h ../ctrl/servo_hw_57d.h \
      ../ctrl/servo_hw_42d.h ../ctrl/can_proto.h ../ctrl/mt6816_proto.h

all: sim_test_57d sim_test_42d

sim_test_57d: $(SRC) $(HDR)
	$(CC) $(CFLAGS) -DSV_BOARD_57D -o $@ $(SRC) -lm
sim_test_42d: $(SRC) $(HDR)
	$(CC) $(CFLAGS) -DSV_BOARD_42D -o $@ $(SRC) -lm

check: all
	./sim_test_57d
	./sim_test_42d

check-57d: sim_test_57d
	./sim_test_57d
check-42d: sim_test_42d
	./sim_test_42d

# trace for plotting: make trace_42d.csv
trace_%.csv: sim_test_%
	./sim_test_$* --demo $@

clean:
	rm -f sim_test sim_test_57d sim_test_42d trace*.csv
.PHONY: all check check-57d check-42d clean

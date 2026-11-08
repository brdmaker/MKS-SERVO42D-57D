# Host build of the SERVO57D control core against the virtual drive.
CFLAGS ?= -O2 -Wall -Wextra -std=gnu11
SRC = sim_main.c sim_plant.c ../ctrl/servo.c ../ctrl/can_proto.c ../ctrl/mt6816_proto.c
HDR = sim_plant.h ../ctrl/servo.h ../ctrl/servo_hw.h ../ctrl/can_proto.h ../ctrl/mt6816_proto.h

sim_test: $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

check: sim_test
	./sim_test

trace.csv: sim_test
	./sim_test --demo $@

clean:
	rm -f sim_test trace.csv
.PHONY: check clean

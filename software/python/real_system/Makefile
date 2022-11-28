TEST = pd
FILE = BF_tvlqr.csv
MOTOR_ID = 7
BUS_NUMBER = 4
FILE_RECOVERY = ZF_tvlqr.csv
INIT_CONF=1


.PHONY: test
test:
	sudo -E ./robust_forward_brachiation.py $(TEST) $(FILE) $(BUS_NUMBER) $(MOTOR_ID) $(FILE_RECOVERY) $(INIT_CONF)

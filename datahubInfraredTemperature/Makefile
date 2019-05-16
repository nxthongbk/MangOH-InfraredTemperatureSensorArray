TARGETS := $(MAKECMDGOALS)

.PHONY: all $(TARGETS)
all: $(TARGETS)

$(TARGETS):
	
	mkapp -v -t $@ infraredTemperature.adef
	mkapp -v -t $@ infraredTemperatureSensor.adef
	mkapp -v -t $@ infraredTemperatureDisplay.adef

install:
	# instapp infraredTemperature.*.update $(TARGET_IP)
	# instapp infraredTemperatureSensor.*.update $(TARGET_IP)
	# instapp infraredTemperatureDisplay.*.update $(TARGET_IP)
	update infraredTemperature.*.update 192.168.2.2
	update infraredTemperatureSensor.*.update 192.168.2.2
	update infraredTemperatureDisplay.*.update 192.168.2.2

clean:
	rm -rf _build_* *.update


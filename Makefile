CC = avr-gcc
OUT = valves

# Board parameters
CPU_FREQ = 8000000L
MCU = attiny85

# Flight timer duration in seconds
FTIME = 120

# Default ISP programmer type
PROGRAMMER=usbtiny
# Baud rate
BAUD=19200

# Compilation flags
CFLAGS = -Os -DFTIME=$(FTIME) -DF_CPU=$(CPU_FREQ) -mmcu=$(MCU)

SRCDIR = src
SRC = $(wildcard $(SRCDIR)/*.c)
OBJ = $(patsubst %.c,%.o,$(SRC))

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

all: $(OBJ)
	$(CC) $(CFLAGS) $^ -o $(OUT)

# Converts elf output into .hex output for upload
# Removes .eeprom section before hex conversion
hex: all
	avr-objcopy -R .eeprom -O ihex $(OUT) $(OUT).hex

# Flash the code to the board through USBTiny ISP programmer
upload: hex
	avrdude -F -v -c $(PROGRAMMER) -p $(MCU) -U flash:w:$(OUT).hex $(BAUD)

# Removes clock divider and keeps all other default settings
fuses:
	avrdude -p $(MCU) -c $(PROGRAMMER) -U lfuse:w:0xE2:m -b $(BAUD)

clean:
	@rm $(OUT)
	@rm $(OBJ)
	@rm *.hex

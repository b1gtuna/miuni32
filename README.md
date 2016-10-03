# 32

# Should probably modify `make program` to do the following
# To program Atmega32u4 via USBTinyISP
	avrdude -p atmega32u4 -c usbtiny -v -U flash:w:miuni_lufa.hex 

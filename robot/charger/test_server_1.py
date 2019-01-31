port = "COM1"
ser = serial.Serial(port,19200,timeout=0.5)
print(ser.name + ' is open.')
  
while True:
    input = raw_input("Enter HEX cmd or 'exit'>> ")
    if input == 'exit':
        ser.close()
        print(port+' is closed.')
        exit()
 
    elif len(input) == 8:
    # user enters new register value, convert it into hex
        newRegisterValue = bits_to_hex(input)
        ser.write(newRegisterValue.decode('hex')+'\r\n')
        print('Saving...'+newRegisterValue)
        print('Receiving...')
        out = ser.read(1)
        for byte in out:
            print(byte) # present ascii
  
    else:
        cmd = input
        print('Sending...'+cmd)
        ser.write(cmd.decode('hex')+'\r\n')
        print('Receiving...')
        out = ser.read(1)
        for byte in out:
            print(byte) # present ascii

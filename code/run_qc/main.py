import esptool
import serial



def main():
    command1 = ['--chip', 'esp32', '-p', '/dev/ttyUSB0', '-b', '460800', '--before=default_reset', '--after=hard_reset', 'write_flash', '--flash_mode', 'dio', '--flash_freq', '40m', '--flash_size', '2MB', '0x1000', 'bootloader.bin', '0x10000', 'espoir_validation_microshield.bin', '0x8000', 'partition-table.bin']
    
    command2 = ['--chip', 'esp32', '--port', '/dev/ttyUSB0', '--baud', '921600', 'erase_flash']
    
    esptool.main(command1)

    ser = serial.Serial('/dev/ttyUSB0', 115200)
    data = ""

    while data.find("QC PASS") == -1:
        data = str(ser.readline())
        data = data.replace("b\'", "")
        data = data.replace("\'", "")
        data = data.replace('\\t','\t')
        data = data.replace("\\r", "")
        data = data.replace("\\n", "")
        print(data)

    ser.close()

    esptool.main(command2)
    
    print('QC completed successfully')


# Using the special variable 
# __name__
if __name__=="__main__":
    main()

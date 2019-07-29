# WARNING: this file is set up to talk EVEN parity!!!

# this was only done on the one weather station that
# went to carillion so we may wish to change. 






from pymodbus.client.sync import ModbusSerialClient 
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
import time


def readmodbus(modbusid,register,fieldtype,readtype,serialport,current_baud):
    instrument = ModbusSerialClient(method ='rtu',port=serialport,baudrate=current_baud,parity='E')
    instrument.connect()
    
    #print("Reading ModbusID %d register %d"%(modbusid,register)),
    x = None
    read = False
    validread = True
    while read == False:
        lock = True

        if lock:

            read = True
           # instrument.serial.reset_input_buffer()
            signed = False
            try:
                if fieldtype in ['int','sint']:
                    readlen = 1
                if fieldtype in ['long','slong','float']:
                    readlen = 2
                if fieldtype == 'string':
                    readlen = 10
                #print(".")
                if readtype == 3:
                    x = instrument.read_holding_registers(register,readlen,unit=modbusid,timeout=.5)
                if readtype ==4:
                    x = instrument.read_input_registers(register,readlen,unit=modbusid,timeout=.5)
                #print("..")
                instrument.close()
                #print(x)
                decoder = BinaryPayloadDecoder.fromRegisters(x.registers, byteorder=Endian.Big)
                if fieldtype == 'slong':
                    x = decoder.decode_32bit_int()
                if fieldtype == 'long':
                    x = decoder.decode_32bit_uint()
                if fieldtype == 'sint':
                    x = decoder.decode_16bit_int()
                if fieldtype == 'int':
                    x = decoder.decode_16bit_uint()
                if fieldtype == 'float':
                    x = decoder.decode_32bit_float()
                if fieldtype == 'string':
                    x = decoder.decode_string(10)

            except Exception as e:
                print(e)
                x=0
                validread = False
    #print(x,validread)
    #if fieldtype in ['sint','slong','int','long']:
    #    print("%X"%x)
    #time.sleep(.1)
    return x


def change_station_baud(modbusid,serialport,baud_setting,current_baudrate):
    instrument = ModbusSerialClient(method ='rtu',port=serialport,baudrate=current_baudrate)

    builder = BinaryPayloadBuilder(byteorder=Endian.Big)

    builder.add_16bit_int(baud_setting)
    payload = builder.build()
    
    pld = payload[0][1]|(payload[0][0]<<8)
    
    instrument.connect()
    instrument.write_register(1001,pld,unit=modbusid,timeout=.1)
    instrument.close()

def change_station_parity(modbusid,serialport,parity_setting,current_baudrate):
    instrument = ModbusSerialClient(method ='rtu',port=serialport,baudrate=current_baudrate)

    builder = BinaryPayloadBuilder(byteorder=Endian.Big)

    builder.add_16bit_int(parity_setting)
    payload = builder.build()
    
    pld = payload[0][1]|(payload[0][0]<<8)
    
    instrument.connect()
    instrument.write_register(1002,pld,unit=modbusid,timeout=.1)
    instrument.close()


    
def change_station_id(modbusid,serialport,id_setting,current_baudrate):
    instrument = ModbusSerialClient(method ='rtu',port=serialport,baudrate=current_baudrate)

    builder = BinaryPayloadBuilder(byteorder=Endian.Big)

    builder.add_16bit_int(id_setting)
    payload = builder.build()
    
    pld = payload[0][1]|(payload[0][0]<<8)
    
    instrument.connect()
    instrument.write_register(1000,pld,unit=modbusid,timeout=.1)
    instrument.close()


def update_station_settings(modbusid,serialport,current_baudrate):
    instrument = ModbusSerialClient(method ='rtu',port=serialport,baudrate=current_baudrate)

    builder = BinaryPayloadBuilder(byteorder=Endian.Big)

    builder.add_16bit_int(0x1234)
    payload = builder.build()
    
    pld = payload[0][1]|(payload[0][0]<<8)
    
    instrument.connect()
    instrument.write_register(1234,pld,unit=modbusid,timeout=.1)
    instrument.close()

    


#print("Irradiance*10",readmodbus(60,1,'sint',4,'/dev/ttyUSB0',9600))
#print("Wind *10 "    ,readmodbus(60,2,'sint',4,'/dev/ttyUSB0',9600))
#print("Temp1 *10 "   ,readmodbus(60,3,'sint',4,'/dev/ttyUSB0',9600))
#print("Temp2 *10 "   ,readmodbus(60,4,'sint',4,'/dev/ttyUSB0',9600))
#print("Temp3 *10 "   ,readmodbus(60,5,'sint',4,'/dev/ttyUSB0',9600))
#print("Temp4 *10 "   ,readmodbus(60,6,'sint',4,'/dev/ttyUSB0',9600))
#print("Temp5 *10 "   ,readmodbus(60,7,'sint',4,'/dev/ttyUSB0',9600))
#print("Snow "        ,readmodbus(60,8,'sint',4,'/dev/ttyUSB0',9600))
#change_station_baud(60,'/dev/ttyUSB0',1,9600); # change to 19200
##change_station_parity(60,'/dev/ttyUSB0',1,9600);
#update_station_settings(60,'/dev/ttyUSB0',9600);  # change to id = 61
#time.sleep(2);
#
print("Irradiance*10",readmodbus(60,1,'sint',4,'/dev/ttyUSB0',19200))
print("Wind *10 "    ,readmodbus(60,2,'sint',4,'/dev/ttyUSB0',19200))
print("Temp1 *10 "   ,readmodbus(60,3,'sint',4,'/dev/ttyUSB0',19200))
print("Temp2 *10 "   ,readmodbus(60,4,'sint',4,'/dev/ttyUSB0',19200))
print("Temp3 *10 "   ,readmodbus(60,5,'sint',4,'/dev/ttyUSB0',19200))
print("Temp4 *10 "   ,readmodbus(60,6,'sint',4,'/dev/ttyUSB0',19200))
print("Temp5 *10 "   ,readmodbus(60,7,'sint',4,'/dev/ttyUSB0',19200))
print("Snow "        ,readmodbus(60,8,'sint',4,'/dev/ttyUSB0',19200))
#change_station_baud(61,'/dev/ttyUSB0',0,19200); # change to 9600
#change_station_id(61,'/dev/ttyUSB0',60,19200);  # change to id = 60
#update_station_settings(61,'/dev/ttyUSB0',19200);  # enable settings and reset

# Bonaire Envronmental Sensor

Particulate Sensor / Pressure /Humidity /VOC sensor

24v / Modbus - inital modbusid = 65 baud = 9600

uses low cost BME680 for pressure, humidity, temp and voc
it also uses 2x pms5003 to measure particulate matter

details about the parameters are in the PDF's of the individual sensors

RS-485 and 24v are hooked up by 1 M12 connector 


modbus registers:

   1: pressure
    
   2: humidity
    
   3: a_pm1_0_atm
    
   4: a_pm2_5_atm
    
   5: a_pm10_0_atm
    
   6: a_pm1_0_cf_1
    
   7: a_pm2_5_cf_1
    
   8: a_pm10_0_cf_1
    
   9: a_p_0_3_um
   
   10: a_p_0_5_um
   
   11: a_p_1_0_um
   
   12: a_p_2_5_um
   
   13: a_p_5_0_um
   
   14: a_p_10_0_um
   
   15: b_pm1_0_atm
   
   16: b_pm2_5_atm
   
   17: b_pm10_0_atm
   
   18: b_pm1_0_cf_1
   
   19: b_pm2_5_cf_1
   
   20: b_pm10_0_cf_1
   
   21: b_p_0_3_um
   
   22: b_p_0_5_um
   
   23: b_p_1_0_um
   
   24: b_p_2_5_um
   
   25: b_p_5_0_um
   
   26: b_p_10_0_um
   
   27: voc
   
   28: temp

fields = ['a_pm1_0_atm',
'a_pm2_5_atm',
'a_pm10_0_atm',	
'a_pm1_0_cf_1',	
'a_pm2_5_cf_1',
'a_pm10_0_cf_1',
'a_p_0_3_um',
'a_p_0_5_um',
'a_p_1_0_um',
'a_p_2_5_um',
'a_p_5_0_um',
'a_p_10_0_um',
'b_pm1_0_atm',
'b_pm2_5_atm',
'b_pm10_0_atm',
'b_pm1_0_cf_1',
'b_pm2_5_cf_1',
'b_pm10_0_cf_1',
'b_p_0_3_um',
'b_p_0_5_um',
'b_p_1_0_um',
'b_p_2_5_um',
'b_p_5_0_um',
'b_p_10_0_um']

str = ""

for x in range(len(fields)):
    str = str + """
         case %d:
	      value = %s;
	      break;
"""%(x+3,fields[x])

print(str)

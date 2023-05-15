import pandas as pd
import json 
import pandas as pd

def average(s):
    o = len(s)
    sum = 0
    for i in range(o):
        parsed_dict = json.loads(s[i].replace("'", "\""))
        if 'N' in parsed_dict:
            sum = sum + float(parsed_dict['N'])
        else:
            sum = sum + float(parsed_dict['S'])

    avg = sum/o
    return avg
    

df = pd.read_csv('D:/nipun prev laptop/final_year/Project scripts/AWS_READ/my_data.csv')


humidity = df['HUMIDITY']
light_intensity = df['LIGHT_INTENSITY']
lpg_concentration = df['LPG_CONCENTRATION']
temperature = df['TEMPERATURE']
motor = df['MOTOR']
smoke = df['SMOKE']
battery_voltage = df['BATTERY_VOLTAGE']
carbon_monoxide = df['CARBON_MONOXIDE']
timestamp = df['TIMESTAMP']

print("AVERAGE Humidity is ",average(temperature))
print("AVERAGE TEMPERAATURE IS ",average(humidity))




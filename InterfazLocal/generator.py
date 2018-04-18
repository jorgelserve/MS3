
#!/usr/bin/python

# A little script to send test data to an influxdb installation
# Attention, the non-core library 'requests' is used. You'll need to install it first, check README

import json
import math
import requests
import sys
import time
import random

IP = "localhost"        # The IP of the machine hosting your influxdb instance

DB = "globo"               # The database to write to, has to exist
USER = "admin"             # The influxdb user to authenticate with
PASSWORD = "admin"  # The password of that user
TIME = 0.5                  # Delay in seconds between two consecutive updates
STATUS_MOD = 5            # The interval in which the updates count will be printed to your console


start = time.time()
randNumDistrUniform = random.uniform(5, 25)
randNums0_1 = []
rangoVariacion = [5,25]
n = 0
insertData = 15


def addToDB(r):
    if r.status_code != 204:
        print 'Failed to add point to influxdb (%d) - aborting.' %r.status_code
        sys.exit(1)


while True:
    for d in range(0, 360):
        actual = time.time()
        # Generar el dato aleatorio cada cierto tiempo
        if(actual-start > 4):
            start = time.time()
            randNumDistrUniform = random.uniform(5, 25)
            # Poner a variar insertData entre el rango de variacion [5,25], si llega a 25 baja 3, si llega a 5, sube 3
            randNums0_1.append(random.random())
            #Segun el numero aleatorio que ingresamos a la pila, decrementamos o incrementamos insertData
            if randNums0_1.pop() < 0.5:
                insertData -= 1
            else:
                insertData += 1

            if insertData < rangoVariacion[0]:
                insertData += 1
                #subir 3:
                randNums0_1.extend([1,1,1])

            if insertData > rangoVariacion[-1]:
                insertData -= 1
                randNums0_1.extend([0,0,0])

        voltajePanel1 = randNumDistrUniform
        voltajePanel2 = 25 - abs(randNumDistrUniform-5)
        tempBateria1 = insertData

        velocidad = abs(math.sin(math.radians(d)))
        seno = math.sin(math.radians(d)) * 100
        variableAlterna = velocidad + 0.3
        velocidad = velocidad*30
        conseno = -math.sin(math.radians(d)) * 100
        gps_data = 'gps_data gps_time=%e,gps_latitude=%e,gps_longitude=%e,gps_altitude=%e,gps_course=%e,gps_speed=%e' % (velocidad, voltajePanel2, conseno, voltajePanel1, voltajePanel2, tempBateria1)
        imu_data = 'imu_data imu_ax=%e,imu_ay=%e,imu_az=%e,imu_cx=%e,imu_cy=%e,imu_cz=%e,imu_gx=%e,imu_gy=%e,imu_gz=%e' % (variableAlterna, seno, conseno, variableAlterna, seno, conseno, variableAlterna, seno, conseno)
        barometer_data = 'barometer_data barometer_temperature=%e,barometer_pressure=%e,barometer_altitude=%e,barometer1_temperature=%e,barometer1_pressure=%e,barometer1_altitude=%e' % (variableAlterna, seno, conseno, variableAlterna, seno, conseno)
        gas_data = 'gas_data gas_nh3=%e,gas_co=%e,gas_no2=%e,gas_c3h8=%e,gas_c4h10=%e,gas_ch4=%e,gas_h2=%e,gas_c2h5oh=%e' % (seno, conseno, variableAlterna*4, velocidad, seno*0.2, seno*0.5, conseno*(-0.2), conseno*0.5)
        #gas1_data = 'gas1_data gas1_nh3=%e,gas1_co=%e,gas1_no2=%e,gas1_c3h8=%e,gas1_c4h10=%e,gas1_ch4=%e,gas1_h2=%e,gas1_c2h5oh=%e' % (conseno, seno, voltajePanel1, velocidad, seno*0.2, seno*0.5, conseno*0.2, conseno*0.5)
        environment_data = 'environment_data temperature_sht11=%e,humidity_sht11=%e,temperature_i2c=%e,temperature1_i2c=%e,mean_temperature=%e' % (voltajePanel1, voltajePanel2, voltajePanel1*0.5, voltajePanel2*0.5, velocidad)
        other_data = 'other_data time_miliseconds=%e,voltage_batery=%e' % (voltajePanel1, voltajePanel2)

        ## without autentication
        #r = requests.post("http://%s:8086/write?db=%s" %(IP, DB), data=v)
        ## with autentication:
        gps = requests.post("http://%s:8086/write?db=%s" %(IP, DB), auth=(USER, PASSWORD), data=gps_data)
        addToDB(gps)

        imu = requests.post("http://%s:8086/write?db=%s" %(IP, DB), auth=(USER, PASSWORD), data=imu_data)
        addToDB(imu)

        barometer = requests.post("http://%s:8086/write?db=%s" %(IP, DB), auth=(USER, PASSWORD), data=barometer_data)
        addToDB(barometer)

        gas = requests.post("http://%s:8086/write?db=%s" %(IP, DB), auth=(USER, PASSWORD), data=gas_data)
        addToDB(gas)

        #gas1 = requests.post("http://%s:8086/write?db=%s" %(IP, DB), auth=(USER, PASSWORD), data=gas1_data)
        #addToDB(gas1)

        environment = requests.post("http://%s:8086/write?db=%s" %(IP, DB), auth=(USER, PASSWORD), data=environment_data)
        addToDB(environment)

        other = requests.post("http://%s:8086/write?db=%s" %(IP, DB), auth=(USER, PASSWORD), data=other_data)
        addToDB(other)

        n += 1
        time.sleep(TIME)
        if n % STATUS_MOD == 0:
            print '%d points inserted.' % n

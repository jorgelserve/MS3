
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
TIME = 10                  # Delay in seconds between two consecutive updates
STATUS_MOD = 5            # The interval in which the updates count will be printed to your console

start = time.time()
randNumDistrUniform = random.uniform(5, 25)
randNums0_1 = []
rangoVariacion = [5,25]
n = 0
insertData = 15
nombreArchivoTramasLeer = "interfazLocal18.txt"
bnad_grafana = 0

def addToDB(r):
    if r.status_code != 204:
        print 'Failed to add point to influxdb (%d) - aborting.' %r.status_code
        sys.exit(1)
def leerTrama():
    try:
        cuentaParticion = 0
        archivo = open(nombreArchivoTramasLeer,"r")
        tramaNue = archivo.readlines()
        archivo.close()
        ultimalinea = len(tramaNue)-1
        returnedValues = str(tramaNue[ultimalinea]) #Retornar solo la ultima linea del archivo
        vectorTramas.append(returnedValues)
        if len(returnedValues) < 20: # Si la longitud de la trama es exageradamente corta reportar
            return 1
        if (vectorTramas[len(vectorTramas)-1] != vectorTramas[len(vectorTramas)-2]): #Si y solo si las tramas son diferentes retorna la trama
            return returnedValues
        else:
            return 0
    except:
        print("****ERROR LEYENDO TRAMA*****")
def procesarTrama(lineas):
    global tiempo, latitud_geo, longitud_geo, curso, velocidad, altitudGps, alturaBar, tempeBar, tempeSHT11, voltajebater, ACCX, ACCY, ACCZ, GIX, GIY, GIZ, MX, MY, MZ
    global humedadDHT11, NH3, CO, NO2, C3H8, C4H10, CH4, H2, C2H50H, tempI2C, tempADC, presionbar
    try:
        returnedValues = lineas
        valores = returnedValues.split("/")
        if (len(valores) == 4):
            print(returnedValues)
            if len(valores[3]) > 50: #Si la longitud de la trama es mayor a 50 es larga
                j = 0
                tramaIMU = False
                for i in range(len(valores[3])):
                    j = j + 1
                    bit = valores[3][j-1]
                    if bit == "f": #Si en la trama hay una f minuscula sera una trama IMU, de lo contrario GASES
                        tramaIMU = True
                if tramaIMU == True:
                    #trama larga IMU
                    #tiempo h latitud / longitud O curso / velocidad A altitud B alturabar C tempbar D TEMPSHT11 E voltajebater
                    #f ACCx G ACCy H ACCz I GIx J GIy K GIz L Mx M My N Mz
                    print("trama larga IMU")
                    tramalarga = False
                    tiempo = valores[1][0:6].strip()
                    latitud = valores[1][7:15].strip()
                    longitud = valores[2][0:9].strip()
                    curso = valores[2][10:].strip()
                    velocidad = valores[3].split("A")[0].strip()
                    altitudGps = valores[3].split("A")[1].split("B")[0].strip()
                    alturaBar = valores[3].split("B")[1].split("C")[0].strip()
                    tempeBar =  valores[3].split("C")[1].split("D")[0].strip()
                    tempeSHT11 =  valores[3].split("D")[1].split("E")[0].strip()
                    voltajebater =  valores[3].split("E")[1].split("f")[0].strip()
                    ACCX =  valores[3].split("f")[1].split("G")[0].strip()
                    ACCY =  valores[3].split("G")[1].split("H")[0].strip()
                    ACCZ =  valores[3].split("H")[1].split("I")[0].strip()
                    GIX =  valores[3].split("I")[1].split("J")[0].strip()
                    GIY =  valores[3].split("J")[1].split("K")[0].strip()
                    GIZ =  valores[3].split("K")[1].split("L")[0].strip()
                    MX =  valores[3].split("L")[1].split("M")[0].strip()
                    MY =  valores[3].split("M")[1].split("N")[0].strip()
                    MZ =  valores[3].split("N")[1].strip()

                else:
                    #trama larga gases
                    #tiempo h latitud / longitud O curso / velocidad A altitud B alturabar C tempbar D TEMPSHT11 E voltajebater
                    #F humedadDHT11 G NH3 H CO I NO2 J C3H8 K C4H10 L Ch4 M H2 N C2H50H O tempI2C P tempADC Q presionbar
                    print("trama larga gases")
                    tramalarga = False
                    tiempo = valores[1][0:6].strip()
                    latitud = valores[1][7:15].strip()
                    longitud = valores[2][0:9].strip()
                    curso = valores[2][10:].strip()
                    velocidad = valores[3].split("A")[0].strip()
                    altitudGps = valores[3].split("A")[1].split("B")[0].strip()
                    alturaBar = valores[3].split("B")[1].split("C")[0].strip()
                    tempeBar =  valores[3].split("C")[1].split("D")[0].strip()
                    tempeSHT11 =  valores[3].split("D")[1].split("E")[0].strip()
                    voltajebater =  valores[3].split("E")[1].split("F")[0].strip()
                    humedadDHT11 =  valores[3].split("F")[1].split("G")[0].strip()
                    NH3 =  valores[3].split("G")[1].split("H")[0].strip()
                    CO =  valores[3].split("H")[1].split("I")[0].strip()
                    NO2 =  valores[3].split("I")[1].split("J")[0].strip()
                    C3H8 = valores[3].split("J")[1].split("K")[0].strip()
                    C4H10 = valores[3].split("K")[1].split("L")[0].strip()
                    CH4 = valores[3].split("L")[1].split("M")[0].strip()
                    H2 =  valores[3].split("M")[1].split("N")[0].strip()
                    C2H50H =  valores[3].split("N")[1].split("0")[0].strip()
                    tempI2C =  valores[3].split("O")[1].split("P")[0].strip()
                    tempADC =  valores[3].split("P")[1].split("Q")[0].strip()
                    presionbar =  valores[3].split("Q")[1].strip()
            else:
                #trama corta
                #tiempo h latitud / longitud O curso / velocidad A altitud B alturabar C tempbar D TEMPSHT11 E voltajebater
                print("trama corta")
                tramalarga = False
                tiempo = valores[1][0:6].strip()
                latitud = valores[1][7:15].strip()
                longitud = valores[2][0:9].strip()
                curso = valores[2][10:].strip()
                velocidad = valores[3].split("A")[0].strip()
                altitudGps = valores[3].split("A")[1].split("B")[0].strip()
                alturaBar = valores[3].split("B")[1].split("C")[0].strip()
                tempeBar =  valores[3].split("C")[1].split("D")[0].strip()
                tempeSHT11 =  valores[3].split("D")[1].split("E")[0].strip()
                voltajebater =  valores[3].split("E")[1].strip()
            datosTransfor = transformarTrama(latitud,longitud) #Convierte las coordenadas a decimales
            latitud_geo = datosTransfor[0]
            longitud_geo = datosTransfor[1]
            altitud_geo = altitudGps
            try:
                print("tiempo: " + str(float(tiempo)))
                print("latitud_geo: " + str(float(latitud_geo)))
                print("longitud_geo: " + str(float(longitud_geo)))
                print("altitud_geo: " + str(float(altitud_geo)))
                print("curso: " + str(float(curso)))
                print("velocidad: " + str(float(velocidad)))
                print("alturabar: " + str(float(alturabar)))
                print("temperaturaSHT11: " +  str(float(tempeSHT11)/100))
                print("voltajebater: " + str(float(voltajebater)))
                print("temperaturaBar: " +  str(float(tempeBar)/100))
            except:
                print("***** ERROR EN VARIABLES PROCESADAS *****")

            return [latitud_geo,longitud_geo,altitud_geo]
    except:
        print("******ERROR PROCESANDO TRAMA********")
def transformarTrama(latitud,longitud):
    try:
        longitud_geo = 0
        latitud_geo = 0
        if(len(longitud) > 8):
            grados_lo = float(longitud[1:3])
            minutos_lo = float(longitud[3:5])
            decimasMinutos_lo = float("0"+longitud[5:8])
        elif(len(longitud) > 9):
            grados_lo = float(longitud[0:2])
            minutos_lo = float(longitud[2:4])
            decimasMinutos_lo = float(longitud[4:8])
        else:
            grados_lo = float(longitud[1:2])
            minutos_lo = float(longitud[2:4])
            decimasMinutos_lo = float("0"+longitud[4:7])
        longitud_geo = (grados_lo + (minutos_lo + decimasMinutos_lo)/60)
        if(longitud[len(longitud)-1] == "W"):
            longitud_geo = longitud_geo * -1
        longitud_geo = str(longitud_geo)
        longitud_geo = longitud_geo[0:9]
        longitud_geo = longitud_geo.strip()
        if(len(latitud) == 9):
            grados_la = float(latitud[1:3])
            minutos_la = float(latitud[3:5])
            decimasMinutos_la = float("0"+latitud[5:8])
        elif(len(latitud) > 9):
            grados_la = float(latitud[0:2])
            minutos_la = float(latitud[2:4])
            decimasMinutos_la = float(latitud[4:8])
        else:
            grados_la = float(latitud[1:2])
            minutos_la = float(latitud[2:4])
            decimasMinutos_la = float("0"+latitud[4:7])
        latitud_geo = (grados_la + (minutos_la + decimasMinutos_la)/60)
        if(latitud[len(latitud)-1] == "S"):
            latitud_geo = latitud_geo * -1
        latitud_geo = str(latitud_geo)
        latitud_geo = latitud_geo[0:9]
        latitud_geo = latitud_geo.strip()
        return [latitud_geo,longitud_geo]
    except:
        print("*****ERROR TRANSFORMANDO TRAMA*****")

while True:
    try:
        trama = leerTrama()
        if trama != 0 and trama != 1:
            cuentatrama = 0
            cuentadesconocida = 0
            coordenadas = procesarTrama(trama)
            print("__________________________________________")
            band_grafana = 1
        if trama == 0:
            if cuentatrama < 1:
                print("Esperando trama nueva.....")
                print("__________________________________________")
            cuentatrama = cuentatrama + 1
        if trama == 1:
            if cuentadesconocida < 1:
                print("Trama leida desconocida")
                print("__________________________________________")
            cuentadesconocida = cuentadesconocida + 1
        time.sleep(0.1) #leer archivo tramas cada 100 milisegundos
        if band_grafana == 1:
            gps_data = 'gps_data gps_time=%e,gps_latitude=%e,gps_longitude=%e,gps_altitude=%e,gps_course=%e,gps_speed=%e' % (tiempo, latitud_geo, longitud_geo, altitudGps, curso, velocidad)
            imu_data = 'imu_data imu_ax=%e,imu_ay=%e,imu_az=%e,imu_cx=%e,imu_cy=%e,imu_cz=%e,imu_gx=%e,imu_gy=%e,imu_gz=%e' % (ACCX, ACCY, ACCZ, GIX, GIY, GIZ, MX, MY, MZ)
            barometer_data = 'barometer_data barometer_temperature=%e,barometer_pressure=%e,barometer_altitude=%e' % (tempeBar, presionbar, alturaBar)
            gas_data = 'gas_data gas_nh3=%e,gas_co=%e,gas_no2=%e,gas_c3h8=%e,gas_c4h10=%e,gas_ch4=%e,gas_h2=%e,gas_c2h5oh=%e' % (NH3, CO, NO2, C3H8, C4H10, CH4, H2, C2H50H)
            #gas1_data = 'gas1_data gas1_nh3=%e,gas1_co=%e,gas1_no2=%e,gas1_c3h8=%e,gas1_c4h10=%e,gas1_ch4=%e,gas1_h2=%e,gas1_c2h5oh=%e' % (conseno, seno, voltajePanel1, velocidad, seno*0.2, seno*0.5, conseno*0.2, conseno*0.5)
            environment_data = 'environment_data temperature_sht11=%e,humidity_sht11=%e,temperature_i2c=%e,mean_temperature=%e' % (tempeSHT11, humedadDHT11, tempI2C, tempADC)
            other_data = 'other_data time_miliseconds=%e,voltage_batery=%e' % (voltajePanel1, voltajebater)

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
            band_grafana = 0
            n += 1
            time.sleep(TIME)
            if n % STATUS_MOD == 0:
                print '%d points inserted.' % n
    except:
        print("******** ERROR DE EJECUCION ***********")

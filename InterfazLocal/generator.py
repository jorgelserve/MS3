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
nombreArchivoTramasLeer = "Interfazprueba.txt"
band_grafana = 0
time_miliseconds = 1
band_init = 0

global vectorTramas
vectorTramas = ['/0/0/0']
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
    except Exception as e:
	print e
        print("****ERROR LEYENDO TRAMA*****")
def procesarTrama(lineas, band_init):
    global tiempo, latitud_geo, longitud_geo, curso, velocidad, altitudGps, alturaBar, tempeBar, tempeSHT11, voltajebater, ACCX, ACCY, ACCZ, GIX, GIY, GIZ, MX, MY, MZ
    global humedadDHT11, NH3, CO, NO2, C3H8, C4H10, CH4, H2, C2H50H, tempI2C, tempADC, presionbar
    if band_init == 0:
        tiempo = 0; latitud_geo = 0; longitud_geo = 0; curso = 0; velocidad = 0; altitudGps = 0; alturaBar = 0; tempeBar = 0; tempeSHT11 = 0; voltajebater = 0; ACCX = 0; ACCY = 0; ACCZ = 0; GIX = 0; GIY = 0; GIZ = 0; MX = 0; MY = 0; MZ = 0;
        humedadDHT11 = 0; NH3 = 0; CO = 0; NO2 = 0; C3H8 = 0; C4H10 = 0; CH4 = 0; H2 = 0; C2H50H = 0; tempI2C = 0; tempADC = 0; presionbar = 0;
        band_init = 1
    try:
        returnedValues = lineas
        valores = returnedValues.split("/")
        if (len(valores) == 4):
            print(returnedValues)
            tramaIMU = False
            tramaGases = False
            for i in range(len(valores[3])):
                i = i + 1
                bit = valores[3][i-1]
                if bit == "f": #Si en la trama hay una f minuscula sera una trama IMU
                    tramaIMU = True
                if bit == "F":
                    tramaGases = True
            if tramaIMU == True:
                #trama larga IMU
                #tiempo h latitud / longitud O curso / velocidad A altitud B alturaBar C tempbar D tempeSHT11 E voltajebater
                #f ACCx G ACCy H ACCz I GIx J GIy K GIz L Mx M My N Mz
                for i in range(len(valores[3])):
                    i = i + 1
                    bit = valores[3][i-1]
                    if bit == "B": #Si se cumple se garantiza existencia de trama hasta "B"
                        tiempo = valores[1][0:6].strip()
                        latitud = valores[1][7:15].strip()
                        longitud = valores[2][0:9].strip()
                        curso = valores[2][10:].strip()
                        velocidad = valores[3].split("A")[0].strip()
                        altitudGps = valores[3].split("A")[1].split("B")[0].strip()
                        datosTransfor = transformarTrama(latitud,longitud) #Convierte las coordenadas a decimales
                        latitud_geo = datosTransfor[0]
                        longitud_geo = datosTransfor[1]
                        altitud_geo = altitudGps
                        datoToSetOK = True
                        print("tiempo: " + str(float(tiempo)))
                        print("latitud_geo: " + str(float(latitud_geo)))
                        print("longitud_geo: " + str(float(longitud_geo)))
                        print("altitud_geo: " + str(float(altitud_geo)))
                        print("curso: " + str(float(curso)))
                        print("velocidad: " + str(float(velocidad)))
                    if bit == "C": #Si se cumple se garantiza existencia de trama hasta "C"
                        alturaBar = valores[3].split("B")[1].split("C")[0].strip()
                        print("alturaBar: " + str(float(alturaBar)))
                    if bit == "D": #Si se cumple se garantiza existencia de trama hasta "D"
                        tempeBar =  valores[3].split("C")[1].split("D")[0].strip()
                        print("temperaturaBar: " +  str(float(tempeBar)/100))
                    if bit == "E": #Si se cumple se garantiza existencia de trama hasta "E"
                        tempeSHT11 =  valores[3].split("D")[1].split("E")[0].strip()
                        print("temperaturaSHT11: " +  str(float(tempeSHT11)/100))
                    if bit == "f": #Si se cumple se garantiza existencia de trama hasta "f"
                        voltajebater =  valores[3].split("E")[1].split("f")[0].strip()
                        print("voltajebater: " + str(float(voltajebater)))
                    if bit == "G": #Si se cumple se garantiza existencia de trama hasta "G"
                        ACCX =  valores[3].split("f")[1].split("G")[0].strip()
                    if bit == "H": #Si se cumple se garantiza existencia de trama hasta "H"
                        ACCY =  valores[3].split("G")[1].split("H")[0].strip()
                    if bit == "I": #Si se cumple se garantiza existencia de trama hasta "I"
                        ACCZ =  valores[3].split("H")[1].split("I")[0].strip()
                    if bit == "J": #Si se cumple se garantiza existencia de trama hasta "J"
                        GIX =  valores[3].split("I")[1].split("J")[0].strip()
                    if bit == "K": #Si se cumple se garantiza existencia de trama hasta "K"
                        GIY =  valores[3].split("J")[1].split("K")[0].strip()
                    if bit == "L": #Si se cumple se garantiza existencia de trama hasta "L"
                        GIZ =  valores[3].split("K")[1].split("L")[0].strip()
                    if bit == "M": #Si se cumple se garantiza existencia de trama hasta "M"
                        MX =  valores[3].split("L")[1].split("M")[0].strip()
                    if bit == "N": #Si se cumple se garantiza existencia de trama hasta "N"
                        MY =  valores[3].split("M")[1].split("N")[0].strip()
                        MZ =  valores[3].split("N")[1].strip()
                        print("trama larga IMU completa")
            elif tramaGases == True:
                #trama larga gases
                #tiempo h latitud / longitud O curso / velocidad A altitud B alturaBar C tempbar D tempeSHT11 E voltajebater
                #F humedadDHT11 G NH3 H CO I NO2 J C3H8 K C4H10 L Ch4 M H2 N C2H50H O tempI2C P tempADC Q presionbar
                for i in range(len(valores[3])):
                    i = i + 1
                    bit = valores[3][i-1]
                    if bit == "B": #Si se cumple se garantiza existencia de trama hasta "B"
                        tiempo = valores[1][0:6].strip()
                        latitud = valores[1][7:15].strip()
                        longitud = valores[2][0:9].strip()
                        curso = valores[2][10:].strip()
                        velocidad = valores[3].split("A")[0].strip()
                        altitudGps = valores[3].split("A")[1].split("B")[0].strip()
                        datosTransfor = transformarTrama(latitud,longitud) #Convierte las coordenadas a decimales
                        latitud_geo = datosTransfor[0]
                        longitud_geo = datosTransfor[1]
                        altitud_geo = altitudGps
                        datoToSetOK = True
                        print("tiempo: " + str(float(tiempo)))
                        print("latitud_geo: " + str(float(latitud_geo)))
                        print("longitud_geo: " + str(float(longitud_geo)))
                        print("altitud_geo: " + str(float(altitud_geo)))
                        print("curso: " + str(float(curso)))
                        print("velocidad: " + str(float(velocidad)))
                    if bit == "C": #Si se cumple se garantiza existencia de trama hasta "C"
                        alturaBar = valores[3].split("B")[1].split("C")[0].strip()
                        print("alturaBar: " + str(float(alturaBar)))
                    if bit == "D": #Si se cumple se garantiza existencia de trama hasta "D"
                        tempeBar =  valores[3].split("C")[1].split("D")[0].strip()
                        print("temperaturaBar: " +  str(float(tempeBar)/100))
                    if bit == "E": #Si se cumple se garantiza existencia de trama hasta "E"
                        tempeSHT11 =  valores[3].split("D")[1].split("E")[0].strip()
                        print("temperaturaSHT11: " +  str(float(tempeSHT11)/100))
                    if bit == "F": #Si se cumple se garantiza existencia de trama hasta "F"
                        voltajebater =  valores[3].split("E")[1].split("F")[0].strip()
                        print("voltajebater: " + str(float(voltajebater)))
                    if bit == "G": #Si se cumple se garantiza existencia de trama hasta "G"
                        humedadDHT11 =  valores[3].split("F")[1].split("G")[0].strip()
                    if bit == "H": #Si se cumple se garantiza existencia de trama hasta "H"
                        NH3 =  valores[3].split("G")[1].split("H")[0].strip()
                    if bit == "I": #Si se cumple se garantiza existencia de trama hasta "I"
                        CO =  valores[3].split("H")[1].split("I")[0].strip()
                    if bit == "J": #Si se cumple se garantiza existencia de trama hasta "J"
                        NO2 =  valores[3].split("I")[1].split("J")[0].strip()
                    if bit == "K": #Si se cumple se garantiza existencia de trama hasta "K"
                        C3H8 = valores[3].split("J")[1].split("K")[0].strip()
                    if bit == "L": #Si se cumple se garantiza existencia de trama hasta "L"
                        C4H10 = valores[3].split("K")[1].split("L")[0].strip()
                    if bit == "M": #Si se cumple se garantiza existencia de trama hasta "M"
                        CH4 = valores[3].split("L")[1].split("M")[0].strip()
                    if bit == "N": #Si se cumple se garantiza existencia de trama hasta "N"
                        H2 =  valores[3].split("M")[1].split("N")[0].strip()
                    if bit == "O": #Si se cumple se garantiza existencia de trama hasta "O"
                        C2H50H =  valores[3].split("N")[1].split("O")[0].strip()
                    if bit == "P": #Si se cumple se garantiza existencia de trama hasta "P"
                        tempI2C =  valores[3].split("O")[1].split("P")[0].strip()
                    if bit == "Q": #Si se cumple se garantiza existencia de trama hasta "Q"
                        tempADC =  valores[3].split("P")[1].split("Q")[0].strip()
                        presionbar =  valores[3].split("Q")[1].strip()
                        print("trama larga Gases completa")
            else:
                #trama corta
                #tiempo h latitud / longitud O curso / velocidad A altitud B alturaBar C tempbar D tempeSHT11 E voltajebater
                tramalarga = False
                for i in range(len(valores[3])):
                    i = i + 1
                    bit = valores[3][i-1]
                    if bit == "B": #Si se cumple se garantiza existencia de trama hasta "B"
                        tiempo = valores[1][0:6].strip()
                        latitud = valores[1][7:15].strip()
                        longitud = valores[2][0:9].strip()
                        curso = valores[2][10:].strip()
                        velocidad = valores[3].split("A")[0].strip()
                        altitudGps = valores[3].split("A")[1].split("B")[0].strip()
                        datosTransfor = transformarTrama(latitud,longitud) #Convierte las coordenadas a decimales
                        latitud_geo = datosTransfor[0]
                        longitud_geo = datosTransfor[1]
                        altitud_geo = altitudGps
                        datoToSetOK = True
                        print("tiempo: " + str(float(tiempo)))
                        print("latitud_geo: " + str(float(latitud_geo)))
                        print("longitud_geo: " + str(float(longitud_geo)))
                        print("altitud_geo: " + str(float(altitud_geo)))
                        print("curso: " + str(float(curso)))
                        print("velocidad: " + str(float(velocidad)))
                    if bit == "C": #Si se cumple se garantiza existencia de trama hasta "C"
                        alturaBar = valores[3].split("B")[1].split("C")[0].strip()
                        print("alturaBar: " + str(float(alturaBar)))
                    if bit == "D": #Si se cumple se garantiza existencia de trama hasta "D"
                        tempeBar =  valores[3].split("C")[1].split("D")[0].strip()
                        print("temperaturaBar: " +  str(float(tempeBar)/100))
                    if bit == "E": #Si se cumple se garantiza existencia de trama hasta "E"
                        tempeSHT11 =  valores[3].split("D")[1].split("E")[0].strip()
                        voltajebater =  valores[3].split("E")[1].strip()
                        print("temperaturaSHT11: " +  str(float(tempeSHT11)/100))
                        print("voltajebater: " + str(float(voltajebater)))
                        print("trama corta completa")
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
                print("alturabar: " + str(float(alturaBar)))
                print("temperaturaSHT11: " +  str(float(tempeSHT11)/100))
                print("voltajebater: " + str(float(voltajebater)))
                print("temperaturaBar: " +  str(float(tempeBar)/100))
            except Exception as e:
                print e
                print("***** ERROR EN VARIABLES PROCESADAS *****")

            return [latitud_geo,longitud_geo,altitud_geo]
    except Exception as e:
        print e
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
            coordenadas = procesarTrama(trama, band_init)
            band_init = 1
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
            tiempog = tiempo; latitud_geog = float(latitud_geo); longitud_geog = float(longitud_geo); altitudGpsg = float(altitudGps); cursog = float(curso); velocidadg = float(velocidad)
            ACCXg = float(ACCX) / 100.0; ACCYg = float(ACCY) / 100.0; ACCZg = float(ACCZ) / 100.0; GIXg = float(GIX) / 100.0; GIYg = float(GIY) / 100.0; GIZg = float(GIZ) / 100.0; MXg = float(MX) / 100.0;
            MYg = float(MY) / 100.0; MZg = float(MZ) / 100.0
            tempeBarg = float(tempeBar); presionbarg = float(presionbar); alturaBarg = float(alturaBar)
            NH3g = float(NH3); COg = float(CO); NO2g = float(NO2); C3H8g = float(C3H8); C4H10g = float(C4H10); CH4g = float(CH4); H2g = float(H2); C2H50Hg = float(C2H50H)
            tempeSHT11g = float(tempeSHT11); humedadDHT11g = float(humedadDHT11); tempI2Cg = float(tempI2C); tempADCg = float(tempADC); voltajebater = float(voltajebater)/1000.0
            then = time.time()
            timediff = then - start
            time_mision = timediff

            #Conversion data
            humedadDHT11g = (humedadDHT11g / 100.0)
            tempeSHT11g = (tempeSHT11g / 100.0)
            tempADCg = (tempADCg / 100.0)
            tiempoH = tiempog[0:2]
            tiempoM = tiempog[2:4]
            tiempoS = tiempog[4:6]
            tempI2Cg = (tempI2Cg / 100.0)
            tempeBarg = (tempeBarg / 100.0)
            presionbarg = (presionbarg / 100.0)
        #    tiempog = str(tiempoH+":"+tiempoM+":"+tiempoS)
            print tiempog
            gps_data = 'gps_data gps_time=%s,gps_latitude=%e,gps_longitude=%e,gps_altitude=%e,gps_course=%e,gps_speed=%e' % (tiempoH, latitud_geog, longitud_geog, altitudGpsg, cursog, velocidadg)
            imu_data = 'imu_data imu_ax=%e,imu_ay=%e,imu_az=%e,imu_cx=%e,imu_cy=%e,imu_cz=%e,imu_gx=%e,imu_gy=%e,imu_gz=%e' % (ACCXg, ACCYg, ACCZg, GIXg, GIYg, GIZg, MXg, MYg, MZg)
            barometer_data = 'barometer_data barometer_temperature=%e,barometer_pressure=%e,barometer_altitude=%e' % (tempeBarg, presionbarg, alturaBarg)
            gas_data = 'gas_data gas_nh3=%e,gas_co=%e,gas_no2=%e,gas_c3h8=%e,gas_c4h10=%e,gas_ch4=%e,gas_h2=%e,gas_c2h5oh=%e' % (NH3g, COg, NO2g, C3H8g, C4H10g, CH4g, H2g, C2H50Hg)
            timpogps = 'horagps horagps=%s,minutogps=%s,segundogps=%s' % (tiempoH,tiempoM,tiempoS)
            #gas1_data = 'gas1_data gas1_nh3=%e,gas1_co=%e,gas1_no2=%e,gas1_c3h8=%e,gas1_c4h10=%e,gas1_ch4=%e,gas1_h2=%e,gas1_c2h5oh=%e' % (conseno, seno, voltajePanel1, velocidad, seno*0.2, seno*0.5, conseno*0.2, conseno*0.5)
            environment_data = 'environment_data temperature_sht11=%e,humidity_sht11=%e,temperature_i2c=%e,mean_temperature=%e' % (tempeSHT11g, humedadDHT11g, tempI2Cg, tempADCg)
            other_data = 'other_data time_miliseconds=%e,voltage_batery=%e' % (time_mision, voltajebater)

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

            hora = requests.post("http://%s:8086/write?db=%s" %(IP, DB), auth=(USER, PASSWORD), data=timpogps)
            addToDB(hora)
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
    except Exception as e:
        print e
        print("******** ERROR DE EJECUCION ***********")

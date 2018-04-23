#trama corta
    #tiempo h latitud / longitud O curso / velocidad A altitud B alturaBar C tempbar D tempeSHT11 E voltajebater
#trama larga IMU
    #tiempo h latitud / longitud O curso / velocidad A altitud B alturaBar C tempbar D tempeSHT11 E voltajebater
    #f ACCx G ACCy H ACCz I GIx J GIy K GIz L Mx M My N Mz
#trama larga gases
    #tiempo h latitud / longitud O curso / velocidad A altitud B alturaBar C tempbar D tempeSHT11 E voltajebater
    #F humedadDHT11 G NH3 H CO I NO2 J C3H8 K C4H10 L Ch4 M H2 N C2H50H O tempI2C P tempADC Q presionbar

import sys
import math
import serial
import time
import subprocess
import random
import requests
import string

global band_altitudBAR, band_altitudGPS, MatrizD, vectorTramas, cuentatrama, cuentadesconocida

#La variable nombreArchivoTramasLeer contiene las tramas que el programa gqrx guardo al decodificar el AFSK
#La variable nombreVectorTrayectoriaGuardar es donde se guardan los vectores de trayectoria calculados por el modelo
#La variable nombreArchivoSETLeerEscribir contiene las coordenadas iniciales de la estacion y de la gondola para setear el cero relativo

nombreArchivoTramasLeer = "/home/oscar/Descargas/datosIterativos.txt"
nombreVectorTrayectoriaGuardar = "vectorprueba22abril.txt"
nombreArchivoSETLeerEscribir = "SET.txt"

MatrizD = []
vectorAlturaBar = []
vectorAlturaGps = []
vectorAlturaBar_ext = []
vectorAlturaGps_ext = []
sumaAlturaBar = 0
sumaAlturaGps = 0
sumaVarianzaAlturaBar = 0
sumaVarianzaAlturaGps = 0
vectorTramas = ['/0/0/0']
cuentatrama = 0
cuentadesconocida = 0
cuentaFusion = 0
#arduino = serial.Serial('/dev/ttyUSB0',9600)
time.sleep(2)

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
        print("****ERROR LEYENDO TRAMA*****")
        print(e)
def procesarTrama(lineas,set):
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
                print("trama IMU")
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
                    if bit == "C": #Si se cumple se garantiza existencia de trama hasta "C"
                        alturaBar = valores[3].split("B")[1].split("C")[0].strip()
                        vectorAlturaBar_ext.append(alturaBar)
                        vectorAlturaGps_ext.append(altitudGps)
                        print("tiempo: " + str(float(tiempo)))
                        print("latitud_geo: " + str(float(latitud_geo)))
                        print("longitud_geo: " + str(float(longitud_geo)))
                        print("altitud_Gps: " + str(float(altitudGps)))
                        print("altura_Bar: " + str(float(alturaBar)))
                        if len(vectorAlturaBar_ext) > 1:
                            tamaño = len(vectorAlturaBar_ext)
                            deltaAlturaBar = float(vectorAlturaBar_ext[tamaño-1])-float(vectorAlturaBar_ext[tamaño-2])
                            alturaBarBias = float(vectorAlturaGps_ext[tamaño-2]) + deltaAlturaBar
                            altitudFus= fusionar(altitudGps,alturaBarBias) #Fusiona la altura del GPS y Barometro en una sola medida mas precisa
                            print("altura_FUS: " + str(float(altitudFus)))
                        datoToSetOK = True
                        print("curso: " + str(float(curso)))
                        print("velocidad: " + str(float(velocidad)))
                    if bit == "D": #Si se cumple se garantiza existencia de trama hasta "D"
                        tempeBar =  valores[3].split("C")[1].split("D")[0].strip()
                        print("temperaturaBar: " +  str(float(tempeBar)/100))
                    if bit == "E": #Si se cumple se garantiza existencia de trama hasta "E"
                        tempeSHT11 =  valores[3].split("D")[1].split("E")[0].strip()
                        print("temperaturaSHT11: " +  str(float(tempeSHT11)/100))
                    if bit == "f": #Si se cumple se garantiza existencia de trama hasta "f"
                        voltajebater =  valores[3].split("E")[1].split("f")[0].strip()
                        print("voltajebater: " + str(float(voltajebater)/1000))
                    if bit == "G": #Si se cumple se garantiza existencia de trama hasta "G"
                        ACCX =  valores[3].split("f")[1].split("G")[0].strip()
                        print("ACCX: " + str(float(ACCX)*9.8/100))
                    if bit == "H": #Si se cumple se garantiza existencia de trama hasta "H"
                        ACCY =  valores[3].split("G")[1].split("H")[0].strip()
                        print("ACCY: " + str(float(ACCY)*9.8/100))
                    if bit == "I": #Si se cumple se garantiza existencia de trama hasta "I"
                        ACCZ =  valores[3].split("H")[1].split("I")[0].strip()
                        print("ACCZ: " + str(float(ACCZ)*9.8/100))
                    if bit == "J": #Si se cumple se garantiza existencia de trama hasta "J"
                        GIX =  valores[3].split("I")[1].split("J")[0].strip()
                        print("GIX: " + GIX)
                    if bit == "K": #Si se cumple se garantiza existencia de trama hasta "K"
                        GIY =  valores[3].split("J")[1].split("K")[0].strip()
                        print("GIY: " + GIY)
                    if bit == "L": #Si se cumple se garantiza existencia de trama hasta "L"
                        GIZ =  valores[3].split("K")[1].split("L")[0].strip()
                        print("GIZ: " + GIZ)
                    if bit == "M": #Si se cumple se garantiza existencia de trama hasta "M"
                        MX =  valores[3].split("L")[1].split("M")[0].strip()
                        print("MX: " + MX)
                    if bit == "N": #Si se cumple se garantiza existencia de trama hasta "N"
                        MY =  valores[3].split("M")[1].split("N")[0].strip()
                        MZ =  valores[3].split("N")[1].strip()
                        print("MY: " + MY)
                        print("MZ: " + MZ)
                        print("trama larga IMU completa")
            elif tramaGases == True:
                #trama larga gases
                #tiempo h latitud / longitud O curso / velocidad A altitud B alturaBar C tempbar D tempeSHT11 E voltajebater
                #F humedadDHT11 G NH3 H CO I NO2 J C3H8 K C4H10 L Ch4 M H2 N C2H50H O tempI2C P tempADC Q presionbar
                print("trama Gases")
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
                    if bit == "C": #Si se cumple se garantiza existencia de trama hasta "C"
                        alturaBar = valores[3].split("B")[1].split("C")[0].strip()
                        vectorAlturaBar_ext.append(alturaBar)
                        vectorAlturaGps_ext.append(altitudGps)
                        print("tiempo: " + str(float(tiempo)))
                        print("latitud_geo: " + str(float(latitud_geo)))
                        print("longitud_geo: " + str(float(longitud_geo)))
                        print("altitud_Gps: " + str(float(altitudGps)))
                        print("altura_Bar: " + str(float(alturaBar)))
                        if len(vectorAlturaBar_ext) > 1:
                            tamaño = len(vectorAlturaBar_ext)
                            deltaAlturaBar = float(vectorAlturaBar_ext[tamaño-1])-float(vectorAlturaBar_ext[tamaño-2])
                            alturaBarBias = float(vectorAlturaGps_ext[tamaño-2]) + deltaAlturaBar
                            altitudFus= fusionar(altitudGps,alturaBarBias) #Fusiona la altura del GPS y Barometro en una sola medida mas precisa
                            print("altura_FUS: " + str(float(altitudFus)))
                        datoToSetOK = True
                        print("curso: " + str(float(curso)))
                        print("velocidad: " + str(float(velocidad)))
                    if bit == "D": #Si se cumple se garantiza existencia de trama hasta "D"
                        tempeBar =  valores[3].split("C")[1].split("D")[0].strip()
                        print("temperaturaBar: " +  str(float(tempeBar)/100))
                    if bit == "E": #Si se cumple se garantiza existencia de trama hasta "E"
                        tempeSHT11 =  valores[3].split("D")[1].split("E")[0].strip()
                        print("temperaturaSHT11: " +  str(float(tempeSHT11)/100))
                    if bit == "F": #Si se cumple se garantiza existencia de trama hasta "F"
                        voltajebater =  valores[3].split("E")[1].split("F")[0].strip()
                        print("voltajebater: " + str(float(voltajebater)/1000))
                    if bit == "G": #Si se cumple se garantiza existencia de trama hasta "G"
                        humedadDHT11 =  valores[3].split("F")[1].split("G")[0].strip()
                        print("humedadDHT11: " + str(humedadDHT11))
                    if bit == "H": #Si se cumple se garantiza existencia de trama hasta "H"
                        NH3 =  valores[3].split("G")[1].split("H")[0].strip()
                        print("NH3: " + NH3)
                    if bit == "I": #Si se cumple se garantiza existencia de trama hasta "I"
                        CO =  valores[3].split("H")[1].split("I")[0].strip()
                        print("CO: " + CO)
                    if bit == "J": #Si se cumple se garantiza existencia de trama hasta "J"
                        NO2 =  valores[3].split("I")[1].split("J")[0].strip()
                        print("NO2: " + NO2)
                    if bit == "K": #Si se cumple se garantiza existencia de trama hasta "K"
                        C3H8 = valores[3].split("J")[1].split("K")[0].strip()
                        print("C3H8: " + C3H8)
                    if bit == "L": #Si se cumple se garantiza existencia de trama hasta "L"
                        C4H10 = valores[3].split("K")[1].split("L")[0].strip()
                        print("C4H10: " + C4H10)
                    if bit == "M": #Si se cumple se garantiza existencia de trama hasta "M"
                        CH4 = valores[3].split("L")[1].split("M")[0].strip()
                        print("CH4: " + CH4)
                    if bit == "N": #Si se cumple se garantiza existencia de trama hasta "N"
                        H2 =  valores[3].split("M")[1].split("N")[0].strip()
                        print("H2: " + H2)
                    if bit == "O": #Si se cumple se garantiza existencia de trama hasta "O"
                        C2H50H =  valores[3].split("N")[1].split("O")[0].strip()
                        print("C2H50H: " + C2H50H)
                    if bit == "P": #Si se cumple se garantiza existencia de trama hasta "P"
                        tempI2C =  valores[3].split("O")[1].split("P")[0].strip()
                        print("tempI2C: " + tempI2C)
                    if bit == "Q": #Si se cumple se garantiza existencia de trama hasta "Q"
                        tempADC =  valores[3].split("P")[1].split("Q")[0].strip()
                        presionbar =  valores[3].split("Q")[1].strip()
                        print("tempADC: " + tempADC)
                        print("presionbar: " + presionbar)
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
                    if bit == "C": #Si se cumple se garantiza existencia de trama hasta "C"
                        alturaBar = valores[3].split("B")[1].split("C")[0].strip()
                        vectorAlturaBar_ext.append(alturaBar)
                        vectorAlturaGps_ext.append(altitudGps)
                        print("tiempo: " + str(float(tiempo)))
                        print("latitud_geo: " + str(float(latitud_geo)))
                        print("longitud_geo: " + str(float(longitud_geo)))
                        print("altitud_Gps: " + str(float(altitudGps)))
                        print("altura_Bar: " + str(float(alturaBar)))
                        if len(vectorAlturaBar_ext) > 1:
                            tamaño = len(vectorAlturaBar_ext)
                            deltaAlturaBar = float(vectorAlturaBar_ext[tamaño-1])-float(vectorAlturaBar_ext[tamaño-2])
                            alturaBarBias = float(vectorAlturaGps_ext[tamaño-2]) + deltaAlturaBar
                            altitudFus= fusionar(altitudGps,alturaBarBias) #Fusiona la altura del GPS y Barometro en una sola medida mas precisa
                            print("altura_FUS: " + str(float(altitudFus)))
                        datoToSetOK = True
                        print("curso: " + str(float(curso)))
                        print("velocidad: " + str(float(velocidad)))
                    if bit == "D": #Si se cumple se garantiza existencia de trama hasta "D"
                        tempeBar =  valores[3].split("C")[1].split("D")[0].strip()
                        print("temperaturaBar: " +  str(float(tempeBar)/100))
                    if bit == "E": #Si se cumple se garantiza existencia de trama hasta "E"
                        tempeSHT11 =  valores[3].split("D")[1].split("E")[0].strip()
                        voltajebater =  valores[3].split("E")[1].strip()
                        print("temperaturaSHT11: " +  str(float(tempeSHT11)/100))
                        print("voltajebater: " + str(float(voltajebater)/1000))
                        print("trama corta completa")
                        enviarWeb(tiempo,latitud_geo,longitud_geo,altitudGps,curso,velocidad,alturaBar,tempeSHT11,voltajebater,tempeBar)
            #Se debe guardar de manera recurrente las coordenadas de la estacion terrena y la gondola
            #para el caso en el que se reinicie la aplicacion y no se haya movido la estacion, poder
            #recuperar el tracking seteando nuevamente de manera automatica
            if datoToSetOK == True and set == True:
                datoToSetOK = False
                archivo3 = open(nombreArchivoSETLeerEscribir,"w")
                archivo3.write("latitudE/" + str(latitudE) + "\n")
                archivo3.write("longitudE/" + str(longitudE) + "\n")
                archivo3.write("altitudE/" + str(altitudE) + "\n")
                archivo3.write("latitudG_i/" + str(latitud_geo) + "\n")
                archivo3.write("longitudG_i/" + str(longitud_geo) + "\n")
                archivo3.write("altitudG_i/" + str(alturaBar) + "\n")
                archivo3.close()
            return [latitud_geo,longitud_geo,alturaBar]
    except Exception as e:
        print("******ERROR PROCESANDO TRAMA********")
        print(e)
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
    except Exception as e:
        print("*****ERROR TRANSFORMANDO TRAMA*****")
        print(e)
def enviarWeb(tempo,lati,longi,altu,curs,velo,altuba,tempera,volta,temperabar):
    try:
        tempoH=tempo[0:2]
        tempoM=tempo[2:4]
        tempoS=tempo[4:6]
        tempo=tempoH+":"+tempoM+":"+tempoS
        band_altitudGPS = 0
        band_altitudBAR = 0
        offset_GPS = 0
        offset_BAR = 0
        curs=str(float(curs)-0.5)
        if float(velo)>0.5:
            velo=str(float(velo)-0.5)
        volta=str(float(volta)/1000)
        tempera = str(float(tempera)/100)
        if(band_altitudBAR == 0):
            offset_BAR = float(altuba)
            band_altitudBAR = 1
        altuba = str(float(altuba) - offset_BAR)
        if(band_altitudGPS == 0):
            offset_GPS = float(altu)
            band_altitudGPS = 1
        altu = str(float(altu) - offset_GPS)
        temperabar = str(float(temperabar)/100)
        mqttmsg='"gps_time":"{}",' \
                '"gps_latitude":"{}",' \
                '"gps_longitude":"{}",' \
                '"gps_altitude":"{}",' \
                '"gps_course":"{}",' \
                '"gps_speed":"{}",' \
                '"barometer_Altitude":"{}",' \
                '"temperature_sht11":"{}",' \
                '"voltaje_bateria":"{}",'\
                '"barometer_temperature":"{}"'.format(tempo,lati,longi,altu,curs,velo,altuba,tempera,volta,temperabar)
        mqttmsg = "{" + mqttmsg + "}"
        #print(mqttmsg)
        r = requests.post("http://www.cansats3kratos.me/data/", data=mqttmsg,headers = {'content-type':'application/json'})
        #print (r.status_code)
        #print (r.headers)
        print ("trama enviada a web")
    except Exception as e:
        print("*******ERROR ENVIANDO A WEB******")
        print(e)
def modelo(lati, longi, alti):
    try:
        latitudG = float(lati)
        longitudG = float(longi)
        altitudG = float(alti)
        alfaP = (3.141592653589793/180)*(latitudE)
        betaP = (3.141592653589793/180)*(longitudE)
        hP = altitudE
        alfaS = (3.141592653589793/180)*(latitudG)
        betaS = (3.141592653589793/180)*(longitudG)
        hS = altitudG
        divCero = 0
        r = 6378000.0
        P = r + hP
        S = r + hS
        Px = P*math.cos(alfaP)*math.sin(betaP)
        Py = P*math.cos(alfaP)*math.cos(betaP)
        Pz = P*math.sin(alfaP)
        Sx = S*math.cos(alfaS)*math.sin(betaS)
        Sy = S*math.cos(alfaS)*math.cos(betaS)
        Sz = S*math.sin(alfaS)
        Dxyz_1 = Sx-Px
        Dxyz_2 = Sy-Py
        Dxyz_3 = Sz-Pz
        D = math.sqrt(Dxyz_1*Dxyz_1+Dxyz_2*Dxyz_2+Dxyz_3*Dxyz_3)
        Dw_1 = math.cos(-betaP)*Dxyz_1+math.sin(-betaP)*Dxyz_2
        Dw_2 = -math.sin(-betaP)*Dxyz_1+math.cos(betaP)*Dxyz_2
        Dw_3 = Dxyz_3
        Duvw_1 = Dw_1
        Duvw_2 = math.cos(alfaP)*Dw_2+math.sin(alfaP)*Dw_3
        Duvw_3 = -math.sin(alfaP)*Dw_2+math.cos(alfaP)*Dw_3
        vectorD = [Duvw_1,Duvw_2,Duvw_3]
        MatrizD.append(vectorD)
        archivo1 = open(nombreVectorTrayectoriaGuardar,"a")
        archivo1.write(str(vectorD))
        archivo1.close()
        if abs(Duvw_1) < 1:
            Duvw_1 = 0
            divCero = divCero + 1
        if abs(Duvw_2) < 1:
            Duvw_2 = 0
            divCero = divCero + 1
        if abs(Duvw_3) < 1:
            Duvw_3 = 0
            divCero = divCero + 1
        Distancia = math.sqrt(Duvw_1*Duvw_1+Duvw_2*Duvw_2+Duvw_3*Duvw_3)
        if Distancia > 0:
            if divCero < 3:
                divCero = 0
                omega_prima = math.asin(Duvw_2/Distancia)*180/3.14159265358979
            if Duvw_1 < 0:
                theta_prima = -math.acos(Duvw_3/math.sqrt(Duvw_1*Duvw_1+Duvw_3*Duvw_3))*180/3.141592653589793
            else:
                theta_prima = math.acos(Duvw_3/math.sqrt(Duvw_1*Duvw_1+Duvw_3*Duvw_3))*180/3.141592653589793
            angulo_omega = omega_prima
            angulo_theta = theta_prima
            return [angulo_theta, angulo_omega]
        else:
            return[0.0, 0.0]
    except Exception as e:
        print("*****ERROR CALCULANDO MODELO****")
        print(e)
def enviarArduino(angulo_theta, angulo_omega, state):
    try:
        SET = state
        #Python le pregunta al arduino si puede recibir datos
        arduino.write(str("ESTALISTO0000090909").encode())
        print("arduino listo?")
        time.sleep(0.1)
        datoRecibido = ''
        while True:
            datoRecibido = str(arduino.readline().decode().splitlines())
            if datoRecibido == "['listo']":
                print(datoRecibido)
                break
            time.sleep(0.01)
        #Solo cuando el arduino responda el programa continua
        datoRecibido = ''

        omega_prima = angulo_omega
        theta_prima = angulo_theta

        #Se hace un acarreo de los datos a enviar al arduino y garantizar que no hay perdida de datos
        if omega_prima < 0:
            signo_omega = "-"
            acarreo_signo_omega = 45
        else:
            signo_omega = "+"
            acarreo_signo_omega = 43

        if theta_prima < 0:
            signo_theta = "-"
            acarreo_signo_theta = 45
        else:
            signo_theta = "+"
            acarreo_signo_theta= 43

        suma = (acarreo_signo_theta
            + acarreo_signo_omega
            + 84 + 79
            + int(abs(theta_prima*100))
            + int(abs(omega_prima*100)))
        if suma > 10000:
            ajusteSum = "";
        if 1000 < suma and suma < 10000:
            ajusteSum = "0";
        if 100 < suma and suma < 1000:
            ajusteSum = "00";
        if 10 < suma and suma < 100:
            ajusteSum = "000";
        if suma < 10:
            ajusteSum = "0000";

        print("theta_prima: " + str(theta_prima))
        print("omega_prima: " + str(omega_prima))

        if abs(theta_prima) > 100:
            ajusteT = ""
        if 10 < abs(theta_prima) and abs(theta_prima) < 100:
            ajusteT = "0"
        if 1 < abs(theta_prima) and abs(theta_prima) < 10:
            ajusteT = "00"
        if 0.1 < abs(theta_prima) and abs(theta_prima) < 1:
            ajusteT = "000"
        if abs(theta_prima) < 0.1:
            ajusteT = "0000"

        if abs(omega_prima) > 100:
            ajusteO = "";
        if 10 < abs(omega_prima) and abs(omega_prima) < 100:
            ajusteO = "0";
        if 1 < abs(omega_prima) and abs(omega_prima) < 10:
            ajusteO = "00";
        if 0.1 < abs(omega_prima) and abs(omega_prima) < 1:
            ajusteO = "000";
        if abs(omega_prima) < 0.1:
            ajusteO = "0000";

        #Si SET es true el arduino comprendera que las coordenadas enviadas son el SET y alli configurara su posicion inicial
        if SET == False:
            print(str("T").encode() + str(signo_theta).encode() + str(ajusteT).encode() + str(int(abs(theta_prima*100))).encode()
                  + str("O").encode() + str(signo_omega).encode() + str(ajusteO).encode() + str(int(abs(omega_prima*100))).encode()
                  + str(ajusteSum).encode() + str(suma).encode())
            arduino.write(str("T").encode() + str(signo_theta).encode() + str(ajusteT).encode() + str(int(abs(theta_prima*100))).encode()
                  + str("O").encode() + str(signo_omega).encode() + str(ajusteO).encode() + str(int(abs(omega_prima*100))).encode()
                  + str(ajusteSum).encode() + str(suma).encode())
        else:
            print(str("T").encode() + str(signo_theta).encode() + str(ajusteT).encode() + str(int(abs(theta_prima*100))).encode()
                  + str("O").encode() + str(signo_omega).encode() + str(ajusteO).encode() + str(int(abs(omega_prima*100))).encode()
                  + str("99999").encode())
            arduino.write(str("T").encode() + str(signo_theta).encode() + str(ajusteT).encode() + str(int(abs(theta_prima*100))).encode()
                  + str("O").encode() + str(signo_omega).encode() + str(ajusteO).encode() + str(int(abs(omega_prima*100))).encode()
                  + str("99999").encode())
        time.sleep(0.1)

        datoRecibido = ''
        #Python le pregunta al arduino si entendio el dato que le fue enviado
        while True:
            datoRecibido = str(arduino.readline().decode().splitlines())
            print("angulo entendido? ")
            if datoRecibido == "['entendido']":
                print(datoRecibido)
                break
            if datoRecibido == "['SET']":
                print(datoRecibido)
                break
            time.sleep(0.01)
        #Si el arduino responde el programa continua
        datoRecibido = ''
    except Exception as e:
        print("******ERROR ENVIANDO A ARDUINO*****")
        print(e)
def estimar(DuAnt,DvAnt,DwAnt,DuNue,DvNue,DwNue):
    Cu = DuNue - DuAnt
    Cv = DvNue - DvAnt
    Cw = DwNue - DwAnt
    P_1u = (1/2)*Cu + DuNue
    P_1v = (1/2)*Cv + DvNue
    P_1w = (1/2)*Cw + DwNue
    P_2u = Cu + DuNue
    P_2v = Cv + DvNue
    P_2w = Cw + DwNue
    return [P_1u,P_1v,P_1w,P_2u,P_2v,P_2w]
def modeloVector(Du,Dv,Dw):
    divCero = 0
    Distancia = math.sqrt(Du*Du+Dv*Dv+Dw*Dw)
    if abs(Du) < 1:
        Du = 0
        divCero = divCero + 1
    if abs(Dv) < 1:
        Dv = 0
        divCero = divCero + 1
    if abs(Dw) < 1:
        Dw = 0
        divCero = divCero + 1
    if divCero < 3:
        divCero = 0
        omega_prima_estimada = math.asin(Dv/Distancia)*180/3.14159265358979
    if Du < 0:
        theta_prima_estimada = -math.acos(Dw/math.sqrt(Du*Du+Dw*Dw))*180/3.141592653589793
    else:
        theta_prima_estimada = math.acos(Dw/math.sqrt(Du*Du+Dw*Dw))*180/3.141592653589793
    return [theta_prima_estimada,omega_prima_estimada]
def fusionar(AltG,AltB):
    try:
        global vectorAlturaGps, vectorAlturaBar, sumaAlturaBar, sumaAlturaGps, sumaVarianzaAlturaBar, sumaVarianzaAlturaGps, cuentaFusion
        vectorAlturaGps.append(AltG)
        vectorAlturaBar.append(AltB)
        vectorAlturaGps_float = [float(i) for i in vectorAlturaGps]
        vectorAlturaBar_float = [float(i) for i in vectorAlturaBar]
        k = j = 0
        if len(vectorAlturaBar) == 1 and len(vectorAlturaGps) == 1:
            alturaFus = 0.5*float(AltG) + 0.5*float(AltB)
        elif len(vectorAlturaBar) > 1 and len(vectorAlturaGps) > 1:
            for suma in vectorAlturaBar_float:
                sumaAlturaBar += suma
            promedioAlturaBar = sumaAlturaBar/len(vectorAlturaBar)
            for n in range(len(vectorAlturaBar)):
                sumaVarianzaAlturaBar += (float(vectorAlturaBar[j])-promedioAlturaBar)*(float(vectorAlturaBar[j])-promedioAlturaBar)
                j = j + 1
            varianzaAlturaBar = sumaVarianzaAlturaBar/len(vectorAlturaBar)
            for suma in vectorAlturaGps_float:
                sumaAlturaGps += suma
            promedioAlturaGps = sumaAlturaGps/len(vectorAlturaGps)
            for n in range(len(vectorAlturaGps)):
                sumaVarianzaAlturaGps += (float(vectorAlturaGps[k])-promedioAlturaGps)*(float(vectorAlturaGps[k])-promedioAlturaGps)
                k = k + 1
            varianzaAlturaGps = sumaVarianzaAlturaGps/len(vectorAlturaGps)

            if varianzaAlturaBar != 0 or varianzaAlturaGps != 0:
                pesoAlturaGps = varianzaAlturaBar/(varianzaAlturaBar + varianzaAlturaGps)
                pesoAlturaBar = 1 - pesoAlturaGps
            else:
                pesoAlturaGps = 0.5
                pesoAlturaBar = 0.5
            alturaFus = pesoAlturaBar*float(AltB) + pesoAlturaGps*float(AltG)
        if len(vectorAlturaGps) > 5 or len(vectorAlturaBar) > 5:
            vectorAlturaGps = []
            vectorAlturaBar = []
            sumaAlturaBar = 0
            sumaAlturaGps = 0
            sumaVarianzaAlturaBar = 0
            sumaVarianzaAlturaGps = 0
        cuentaFusion = 1
        return str(alturaFus)
    except Exception as e:
        print("*****  ERROR FUSIONANDO DATOS  *****")
        print(e)

try:
    continuar = 0
    while(1):
        trama = leerTrama()
        if trama != 0 and trama != 1:
            print(trama)
            for line in sys.stdin:
                if str(line).encode() == b'\n':
                    break
                if line.strip() == "SET-ESTACION":
                    coordenadas = procesarTrama(trama,False)
                    latitudE_set = coordenadas[0]
                    longitudE_set = coordenadas[1]
                    altitudE_set = coordenadas[2]
                    print("**** SET-ESTACION OK ****")
                    continuar = continuar + 1
                    break
                if line.strip() == "SET-GONDOLA":
                    coordenadas = procesarTrama(trama,False)
                    latitudG_i_set = coordenadas[0]
                    longitudG_i_set = coordenadas[1]
                    altitudG_i_set = coordenadas[2]
                    print("**** SET-GONDOLA OK ****")
                    continuar = continuar + 1
                    break
        if continuar == 2:
            archivo3 = open(nombreArchivoSETLeerEscribir,"w")
            archivo3.write("latitudE/" + str(latitudE_set) + "\n")
            archivo3.write("longitudE/" + str(longitudE_set) + "\n")
            archivo3.write("altitudE/" + str(altitudE_set) + "\n")
            archivo3.write("latitudG_i/" + str(latitudG_i_set) + "\n")
            archivo3.write("longitudG_i/" + str(longitudG_i_set) + "\n")
            archivo3.write("altitudG_i/" + str(altitudG_i_set) + "\n")
            archivo3.close()
            break
            print("_________________")
        time.sleep(0.1)


    '''archivo2 = open(nombreArchivoSETLeerEscribir,"r")
    datoSET = archivo2.readlines()
    latitudE = float(datoSET[0].split("/")[1])
    longitudE = float(datoSET[1].split("/")[1])
    altitudE = float(datoSET[2].split("/")[1])
    latitudG_i = float(datoSET[3].split("/")[1])
    longitudG_i = float(datoSET[4].split("/")[1])
    altitudG_i = float(datoSET[5].split("/")[1])
    angulosSET = modelo(latitudG_i,longitudG_i,altitudG_i)
    thetaSET = angulosSET[0]
    omegaSET = angulosSET[1]
    archivo2.close()
    enviarArduino(thetaSET,omegaSET,True)'''
    print("__________________________________________")
except Exception as e:
    while(1):
        print(e)
        print("****ERROR GENERANDO SET***")
        time.sleep(1)

time.sleep(2)

while (1):
    try:
        trama = leerTrama()
        if trama != 0 and trama != 1:
            cuentatrama = 0
            cuentadesconocida = 0
            coordenadas = procesarTrama(trama,True)
            angulos = modelo(coordenadas[0],coordenadas[1],coordenadas[2])
            #enviarArduino(angulos[0],angulos[1],False) #theta,omega
            print("__________________________________________")
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
    except:
        print("******** ERROR DE EJECUCION ***********")

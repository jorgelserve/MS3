#/142032h0611.90N/07534.73WO048/000A1534T310P19596A1444V0
    #tiempo h latitud / longitud O curso / velocidad A altitud T temperatura P presionbar A alturabar V voltajebater
#/142041h0611.90N/07534.73WO048/000A1534B514C4115D20E26104F6913G-7520H2228I3091J-3745K634L3200M8800
    #tiempo h latitud / longitud O curso / velocidad A altitud B NH3 C co D NO2 E CEH8 F C4H10 G CH4 H H2 I C2H5OH J TEMPSHT11 K HUMESHT11 L TEMPPCB M TEMPADC

import math
import serial
import time
import subprocess
import random
import requests

nombreArchivo = "datosMarzo20.txt"
latitudE = 6.1985
longitudE = -75
altitudE = 0
arduino = serial.Serial('/dev/ttyUSB0',9600)
time.sleep(2)

cont = 0
conteoTEMP = 0
global MatrizD
MatrizD = []

def leerProcesarTrama():
    archivo = open(nombreArchivo,"r")
    lineas=archivo.readlines()
    ultimalinea=len(lineas)-1
    returnedValues=lineas[ultimalinea]
    if (returnedValues != 'null'):
        print(returnedValues)
        tramalarga=False
        valores = returnedValues.split("/")
        if len(valores[3]) > 50:
            #trama larga
            print("trama larga")
            tramalarga=True
            try:
                tiempo = valores[1][0:6]
                latitud = valores[1][7:15]
                longitud = valores[2][0:9]
                curso = valores[2][10:]
                velocidad = valores[3].split("A")[0]
                altitud = valores[3].split("A")[1].split("B")[0]
                NH3 = valores[3].split("B")[1].split("C")[0]
                CO =  valores[3].split("C")[1].split("D")[0]
                NO2 =  valores[3].split("D")[1].split("E")[0]
                C3H8 =  valores[3].split("E")[1].split("F")[0]
                C4H10 =  valores[3].split("F")[1].split("G")[0]
                CH4 =  valores[3].split("G")[1].split("H")[0]
                H2 =  valores[3].split("H")[1].split("I")[0]
                C2H50H =  valores[3].split("I")[1].split("J")[0]
                TEMPSHT11 =  valores[3].split("J")[1].split("K")[0]
                HUMESHT11 =  valores[3].split("K")[1].split("L")[0]
                TEMPPCB =  valores[3].split("L")[1].split("M")[0]
                TEMPADC =  valores[3].split("M")[1]
            except:
                print("***** error inesperado decodificacion ******")
        else:
            #trama corta
            print("trama corta")
            try:
                tiempo = valores[1][0:6]
                latitud = valores[1][7:15]
                longitud = valores[2][0:9]
                curso = valores[2][10:]
                velocidad = valores[3].split("A")[0]
                altitud = valores[3].split("A")[1].split("T")[0]
                temperatura = valores[3].split("T")[1].split("P")[0]
                presionbar = valores[3].split("P")[1].split("A")[0]
                alturabar = valores[3].split("A")[2].split("V")[0]
                voltajebater = valores[3].split("V")[1]
            except:
                print("***** error inesperado decodificacion ******")

        if ( not tramalarga):
            tiempo = tiempo.strip()
            latitud = latitud.strip()
            longitud = longitud.strip()
            curso = curso.strip()
            velocidad = velocidad.strip()
            altitud = altitud.strip()
            presionbar = presionbar.strip()
            alturabar = alturabar.strip()
            temperatura = temperatura.strip()
            voltajebater = voltajebater.strip()
            print("Mandando trama corta")
            print("tiempo: " + tiempo)
            print("latitud: " + latitud)
            print("longitud: " + longitud)
            print("curso: " + curso)
            print("velocidad: " + velocidad)
            print("altitud: " + altitud)
            print("presionbar: " + presionbar)
            print("alturabar: " + alturabar)
            print("temperatura: " +  temperatura)
            print("voltajebater: " + voltajebater)
            longitud_geo = 0
            latitud_geo = 0
            try:
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
            except:
                print("******* error inesperado extrayendo coordenadas ******")
            print("latitud_geo: " + latitud_geo)
            print("longitud_geo: " + longitud_geo)
            print("altitud_geo: " + altitud)
        return [latitud_geo,longitud_geo,altitud]
def enviarWeb():
    try:
        mqttmsg='"gps_time":"{}",' \
                '"gps_latitude":"{}",' \
                '"gps_longitude":"{}",' \
                '"gps_altitude":"{}",' \
                '"gps_course":"{}",' \
                '"gps_speed":"{}",' \
                '"barometer_Pressure":"{}",' \
                '"barometer_Altitude":"{}",' \
                '"temperature_sht11":"{}",' \
                '"voltaje_bateria":"{}"'.format(tiempo,latitud_geo,longitud_geo,altitud,curso,velocidad,presionbar,alturabar,temperatura,voltajebater)
        mqttmsg = "{" + mqttmsg + "}"
        print(mqttmsg)
        r = requests.post("http://www.cansats3kratos.me/data/", data=mqttmsg,headers = {'content-type':'application/json'})
        print (r.status_code)
        print (r.headers)
        print ("trama corta enviada")
    except:
        print("******* error, trama no enviada ******")
def modelo(latitud, longitud, altitud):
    latitudG = float(latitud)
    longitudG = float(longitud)
    altitudG = float(altitud)
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
    Distancia = math.sqrt(Duvw_1*Duvw_1+Duvw_2*Duvw_2+Duvw_3*Duvw_3)
    vectorD = [Duvw_1,Duvw_2,Duvw_3]
    MatrizD.append(vectorD)
    archivo1 = open("matriz.txt","w")
    archivo1.write(str(MatrizD))
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
def enviarArduino(angulo_theta, angulo_omega):

    arduino.write(str("ESTALISTO0000000000").encode())
    print("arduino listo?")
    time.sleep(0.1)
    datoRecibido = ''
    while True:
        datoRecibido = str(arduino.readline().decode().splitlines())
        if datoRecibido == "['listo']":
            print(datoRecibido)
            break
        time.sleep(0.01)
    datoRecibido = ''

    omega_prima = angulo_omega
    theta_prima = angulo_theta
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

    print(str("T").encode() + str(signo_theta).encode() + str(ajusteT).encode() + str(int(abs(theta_prima*100))).encode()
          + str("O").encode() + str(signo_omega).encode() + str(ajusteO).encode() + str(int(abs(omega_prima*100))).encode()
          + str(ajusteSum).encode() + str(suma).encode())

    arduino.write(str("T").encode() + str(signo_theta).encode() + str(ajusteT).encode() + str(int(abs(theta_prima*100))).encode()
          + str("O").encode() + str(signo_omega).encode() + str(ajusteO).encode() + str(int(abs(omega_prima*100))).encode()
          + str(ajusteSum).encode() + str(suma).encode())
    time.sleep(0.1)
    datoRecibido = ''
    while True:
        datoRecibido = str(arduino.readline().decode().splitlines())
        print("angulo entendido? ")
        if datoRecibido == "['entendido']":
            print(datoRecibido)
            break
        time.sleep(0.01)
    datoRecibido = ''
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
def estimacion():
    DuAnt = MatrizD[len(MatrizD)-2][0]
    DvAnt = MatrizD[len(MatrizD)-2][1]
    DwAnt = MatrizD[len(MatrizD)-2][2]
    DuNue = MatrizD[len(MatrizD)-1][0]
    DvNue = MatrizD[len(MatrizD)-1][1]
    DwNue = MatrizD[len(MatrizD)-1][2]
    coordEstim = estimar(DuAnt,DvAnt,DwAnt,DuNue,DvNue,DwNue)

    angulosEstim_v1 = modeloVector(coordEstim[0],coordEstim[1],coordEstim[2])
    enviarArduino(angulosEstim_v1[0],angulosEstim_v1[1])
    time.sleep(3)

    angulosEstim_v2 = modeloVector(coordEstim[3],coordEstim[4],coordEstim[5])
    enviarArduino(angulosEstim_v2[0],angulosEstim_v1[1])
    time.sleep(3)

while (1):
    coordenadas = leerProcesarTrama()
    '''if conteoTEMP == 0:
        coordenadas = [10.999641,-74.811614,0]
    if conteoTEMP == 1:
        coordenadas = [10.993981,-74.808701,0]
    if conteoTEMP == 2:
        coordenadas = [10.987427,-74.811614,0]
    if conteoTEMP == 3:
        coordenadas = [10.984865,-74.818290,0]
    if conteoTEMP == 4:
        coordenadas = [10.988619,-74.825269,0]
    if conteoTEMP == 5:
        coordenadas = [10.993683,-74.827151,0]
    if conteoTEMP == 6:
        break'''
    #enviarWeb()
    angulos = modelo(coordenadas[0],coordenadas[1],coordenadas[2])
    enviarArduino(angulos[0],angulos[1]) #theta,omega
    if len(MatrizD) > 1: #Estimacion
        #estimacion()
        time.sleep(6) #quitar si se hace estimacion
    else:
        time.sleep(6)
    print("__________________________________________")
    conteoTEMP = conteoTEMP + 1
    time.sleep(1)

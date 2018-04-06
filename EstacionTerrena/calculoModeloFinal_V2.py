#/142032h0611.90N/07534.73WO048/000A1534T310P19596A1444V0
    #tiempo h latitud / longitud O curso / velocidad A altitud B presionbar C alturabar D voltajebater E TEMPSHT11
#/142041h0611.90N/07534.73WO048/000A1534B514C4115D20E26104F6913G-7520H2228I3091J-3745K634L3200M8800
    #tiempo h latitud / longitud O curso / velocidad A altitud B presionbar C alturabar D voltajebater E TEMPSHT11 F
    #HUMESHT11 G NH3 H co I NO2 J C3H8 K C4H10 L CH4 M H2 N C2H5OH O TEMPPCB P TEMPADC

import math
import serial
import time
import subprocess
import random
import requests

nombreArchivoLeer = "PruebaEstaticaJ-180329_1.txt"
nombreVectorGuardar = "vectorPruebaEstaticaJ-180329_1.txt"
latitudE = 6.1988333
longitudE = -75.57933
altitudE = 1538
latitudG_1 = 6.20000
longitudG_1 = -76.00000
altitudG_1 = 1538
arduino = serial.Serial('/dev/ttyACM0',9600)
time.sleep(2)

cont = 0
global MatrizD
MatrizD = []
global vectorTramas
vectorTramas = ['/0/0/0']
global cuenta


def leerTrama():
    archivo = open(nombreArchivoLeer,"r")
    tramaNue = archivo.readlines()
    ultimalinea = len(tramaNue)-1
    returnedValues = str(tramaNue[ultimalinea])
    #tramaNue = '/000000h0000.00N/00000.00EO000/000A0B19350C1468D0E3140'
    vectorTramas.append(returnedValues)
    if (vectorTramas[len(vectorTramas)-1] != vectorTramas[len(vectorTramas)-2]):
        return returnedValues
    else:
        return 0
def procesarTrama(lineas):
    returnedValues = lineas
    valores = returnedValues.split("/")
    if (len(valores) == 4):
        print(returnedValues)
        if len(valores[3]) > 50:
            #trama larga
            print("trama larga")
            tramalarga = True
            tiempo = valores[1][0:6].strip()
            latitud = valores[1][7:15].strip()
            longitud = valores[2][0:9].strip()
            curso = valores[2][10:].strip()
            velocidad = valores[3].split("A")[0].strip()
            altitud = valores[3].split("A")[1].split("B")[0].strip()
            presionbar = valores[3].split("B")[1].split("C")[0].strip()
            alturabar =  valores[3].split("C")[1].split("D")[0].strip()
            voltajebater =  valores[3].split("D")[1].split("E")[0].strip()
            TEMPSHT11 =  valores[3].split("E")[1].split("F")[0].strip()
            HUMESHT11 =  valores[3].split("F")[1].split("G")[0].strip()
            NH3 =  valores[3].split("G")[1].split("H")[0].strip()
            CO =  valores[3].split("H")[1].split("I")[0].strip()
            NO2 =  valores[3].split("I")[1].split("J")[0].strip()
            C3H8 =  valores[3].split("J")[1].split("K")[0].strip()
            C4H10 =  valores[3].split("K")[1].split("L")[0].strip()
            CH4 =  valores[3].split("L")[1].split("M")[0].strip()
            H2 =  valores[3].split("M")[1].split("N")[0].strip()
            C2H50H =  valores[3].split("N")[1].split("O")[0].strip()
            TEMPPCB =  valores[3].split("O")[1].split("P")[0].strip()
            TEMPADC =  valores[3].split("P")[1].strip()
        else:
            #trama corta
            print("trama corta")
            tramalarga = False
            tiempo = valores[1][0:6].strip()
            latitud = valores[1][7:15].strip()
            longitud = valores[2][0:9].strip()
            curso = valores[2][10:].strip()
            velocidad = valores[3].split("A")[0].strip()
            altitud = valores[3].split("A")[1].split("B")[0].strip()
            presionbar = valores[3].split("B")[1].split("C")[0].strip()
            alturabar =  valores[3].split("C")[1].split("D")[0].strip()
            voltajebater =  valores[3].split("D")[1].split("E")[0].strip()
            TEMPSHT11 =  valores[3].split("E")[1].strip()
        print("tiempo: " + tiempo)
        print("latitud: " + latitud)
        print("longitud: " + longitud)
        print("curso: " + curso)
        print("velocidad: " + velocidad)
        print("altitud: " + altitud)
        print("presionbar: " + presionbar)
        print("alturabar: " + alturabar)
        print("voltajebater: " + voltajebater)
        print("temperatura: " +  TEMPSHT11)
        datosTransfor = transformarTrama(latitud,longitud)
        latitud_geo = datosTransfor[0]
        longitud_geo = datosTransfor[1]
        print("latitud_geo: " + latitud_geo)
        print("longitud_geo: " + longitud_geo)
        print("altitud_geo: " + alturabar)
        return [latitud_geo,longitud_geo,alturabar]
def transformarTrama(latitud,longitud):
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
def enviarWeb(tempo,lati,longi,altu,curs,velo,presi,altuba,tempera,volta):
    tempoH=tempo[0:2]
    tempoM=tempo[2:4]
    tempoS=tempo[4:6]
    tempo=tempoH+":"+tempoM+":"+tempoS

    curs=str(float(curs)-0.5)
    if float(velo)>0.5:
        velo=str(float(velo)-0.5)
    volta=str(float(volta)/1000)

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
                '"voltaje_bateria":"{}"'.format(tempo,lati,longi,altu,curs,velo,presi,altuba,tempera,volta)
        mqttmsg = "{" + mqttmsg + "}"
        print(mqttmsg)
        r = requests.post("http://www.cansats3kratos.me/data/", data=mqttmsg,headers = {'content-type':'application/json'})
        print (r.status_code)
        print (r.headers)
        print ("trama corta enviada")
    except:
        print("******* error, trama no enviada a web******")
def modelo(lati, longi, alti):
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
    Distancia = math.sqrt(Duvw_1*Duvw_1+Duvw_2*Duvw_2+Duvw_3*Duvw_3)
    vectorD = [Duvw_1,Duvw_2,Duvw_3]
    MatrizD.append(vectorD)
    archivo1 = open(nombreVectorGuardar,"w")
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
def enviarArduino(angulo_theta, angulo_omega, state):
    SET = state
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
    enviarArduino(angulosEstim_v1[0],angulosEstim_v1[1],False)
    time.sleep(3)

    angulosEstim_v2 = modeloVector(coordEstim[3],coordEstim[4],coordEstim[5])
    enviarArduino(angulosEstim_v2[0],angulosEstim_v1[1],False)
    time.sleep(3)

angulosSET = modelo(latitudG_1,longitudG_1,altitudG_1)
thetaSET = angulosSET[0]
omegaSET = angulosSET[1]
enviarArduino(thetaSET,omegaSET,True)
print("__________________________________________")
time.sleep(3)

while (1):
    trama = leerTrama()
    if trama != 0:
        cuenta = 0
        coordenadas = procesarTrama(trama)
        #enviarWeb()
        angulos = modelo(coordenadas[0],coordenadas[1],coordenadas[2])
        enviarArduino(angulos[0],angulos[1],False) #theta,omega
        if len(MatrizD) > 1: #Estimacion
            #estimacion()
            continue
        print("__________________________________________")
    else:
        if cuenta < 1:
            print("Esperando trama nueva.....")
            print("__________________________________________")
        cuenta = cuenta + 1
    time.sleep(0.1) #leer archivo tramas cada 100 milisegundos

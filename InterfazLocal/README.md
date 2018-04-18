# Kratos

# Misión Simple 3

## Developers:

* Alejandro Alzate
* Jonathan Zapata

### Requisitos

* python 2.7
* python requests `[sudo] pip install requests`
* InfluxDB 1.3 (Habilitado para ejecutarse al inicio del sistema)
* Grafana (graficador de base de datos) 4.6

## Instalación:
#### Instalar Grafana:
* Seguir desde el paso APT repository
> http://docs.grafana.org/installation/debian/

#### Instalar InfluxDB:
* Seguir los pasos de la sección 'Ubuntu & Debian'
> https://portal.influxdata.com/downloads

### Configuración

* Los siguientes pasos sólo se hacen una vez, cuando se instala la bd y el servidor grafana:

* Lanzar el servidor de base de datos
`sudo systemctl start influxdb`

* Establecer que lance el servidor de base de datos cuando inicia el sistema operativo: 
`sudo systemctl enable influxdb.service`

* Probar que si funciona:

* Abrir command line y ejecutar `influx` para abrir el shell de la bd.
* Allí se debe crear y usar la bd:

```
create database globo
use globo
```

Crear un registro para probar:

```
insert sine_wave value=0.5
select * from sine_wave
```

Debe salir algo como esto:

```
name: sine_wave
time                 value
----                 -----
1511610827570804924  0.5
```

Otros comandos útiles:
`> delete from "sine_wave"`
`> DROP MEASUREMENT "h2o_feet"`

Esta ventana de cmd se puede dejar abierta para ir consultando en la bd.
Luego, en una ventana de cmd nueva, se debe poner a correr el server de grafana:

`service grafana-server start`

Abrir en el navegador `localhost:3000`
Loguearse con el usuario creado al descargar grafana o por defecto con usr:admin pw:admin

* Agregar el Data Source:

Si ya se agregó el datasource, verificar que sí esté apuntando a la bd globo.

Escribir un nombre para la fuente de datos, por ej. `InfluxDataSource`. Seleccionar influxDB como base de datos por defecto. en URL `http://localhost:8086` y en Database `globo`. Guardar los cambios

* Instalar plugins necesarios:

```
[sudo] grafana-cli plugins install briangann-gauge-panel
[sudo] grafana-cli plugins install natel-plotly-panel
```

`service grafana-server restart`

* Agregar Archivo .json:

Clickear en el ícono de Grafana. Luego en la parte de Dashboards y luego en import, y seleccionar el .json, por ej. `Main_globo.json` que hay en este repo. 
Darle un nombre y seleccionar el datasource previamente creado y luego Import.
En un editor de texto abrir el generator.py de este repo y modificar la linea que dice `DB = "beispiel"` por `DB = "kratos"`
En el cmd line dirigirse a la ruta donde se descargó el repo y poner a correr el generador de datos aleatorios:
`python generator.py`

### Notas importantes:

Al realizar el insert en el código en python, se tiene que seguir la sintaxis:

`<measurement>[,<tag_key>=<tag_value>[,<tag_key>=<tag_value>]] <field_key>=<field_value>[,<field_key>=<field_value>] [<timestamp>]`

Tener mucho cuidado que si se crea un campo con un tipo de dato (por ejemplo string), luego no se puede cambiar por otro tipo de dato (por ejemplo float), toca eliminar el measurement (la tabla) y volver a insertar los datos. Y adicional si se está realizando en el python, no le va a sacar el error.


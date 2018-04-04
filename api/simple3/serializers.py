from rest_framework import serializers
from .models import JsonModel

class JSONSerializer(serializers.HyperlinkedModelSerializer):
    class Meta:
        model = JsonModel
        fields = ('gps_time',
                  "gps_latitude",
                  "gps_longitude",
                  "gps_altitude",
                  "gps_course",
                  "gps_speed",
                  "imu_ax",
                  "imu_ay",
                  "imu_az",
                  "imu_cx",
                  "imu_cy",
                  "imu_cz",
                  "imu_gx",
                  "imu_gy",
                  "imu_gz",
                  "barometer_temperature",
                  "barometer_Pressure",
                  "barometer_Altitude",
                  "time_milisenconds",
                  "gas_nh3",
                  "gas_co",
                  "gas_no2",
                  "gas_c3h8",
                  "gas_c4h10",
                  "gas_ch4",
                  "gas_h2",
                  "gas_c2h5oh",
                  "temperature_sht11",
                  "humidity_sht11",
                  "temperature_i2c",
                  "mean_temperature",
                  "voltaje_bateria",)

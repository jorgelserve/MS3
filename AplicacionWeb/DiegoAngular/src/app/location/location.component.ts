import { Injectable, Component, OnInit } from '@angular/core';
import { LocationService } from '../Services/location.service';
import { BaseChartDirective }   from 'ng2-charts/ng2-charts';
import { ViewChild }            from '@angular/core';

@Component({
  selector: 'app-location',
  templateUrl: './location.component.html',
  styleUrls: ['./location.component.css'],
  providers: [LocationService]
})

export class LocationComponent {
  public Despliegue;
  public valueJson;
  public errorMessage;
  public count_object;
  public altura;
  public visajetemp;
  public visajelon;
  public visajelat;
  public visajetime;
  public visajevel;
  public latitud_ini;
  public longitud_ini;
  public band_coor = 0;
  public dim = 0;
  public Coordinates: Array<any> = [];
  public Temperatura: Array<any> = [];
  public Altitud: Array<any> = [];
  public Bateria: Array<any> = [];
  counter = 10;


  @ViewChild(BaseChartDirective) chart: BaseChartDirective;
  //@ViewChild(BaseChartDirective) chart1: BaseChartDirective;
  constructor(private locSer: LocationService) {
  }
  public brandInfo = '#63c2de';
  // convert Hex to RGBA
  public convertHex(hex: string, opacity: number) {
    hex = hex.replace('#', '');
    const r = parseInt(hex.substring(0, 2), 16);
    const g = parseInt(hex.substring(2, 4), 16);
    const b = parseInt(hex.substring(4, 6), 16);

    const rgba = 'rgba(' + r + ', ' + g + ', ' + b + ', ' + opacity / 100 + ')';
    return rgba;
  }
  public mainChartData: Array<any> = [
    {
      data: this.Temperatura,
      label: 'Temperatura'
    }
  ];
  public coordenadaChartData: Array<any> = [
    {
      data: this.Coordinates,
      label: 'Latitud'
    }
  ];
  public altChartData: Array<any> = [
    {
      data: this.Altitud,
      label: 'Altitud'
    }
  ];
  public batChartData: Array<any> = [
    {
      data: this.Bateria,
      label: 'Bateria'
    }
  ];
  public lineChartData: Array<any> = [
    { data: [65, 59, 80, 81, 56, 55, 40] }
  ];
  public lineChartLabels: Array<any> = ['January', 'February', 'March', 'April', 'May', 'June', 'July'];
  public lineChartOptions: any = {
    responsive: true
  };
  public tempChartOptions: any = {
    responsive: true,
    scales: {
      xAxes: [{
        type: 'linear',
        display: true,
        scaleLabel: {
        display: true,
        labelString: 'Altitud'
      }

      }],
      yAxes: [{
        type: 'linear',
        display: true,
        scaleLabel: {
        display: true,
        labelString: 'Temperatura'

      }

      }]
    },

    elements: {
      line: {
        borderWidth: 2
      },
      point: {
        radius: 0,
        hitRadius: 10,
        hoverRadius: 4,
        hoverBorderWidth: 3,
      }
    }
  };
  public coorChartOptions: any = {
    responsive: true,
    scales: {
      xAxes: [{
        type: 'linear',
        display: true,
        scaleLabel: {
        display: true,
        labelString: 'Longitud'
      }

      }],
      yAxes: [{
        type: 'linear',
        display: true,
        scaleLabel: {
        display: true,
        labelString: 'Latitud'
      }

      }]
    }
  };
  public altChartOptions: any = {
    responsive: true,
    scales: {

      yAxes: [{
        type: 'linear',
        display: true,
        scaleLabel: {
        display: true,
        labelString: 'Altura'
      }

      }]
    }
  };
  public batChartOptions: any = {
    responsive: true,
    scales: {

      yAxes: [{
        type: 'linear',
        display: true,
        scaleLabel: {
        display: true,
        labelString: 'Voltaje Batería'
      }

      }]
    }
  };
  public lineChartColors: Array<any> = [
    { // brandInfo
      backgroundColor: this.convertHex(this.brandInfo, 10),
      borderColor: this.brandInfo,
      pointHoverBackgroundColor: '#fff'
    }
  ];
  //  public lineChartLegend:boolean = false;
  public lineChartType: string = 'line';
  // events
  public chartClicked(e: any): void {
    console.log(e);
  }

  public chartHovered(e: any): void {
    console.log(e);
  }
  ConvertString(value) {
    return parseFloat(value)
  }
  getJson() {
    this.locSer.getJson().subscribe(
      res => {
        this.Despliegue = "Sin desplegar";
        this.valueJson = res.reverse();
        let max = res.length;
        this.altura = this.valueJson[max-1]['gps_altitude'];
        this.visajetemp = this.valueJson[max-1]['temperature_sht11'];
        this.visajetemp = (this.visajetemp / 10);
        this.visajelon = this.valueJson[max-1]['gps_longitude'];
        this.visajelat = this.valueJson[max-1]['gps_latitude'];
        this.visajetime = new Date();
        this.visajevel = this.valueJson[max-1]['gps_speed'];

      //  this.visajevel = ;
        if(max > this.dim){
          for (let i = 0; i <= (max-1); i++) {
            if(this.valueJson[i]['barometer_Altitude'] != null && this.valueJson[i]['temperature_sht11'] != null){
              this.valueJson[i]['barometer_Altitude'] = parseFloat(this.valueJson[i]['barometer_Altitude']);
              this.valueJson[i]['temperature_sht11'] = parseFloat(this.valueJson[i]['temperature_sht11']);
              let temp_sht = this.valueJson[i]['temperature_sht11'];
              temp_sht = temp_sht / 10;
              this.Temperatura.push({"x": temp_sht, "y": this.valueJson[i]['barometer_Altitude']});
            }
            if(this.valueJson[i]['gps_latitude'] != null && this.valueJson[i]['gps_longitude'] != null){

              this.valueJson[i]['gps_longitude'] = parseFloat(this.valueJson[i]['gps_longitude']);
              this.valueJson[i]['gps_latitude'] = parseFloat(this.valueJson[i]['gps_latitude']);
              if(this.band_coor == 0){
                if(this.valueJson[i]['gps_latitude'] != 0 && this.valueJson[i]['gps_longitude'] != 0){
                  this.latitud_ini = this.valueJson[i]['gps_latitude'];
                  this.longitud_ini = this.valueJson[i]['gps_longitude'];
                  this.band_coor = 1;
                }
              }

              this.Coordinates.push({"x": this.valueJson[i]['gps_latitude'], "y": this.valueJson[i]['gps_longitude']});
            }
            if(this.valueJson[i]['gps_altitude'] != null){

              this.valueJson[i]['gps_altitude'] = parseFloat(this.valueJson[i]['gps_altitude']);

              //this.valueJson[i]['gps_latitude'] = parseFloat(this.valueJson[i]['gps_latitude']);
              this.Altitud.push({"x": i, "y": this.valueJson[i]['gps_altitude']});
            }
            if(this.valueJson[i]['voltaje_bateria'] != null){
              this.valueJson[i]['voltaje_bateria'] = parseFloat(this.valueJson[i]['voltaje_bateria']);
              let voltaje_bat = this.valueJson[i]['voltaje_bateria'];
              voltaje_bat = (voltaje_bat / 100);
              //this.valueJson[i]['gps_latitude'] = parseFloat(this.valueJson[i]['gps_latitude']);
              this.Bateria.push({"x": i, "y": voltaje_bat});
            }
            //this.Coordinates.push({"x": this.valueJson[i]['gps_longitude'], "y": this.valueJson[i]['gps_latitude']});
            //this.Temperatura.push({"x": this.valueJson[i]['barometer_Altitude'], "y": this.valueJson[i]['temperature_sht11']});
            //this.Humedad.push({"x": this.valueJson[i]['barometer_Altitude'], "y": this.valueJson[i]['humidity_sht11']});
            //chart.update();
            //console.log(this.valueJson[i]);
          }
        }

        this.dim = res.length;
        this.chart.chart.update();
        //this.chart1.chart.update();

      },
      err => {
        this.errorMessage = <any>err;
        if (this.errorMessage != null) {
          console.log(this.errorMessage);
          alert('Error en la petición');
        }
      }
    );
  }

  apiTimer: any
  ngOnInit() {

    this.apiTimer = setInterval(() => {
      this.getJson()
    }, (this.counter * 400));
  }
  /*  getLocation(){
      this.locSer.getLocation().subscribe(
        res => {
          console.log(res,"respuesta")
          this.valueJson = res;
        },
        err => {
          this.errorMessage = <any>err;
          if(this.errorMessage != null){
            console.log(this.errorMessage);
            alert('Error en la petición');
          }
        }
      );
    }*/

}

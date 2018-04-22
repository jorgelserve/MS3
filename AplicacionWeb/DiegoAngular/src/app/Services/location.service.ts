import { HttpModule , JsonpModule, Http, Response, Headers, RequestOptions } from '@angular/http';
import { Injectable } from '@angular/core';
import "rxjs/add/operator/map";
import { isNull } from 'lodash';

@Injectable()
export class LocationService {
  public url: string;
  public url1: string;

  constructor(private _http: Http) {
    this.url='http://www.cansats3kratos.me/data/'
    this.url1='http://www.cansats3kratos.me/images/'
  }


  getJson(){
     return this._http.get(this.url)
                      .map(res => res.json());

   }

  getLocation(){
    return this._http.get('https://maps.googleapis.com/maps/api/geocode/json?address=medellin')
                      .map(res => res.json());
  }

  getImage(){
    return this._http.get(this.url1).map(res=>res.json());
  }
}

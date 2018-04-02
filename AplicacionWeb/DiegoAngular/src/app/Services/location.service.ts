import { HttpModule , JsonpModule, Http, Response, Headers, RequestOptions } from '@angular/http';
import { Injectable } from '@angular/core';
import "rxjs/add/operator/map";
import { isNull } from 'lodash';

@Injectable()
export class LocationService {
  public url: string;

  constructor(private _http: Http) {
    this.url='http://www.cansats3kratos.me/data/'
  }


  getJson(){
     return this._http.get(this.url)
                      .map(res => res.json());

   }

  getLocation(){
    return this._http.get('https://maps.googleapis.com/maps/api/geocode/json?address=medellin')
                      .map(res => res.json());
  }
}

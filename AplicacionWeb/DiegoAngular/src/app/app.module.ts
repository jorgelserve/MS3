import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';
import { HttpModule } from '@angular/http';
import { routing } from './app.routing'

import { AppComponent } from './app.component';
import { LocationComponent } from './location/location.component';
import { HeaderComponent } from './header/header.component';
import { HomeComponent } from './home/home.component';
import { AgmCoreModule } from '@agm/core';
import { ChartsModule } from 'ng2-charts/ng2-charts';
import { BsDropdownModule } from 'ngx-bootstrap/dropdown';
import { FormsModule } from '@angular/forms';
import { CommonModule } from '@angular/common';

@NgModule({
  declarations: [
    AppComponent,
    LocationComponent,
    HeaderComponent,
    HomeComponent
  ],
  imports: [
    BrowserModule,
    HttpModule,
    routing,
    AgmCoreModule.forRoot({
    apiKey: 'AIzaSyCi1V5aeAKtJf-qzHAmKeUb-zDTM_CB1ME'
  }),
  ChartsModule,
  BsDropdownModule,
  CommonModule,
  FormsModule
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }

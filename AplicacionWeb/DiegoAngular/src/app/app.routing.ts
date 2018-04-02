import { ModuleWithProviders } from '@angular/core';
import { Routes, RouterModule } from '@angular/router';

import { AppComponent } from './app.component';
import { LocationComponent } from './location/location.component';
import { HomeComponent } from './home/home.component'

const APP_ROUTES : Routes = [
	{ path: '', component:LocationComponent}
];

export const routing = RouterModule.forRoot(APP_ROUTES);

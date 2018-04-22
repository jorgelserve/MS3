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
  public imagen;
  public imagen1 = "data:image/png;base64,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z0yaznVco2b/AK/plRjZ3Oc1/QdR1TULy88l5EhEcdpD9pxuBx5signarbSVGeOCT1rPl0XxWth/Z8IL2iQNAySTphlZwfkPdlXIDMB9PXsf7e0zyTL9rTaMcYO7nocYzz24rQjkWWNZEIZGAZSO4NccqCu3qerSzapGKglFqO19elu/z9Wzzt/DXiFLWa1tYfs9o7GB7WG5URNETkFOMrwAGY5ZtzcdKnt/D3iOO6RfPkijiDwl4pEAMWdwMS4+RmwAWOTliewrv6KXsI92aPN6zVuWP3f8Ey/D8F9baRGmou7XJLM2+XzCuTnbn26fQVqUUVqlZWPNqTdSbm+oUUUUyAooooAKKKKAKa/8hmT/AK91/wDQmq5VNf8AkMyf9e6/+hNVygAooooAKKKKACiiigAooooAqap/yCrv/ri38qt9qqap/wAgq7/64t/KrfagAooooAKKKKACiiigAooooAKKKKACiiigAooooAKMUUUAFGB6UUUAFZlpazR69qNw6ERSxwhGyOSobP8AMVp0UAFFFFAGdfaLaahMJpw+8YGVbGVHO36frUUPh6ygnSZPM3IwZfm74I/Hqa1qKAMiDQ3t4PLj1G43DGGKpjAXaARjB44rTt4Rb28UKklY0CAnqcDFSUU7kxio7BRRRSKCiiigAooooAKKKKACiiigCmv/ACGZP+vdf/Qmq5VNf+QzJ/17r/6E1XKACiiigAooooAKKKKACiiigCpqn/IKu/8Ari38qt9qqap/yCrv/ri38qt9qACiiigAoqG4uobZQZXxuOFUDLMfQAck1wPjNr25vLa7SW6EMJWMWdujuwYkku7JwhxgAcnP1xQB6JRVDRZLuXRbN75WW6MSmQOuGz7jsfUetX6ACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKAKa/8hmT/AK91/wDQmq5VNf8AkMyf9e6/+hNVygAooooAKKKKACiiigAooooAqap/yCrv/ri38qt9qqap/wAgm7/64t/KmSX0guHSC3M0cQxKytyG9AD1wOTz3HWgC5JIkSF5HVEUZLMcAVmXGqO+1LdSof7rshLP/uJ1P1OB9aqwQ3eoX88jXET26uDCxGWiH93YRgMP7zZPPSti3tIbYEopLt952OWb6k0AULfTZZHMlwzx7uG+fMjj0Zx0H+yuB71pxQxwRiOJFRB0VRgU+igAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooApr/AMhmT/r3X/0JquVSXjWnz3t1x+DH/EVdoAKKKKACiiigAooqK5m+z2s023d5aFtucZwM4oAlprusaF3YKo5JJwBWXBeXl24ikaGylIz5RUu34E4B+ozWfeo8kwnhae/giBMuGAOR/cb7ufZQD/tdqALN5qaalFJaWJ81XBRpIyCcdDj0/wB48DtnpVG01d4nSz+02qRtK6+ainMYB4+9w2f7x+uDmq+haRebL6MRRW1hcXTXEe3kOrYOSCAT/wAC474NQeI73QU0+4t7cwz3i4VmWUbmbIOG4JYZAJ4IAHOBQBffVdPtL+3uLYM0Zk8u5vnbhxggDJ5fBx04XntmuqBBAIIIPpXn+g2H9uQyTNr4fUE2rcQwBcRKRnYCPmUEH7yEV1TWp02x846gYVgTHzYEKqB02+nHrn3oA16Kq2V7HdaXb3xZFjlhWXdnCgEA9+3NMTVrBxn7VGo9XO3P50AXaTNYOqeKLWygdoGjlKkDzGcLGCTgDcSByeByB71yVxq97LqPm3a+aUBzat8skY4+ZQeDnI9Bxw3ek5JblRi5bHplFcNpvj3T/tpspLosy7c+cpjIJ6LubAz7N+DMa7O3uobpS0T528MpGGU+hB5FMkmooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKAKN+RbPHfHhIQRL7RnqfwIB+mauI6yIHRgysMgg5BFOIzWM0N1ozFrOJrnTzybZPvw+pjz95f9nt29KANmiq1lqFrqEZe1mWQKcMOjIfRlPKn2NWaACiiigAqtqHOm3Q/wCmL/yNWabJGssTxuMo4KsPUGgDAGpWWo2k0Gp2bQWol8lZJmUo54AOQflJJ4zisi71W6lhmtbiUQxW+Y5Ujfy/l6BpJDwgYYIC8nNb48PWi2c9tcT3Nxay5MkUsg2kHqDgAkfU1mR2emzXUZ0KxileBRHHcybmtrYD+4M4Zh6L+JFAHN6rf6rcadDpoWRbNBEjTLKbdmjOETCtzy3GXKghScd67fTtEtF0e3gu7Ky3Kp3CFPkwffqcjGex+lOm0CGbTXtTPKJXkWZ7jgu0gIIY9uwGOgHAxXJTeGbywtls/wDR5WknBiX7VMTIBgklc+gyRkLQBpa//YtwUa1h23lthI7y1fyfJH93eOo/2cH6VXEGo69Mcl5MceY42rH/ALq9Afc5PtWraeHobdBc6lKuEHC5Cqg9PQfQfmam1DWHsorZLWxuBbzSeUskUO4r8pbKx5BIwDz+hoAnisrfTrW2jvJmuDCipFGQW+6MDC9WPucn6VR8Qavc21tGivbwT3EipDaSOfMmBb5slclQBk5AOPXtVBoG8UOqWdqv9nRzhpb+6lPmTsnZApBUZ7kr3AHep4NOvvDN9cXy2aarFOcyzxr/AKWg9OSfMUegIP1oAqQznWNT/sm8lj0m2jcPHYiP57vGDu3uMMuewGeOcdKsar4MYxl9NlEiDJ+yXLkKD38qQfNEfYZX1WteS50HxHpEjzvbXFmnMnmnaYiPXOCjD8DXOwahrUVpeDQpp9RtEIW3a6hZpAP4irnAcDtu5PrjGQNjlrrT3N55MlrKbqBlbyZI1W5i5JyuPlkXrymc45WnaLq2r2mrNdXeqNcabKuUxAA8DYztGzkKMHIw3+7XbaRp+i61Z3KS3dxfzuwa6S4do5EcdMpwUI7fpWVrnhS6slkuone9gUFmlyouowAeTnCzAD+9hv8AaNZ8rXwmnOpaSOl0/X0miRpGWSNxlZovmDD6DOfqM47gVtxyJKgeN1dG5DKcg143vutNV7uFigOC7ohEbHA2+dGwBRuuMgH0aui0jxLI999lSKUXv3mWAeZGy5wWY8YHf58HHRm6VSlfR7ilCyutj0Sis/S9UTUvtKKMSW0gikK8qTtDZU9xhhWhVEBRRRQAUVzniDxBdWKx/wBmwwyruImuJSTHH0AXgjLEnt0wc9q19JvxqmlW18qbBMgfbnOPoe496ALlFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQBRvNIsr6QTSxFbhRhZ4mMci/wDAlwce3Sqv2LWLXP2XU47hB0S9hy3/AH2mP1BrYooAyDe63FxJpEMuB1t7sc/g6r/OnDVL7A3aFeg+0sJ/9nrVooAyG1LVWJEehSj0MtzEo/QtQf7fuDj/AIl9kpHUb52H/oA/nWvRQBkLoEU5Dalcz6gw52TkCIH/AK5rhT+INayqqKFUAKBgADAApaKAGSSxwxtJI6oi9WY4ArlL7WbPVdcsbWxubdL22m3oZZArMMYZQvU5U+mfaoNat9RlupTendD5reSquyq0ZA2jcvKsOeec9wRXM6poVvqiLDJbGZk+aNAoS4j+YEshXhwOeU/FRUybXQqKT6nTzXlzq+q239j3Ut59md2uLhYVa1yBgIu4jJz3UkjnJ7VHp32VdWU+KhONXZisLXbD7Ng/ww4+UcY6/MfeuX0fxbr3hnZBcD+2dJUAAghbmEc8ejAY78+4rtj4t8Oa5opeL/iYxykRmzEO6TcRnayH7v1OB6GmpJ7BKLjuXbzw3Cbhr3Spm02+P3pIVGyT2dOjfXr71lt4xurSyk+1Wdu06zfZ1uIrgC3ds4JJPKgH1HUEZqp/YWuf2OkUaTi28wu+nNe9Y8nEavjIwMcE4PtW5ot1od1E2n29pHazIu2WxniCSKPcH7w9xkUySifCLanFNc39zB9tuWEpmt4F4xjavOdyjA4PXrVpNYvdDAh122X7MOF1C1QmLH+2nWP68j6VU1O3HhNEudJvo7eKWQIunXLZilYnGI+6HnPGR3xTZdQvPEN1c6fLOmlWMMYa5Iy0kynrtcgIqds8k+i0AO8Sz6PdGFrUSXOtNHvsm01v3+OxLjgR567vl69aq3VtqElxDF4nuZJNJ8kDbGyRefKOqyIhLPnsq8ccirtjaF9QlutCEqRzLtluZ+IHOfvJGAN7e/yrz1Na0dnY6U32y6mae7YY+0TfNIfZQPuj2UAUAYl7bME/tCPSxYRbRbh0iU3To7AYPUImcZ+83sDVvSvDUdvbLCsEdnag7hbwDGT6secn3JJ9xTNQv7rWYZrKyWaJJFKNJEAzqD3yeB9M/jW7HqEfmCKeKW3Y8J5wAD/QgkZ9utAFiCCK2jEcKBF9B/P61JRQelAFa4vYoH8sZkmIyIk5bHqewHucCswvc6mSF2vH3CsREPq3Vz7DA9TU1loa26NHPO1xGXLhGGAc8/N3c/U49q1QAAABgDgYoAzxo1nLEEvYY7sYwFmjBRfZU6D+fvV9EWNFRFCqowABgAU6igAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigBrosiFHUMpGCCMg1h6h4djmQmALjO7yn6ZHQg9Qf5diK3qKAPKPE+k6y6x/YTAtyjhpTcx5mdMYwGyFf8A4Fz6PWFNpEsUC60RJpt+i/Pd20uGhYAALLjlenRhg5wCa9uuLWG6j2TRhx2z1H0PauW1vwlHdxfcM6LypGBImPTjDD2II/2TUuKepcZtadDn9J+JF7pR+z+LIFaBSQNTtVJTGcZkT+H6j8q3r+RPF8sC6bYxzWkMqv8A2o8mwYGCVhZfmJPQkYA571yw8PLZ2ht7dZncMSiqAUY5ztMZx5fTqpKjnIWpvDfhZtOnleJ5Fe5jAk0+ymYQD3ZuNx7fLtXtk9KFfZhJRtdHQadbNaa5eyxrFrGoHakc53ZtxjBR5DlVA44UbjySO9bg0sSYutauVuWTDCMjbBEfZT1P+02T6YqtB4ck3xyyXbwmNNkcEPyxIPoMfpj8amj0CRrsyXeoTXEOBshOQFPrkkn8sVRA2814s/k2KF3bgMVyT9F/x/I0y20Sa6k8/UJGyeqBssfqf6D9Kj1N7zT/ABBpYsbcG2kikhZfK+TcWQj5hypwGPQ5wfSr1xra2/7p4HS527iknCgeoYfeH0yfUCgDQRILODChIokGT2A9zWJrHiC1t7SVnMSW4Hzy3AG3H0P6Zxntms77Vfa2xe12vCvJupflgj91x94j2z/vr0qlfQWGiX9rdP8A6feOnmJe3TboLYE43JGvGSe65bpk0AdN4bN0bCQzpOsRkJgE+d+zA655AznAIBx2FbNZsVtqkcaxPfxS5+9MYdrj6AHb/nvTTcNYX9paSXizC6ZlRZSBJkKWJGAMjj049aANSikzjrQrBhlSCPUUALRWPqviKx0u3klklTan3nJwi/UjOT7DJrlF8ZSS6gZJZZrW3/5ZTOv7puMncozsHI5OfcrSbS3Gk3seh0Vz2keLtO1Nmi+0ReYv8SNlWGcbgf7uf4hlfeugBBGQaYhaKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKAKGo6NY6ogW6h3EHO5Tg+n49TVm2tYLSIRQRhFHp3+p71NRQAUUUUAV7y3a4iXYQJEdZELdMg9/btWHH4Zlvb83utXklyod2isg2YEViMBuAXxgcHjPbvXSUUAcjqEVzf+L49P+Saxhijke2dgsaqSRkqOWb5TgYK+uK3JNO0izdb2W2tojBkpIygCP1K9geO1ZPiTw3cahOb2ynYThUHlCUw7tu7GJF5X7x/KqD6NeC/Et9L5UewLGWuGuHLdwkZGAf9rk0AaF94jlmYQ6fGy7/uyMm53/3E/qcCq9jp0sWqw3lwzyXkW5hAhDudylcyOeFHPQYH1rVsdKManYrWsb/fO7dPJ/vP2+g/MUtxqdlpllI0Hkx28R/eTu22FCTj5m7nJHTPvigCxJb+Yhl1OZPKHJiBxGP94n7348e1Ubu3hmtZZbXTVWJELbggRpAB0VSVHPqxA+orP1DVbS5ja3sLqfVNVlQmH7DtIhz0YMQUQf7RyfTNVdQtbmOS1fxKt9e6eEzMY3DQq/H340VSVHPXIz2oAkh1FNSsorXSdHjme7Qhn1GSMxhBjOVViWHI+UAA+o61lXnw1u7G3E+i3sb3Gz97aTZSF29Y+SYz0GORwOldjJpei67YQPEsLRIB9nntWCtH6bGXp9Kyn1jUdB1GDTLi4ttT87iJmk8udABn94ACCMfxAD6Umk9xptO6PO7SSSx1gCW0Gn6mjMWsbsbkJZsMyYx1H8aEe+elb2l+Or+11+DTH0uVYrl9sOyQOgGM8sdoycHgYI4+VjXQjS18cR+Zq0tnLp8UjItpbESfMMglpCNwPpjb2rndY8EarokJOnxnWtORcfZpMC4RQDwCRiTr7H61NnHYu8Zb6Ho9nqtvdny8mKYcGN+Dn29f5+1Xq8f0nW5GjKRym8ii+VraZ9s8OAOFZueDnhwR6EV2Om+KFDJE03nbnEYil+SdWJwBtJ5+uSP9qnGSZMoOJ19FQWl3DfW6z27h4ySM+4JBH4EEVPVEhRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAIzKilmIAAySTgCudkm3eKIr2zWScG1MEiA4BG7crL+JIJOBjHWsi+vdT1O2Td5ltKciSFUDPE2eNqHh+Pfd3UVyk1jdW6zvFKJLe4k/fq7NJDM+4YLZIZJB6jawx0qZSsVGPN1O98R6jJp9gJLiW3e5kYCDT9z/vueVG0FnOPbb61UFzea1qR0e7v4tKi8lWNnbKyTSgjlQ7gfKOM7BnnqK5zQ/EEWm6v5q6XAtxMixtCxAnfGeYpj8suefkba3Heu1lvvDniPTmF3LbtHE3zpcHypbd/cHDI3vxTTT2E01ox91ot5bSrc6LfGGRECG1uCXgkUdB6oeTyPyot/EqBjbajYXdpej/liIWmEg9UZAQw/IjuKw7LUtX8y4g0q7udQ02OMsl3NaeZICP4FbcvmexIz9av+H7DSdUjkvpLuTUb5k8q4aZmV4s8lNnGz8h+tMRVtdLm1LU2v7Cxk0uyfJkH2l4WuT2JRDgd+ev8qm0a5s/D0hs9Tsf7OuZXOLpmMkU5J4/enkH2b9atCw1fw+B/Zch1GwUYFlcPiWMekch6j/Zb86ZceIf7WtVtrHRZrp7jKbLsLHEuB82/JJwPYHtigC1q2jWKmTVYbv8Asq6RdzXkbBVI/wCmgPysPrWIniTWLrTonElnbxSz+SuoNbTFGX/noFIAUehY7ffFUv7LtbJrOJEsdRnt5i86eU5hjbBwqszNtwSDtAY8dB22IrO/1qVZriTzVBypYbYk/wB1Ocn3OT7r0oAw9Q0bSr8xSRrcXN4j7/7XuHKyvnGVUDG5DjgHCema3NK8MsCJZA6EjBll+aZweo6fKD6AAf7Peugs9Lt7Q78GSbqZH5Ofb0/nV6iwX6EFnZwWFqltbRiOJckKPUnJP4kk1PRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFAFW80+3vVxKg3YwHHUf8A1vY8Vz2oaNNCzSkNINu0zxgb9vowOQ6+zBh6ba6uigDyTX7aCy0qe6aye4gUAyJbReYpXnkoSTHjJOfmX0YdKzrC8n+zRXtspvLWSD9yZ1AnjTb/AMs5HUnAIOA+Vz0Ir1u70iG4fzYiYJ85Dpxz6kf1GDXFeIfCf2naXeWzmTd5U0LEwkt1ygIAJ7lcE9w1Q4dVoaKfSWpZ0Hxn5MH2e+jM0EC7XmghKywAcfvoBkgf7abl+lW9avfD1/LDc2VxJPqzLi2k0lg07DrgkfKV9d/y1y9zpqW0UD3cshmi/wBXKDtfdxkxOg5PH3RhueVNaOmDWTOpa4lhCq0cewYlkDEElkGUVuB8wG7rkDrTi3s0KUVumWrltQeSzs/Ec93Kr27SzWtpJGodsgBXC4bbjOTkJx1qYWJ1OaBIrC3gjt1KQwWyhRGhGCGcYOCOwwP96tew8Pbcvc5QM25kDEs59WbJJP4k+/at+KGOCMRxIqIOgUYqiDKsdBggRTOFkKjCxqMIo9AP8j2rYAwMCiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACiiigAooooAKKKKACkZVdSrKGUjBBGQaKKAMW78OpNMr213Napn50Q5DDIP8AQjnI5PFaVpY29mmIYwCRgseSfx/p0oooAs0UUUAFFFFABRRRQAUUUUAFFFFABRRRQAUUUUAFFFFABRRRQB//2Q==";
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
    console.log(e);
  }

  public chartHovered(e: any): void {
  }
  ConvertString(value) {
    return parseFloat(value)
  }
  getImage(){
    this.locSer.getImage().subscribe(
      res=>{
        this.imagen=res

        this.imagen1=this.imagen[0]["img"]

        this.imagen1="data:image/png;base64,"+this.imagen1
        console.log(this.imagen1);

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
  getJson() {
    this.locSer.getJson().subscribe(
      res => {
        this.Despliegue = "Sin desplegar";
        this.valueJson = res.reverse();
        let max = res.length;
        this.altura = this.valueJson[max-1]['gps_altitude'];
        this.visajetemp = this.valueJson[max-1]['temperature_sht11'];
        this.visajetemp = this.visajetemp;
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
  imageTimer: any
  ngOnInit() {

    this.apiTimer = setInterval(() => {
      this.getJson()
    }, (this.counter * 400));

    this.imageTimer=setInterval(()=>{
      this.getImage()
    },(this.counter  * 3000));
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

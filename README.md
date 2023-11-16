# esp_airQualityMonitor
Air Quality monitor using SHT3x and PMS5003
The first 6 are two sets of PM1, PM2.5, and PM10 concentrations. the first set are labeled "standard", while the second set are labeled "atmospheric environment". What do these measure and how do the "standard" ones differ from the "atmospheric environment" ones?

It has to do with the density of the air used for calculations. "Standard" refers to the concentration "corrected" to the "standard atmosphere" which in the US is DEFINED as "having a temperature of 288.15 K at the sea level 0 km geo-potential height and 1013.25 hPa"  - See https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html

On the other hand, the "ambient conditions" are just as the air is "now" (whatever temperature and pressure there is).

The AQI is a standardized index for reporting air quality on a scale from 0 to 500, with higher values indicating poorer air quality.

How to Map PMS output to AQI:  https://www.airnow.gov/sites/default/files/2020-05/aqi-technical-assistance-document-sept2018.pdf


For this AQI… use this descriptor… and this color
0 to 50         Good                            Green
51 to 100       Moderate                        Yellow
101 to 150      Unhealthy for Sensitive Groups  Orange
151 to 200      Unhealthy                       Red
201 to 300      Very Unhealthy                  Purple
301 to 500      Hazardous                       Maroon




AQI	    Air Pollution Level	    Health Implications	                                                                Cautionary Statement (for PM2.5)
0 - 50	Good	                Air quality is considered satisfactory, and air pollution poses little or no risk	None
51 -100	Moderate	            Air quality is acceptable; however, for some pollutants there may be a moderate     
                                health concern for a very small number of people who are unusually sensitive to 
                                air pollution.	                                                                    Active children and adults, and people with respiratory 
                                                                                                                    disease, such as asthma, should limit prolonged outdoor exertion.

101-150	Unhealthy for 
        Sensitive Groups	    Members of sensitive groups may experience health effects. The general public is 
                                not likely to be affected.	
                                
151-200	Unhealthy	            Everyone may begin to experience health effects; members of sensitive groups may experience more serious health effects	Active children and adults, and people with respiratory disease, such as asthma, should avoid prolonged outdoor exertion; everyone else, especially children, should limit prolonged outdoor exertion
201-300	Very Unhealthy	        Health warnings of emergency conditions. The entire population is more likely to be affected.	Active children and adults, and people with respiratory disease, such as asthma, should avoid all outdoor exertion; everyone else, especially children, should limit outdoor exertion.
300+	Hazardous	            Health alert: everyone may experience more serious health effects	Everyone should avoid all outdoor exertion


To map PM2.5 (Particulate Matter with a diameter of 2.5 micrometers or smaller) to Air Quality Index (AQI) in an ESP (presumably an ESP8266 or ESP32) project, you would typically use a formula or a lookup table. The AQI is a numerical scale used to communicate how polluted the air currently is to the general public.

def map_pm25_to_aqi(pm25):
    if pm25 < 0:
        return "Invalid value"
    elif pm25 <= 12.0:
        aqi = map_range(pm25, 0, 12, 0, 50)
    elif pm25 <= 35.4:
        aqi = map_range(pm25, 12.1, 35.4, 51, 100)
    elif pm25 <= 55.4:
        aqi = map_range(pm25, 35.5, 55.4, 101, 150)
    elif pm25 <= 150.4:
        aqi = map_range(pm25, 55.5, 150.4, 151, 200)
    elif pm25 <= 250.4:
        aqi = map_range(pm25, 150.5, 250.4, 201, 300)
    elif pm25 <= 350.4:
        aqi = map_range(pm25, 250.5, 350.4, 301, 400)
    elif pm25 <= 500.4:
        aqi = map_range(pm25, 350.5, 500.4, 401, 500)
    else:
        return "Invalid value"

    return int(aqi)

def map_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min




 float pm25Value = 50.0;  // Replace this with your actual PM2.5 value

void setup() {
  Serial.begin(9600);
  float aqi = calculateAQI(pm25Value);
  Serial.print("AQI: ");
  Serial.println(aqi);
}

void loop() {
  // Your main code can go here
}

float calculateAQI(float pm25) {
  // Define AQI breakpoints and corresponding concentration ranges
  float breakpoints[] = {0, 12, 35.4, 55.4, 150.4, 250.4, 350.4, 500.4};
  float concentrations[] = {0, 12, 35.4, 55.4, 150.4, 250.4, 350.4, 500.4};

  // Determine the AQI category
  int i;
  for (i = 0; i < 7; i++) {
    if (pm25 <= concentrations[i+1]) {
      break;
    }
  }

  // Use the AQI formula to calculate AQI
  float aqi = ((breakpoints[i+1] - breakpoints[i]) / (concentrations[i+1] - concentrations[i])) * (pm25 - concentrations[i]) + breakpoints[i];

  return aqi;
}   

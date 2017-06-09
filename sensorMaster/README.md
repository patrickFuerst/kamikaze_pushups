## Sensor Master

Sensor Master programm is responsable for collecting the data of each sensor/arduino and convert it to a json format before making it available through a socket.

The strucutre is as follows: *Name* of the sensor followd by its *data*

```json
{
  "orientation_sensor":
  {
    "x": 0.001,
    "y": 0.2,
    "z": 0.1,
    "w": 0.0,
    "ax": 0.0,
    "ay": 0.0,
    "az": 0.0,
    "system_status": 3, // from 0 to 3. 0 meanig bad data and 3 good data
    "calibrated": true,
  }
}
```


See *sensors* README for specific information on available data.

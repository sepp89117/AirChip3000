#include <AirChip3000.h>

AirChip3000 hc2 = AirChip3000();
bool sensorOK;

void setup()
{
  Serial.begin(19200);

  Serial.println("Init Sensor...");
  Serial.println();

  if (!hc2.begin())
  {
    sensorOK = false;
    Serial.println("Sensor error");
  }
  else
  {
    sensorOK = true;
    Serial.println("Init done");

    Serial.print("DeviceType: ");
    Serial.println(hc2.getDeviceType());

    Serial.print("FirmwareVersion: ");
    Serial.println(hc2.getFirmwareVersion());

    Serial.print("Device Serial Number: ");
    Serial.println(hc2.getDeviceSerialNumber());

    Serial.print("Device Name: ");
    Serial.println(hc2.getDeviceName());

    Serial.print("Sensor Status: ");
    Serial.println(hc2.getSensorStatus());

    Serial.print("Sensor Quality: ");
    Serial.print(hc2.getSensorQuality());
    Serial.println(" (0 = 'good' to 100 = 'bad' or 255 = 'na')");

    Serial.print("Temperature [Celsius]: ");
    Serial.println(hc2.getTemperature());

    Serial.print("Temperature [Fahrenheit]: ");
    Serial.println(hc2.getTemperature(true));

    Serial.print("Humidity [%r.H.]: ");
    Serial.println(hc2.getHumidity());

    Serial.print("Factory Correction A1 [%]: ");
    Serial.println(hc2.getFactoryCorrectionA1());

    Serial.print("User Correction A2 [%]: ");
    Serial.println(hc2.getUserCorrectionA2());

    Serial.print("Sensor Temperature Correction: ");
    Serial.println(hc2.getSensorTemperatureCorrection());

    Serial.print("Sensor Drift Correction: ");
    Serial.println(hc2.getSensorDriftCorrection());

    Serial.print("Is Recording: ");
    Serial.println(hc2.getIsRecording());

    Serial.print("Recording Mode: ");
    Serial.println((hc2.getRecordingMode() == (char *)'1' ? "Start-Stop mode" : "Loop recording"));

    Serial.print("Log Interval: ");
    Serial.print(hc2.getLogInterval());
    Serial.println(" (seconds = [x] * 5s)");

    Serial.print("First Data Date: ");
    Serial.println(hc2.getFirstDataDate());

    Serial.print("Number of data records in memory: ");
    Serial.println(hc2.getNDataRecords());
    Serial.println();

    if (hc2.getIsRecording())
    {
      Serial.println("Stop recording...");
      if(hc2.stopRecording()) {
        Serial.println("Recording stopped.");
      }

    }
  }

  Serial.println(F("Setup done."));
  Serial.println();
}

void loop()
{
  // Measure every 60 seconds - the measurement and output of this data takes approximately 625ms
  delay(60e3 - 625); // I know there is a better way to do an interval measurement. But I wanted to document how long a measurement takes on average.

  if (sensorOK)
  {
    Serial.println();
    Serial.println("------------MEASURE------------");

    Serial.print("Temperature [Celsius]: "); 
    Serial.println(hc2.getTemperature());

    Serial.print("Temperature [Fahrenheit]: "); 
    Serial.println(hc2.getTemperature(true));

    Serial.print("Humidity [%r.H.]: ");
    Serial.println(hc2.getHumidity());

    Serial.println("-------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("Sensor error. Retry...");
    if (hc2.begin())
    {
      sensorOK = true;
      Serial.println("Sensor inited!");
    }
  }
}

Applying new binary to device.

•	Changing Desired Property in Device Twin with following Details
o	desired.firmware.fwPackageURI : URL contains new firmware binary.
o	desired.firmware.fwVersion : New Version of Binary

Sample Device Twin Change

 "properties": {
        "desired": { 
                                           "firmware": {
                "fwPackageURI": https://fixxxxxxxxxxage.blob.core.windows.net/firmware/otafu_1.1.bin?sig=xxxxxxxxxxxxxxx”,
                "fwVersion": "1.1"
                                             }
                             }
              }

Update the Device Twin Reported Properties after successful Firmware Upgrade.

1.	At starting of Firmware Upgrade, update " reported. tbufwVersionCommsBoard " property to new binary version and Status as "In-Progress"
2.	After Successful Upgrade, update " reported.firmwareUpgradeStatus.status " property & latest firmware version under " reported.firmware.currentFwVersion"
3.	Update “reported.firmwareUpgradeStatus .errorMessage” property if any error happened during firmware upgrade process.

Sample Device Twin Change

"properties": {
        "reported": {  
                                    "firmwareUpgradeStatus": {
                                                    "status": "success",
                                                     "errorMessage" : ""
                                                    },
                                    "tbufwVersionCommsBoard": "1.1",
        "firmware": {
                "currentFwVersion": "1.1",
                "fwUpdateStatus": "current"
         }
       }
  }






Telemetry: (60s/time)

{
  "body": {
    "power": true,
    "childLock": false,
    "sleepMode": false,
    "autoMode": true,
    "fanSpeed": null,
    "timerMinutes": 0,
    "pm25": 26,
    "voc": 0.48,
    "hcho": 0.01,
    "aqi": 36,
    "filterLife": 40,
    "errorFlags": 0,
    "timestamp": 1765159099
  }
}

Device Twin


1. Filter
   {
  "reported": {
    "filter": {
      "life": 40,
      "model": 12345,
      "serialNumber": 67890,
      "installDate": 1764036551
    }
  }
}

2. Status
   {
  "reported": {
    "status": {
      "power": true,
      "autoMode": true,
      "fanSpeed": null,
      "childLock": false,
      "sleepMode": false,
      "timerMinutes": 0,
      "lastStatusUpdate": 1764036551
    }
  }
}
3. Error
{
  "reported": {
    "errors": {
      "errorFlags": 0,
      "pm25SensorError": false,
      "vocSensorError": false,
      "hchoError": false,
      "wifiError": false,
      "lastErrorTime": null
    }
  }
}

4.Capabilities
{
  "reported": {
    "capabilities": {
      "supportsHCHO": true,
      "supportsVOC": true,
      "supportsPM25": true,
      "maxFanSpeed": 3,
      "maxTimerMinutes": 720,
      "supportedModes": ["auto", "manual", "sleep"]
    }
  }
}

5.Connectivity
{
  "reported": {
    "connectivity": {
      "wifiConnected": true,
      "iotHubConnected": true,
      "signalStrength": -65,
      "lastConnectedTime": 1764036551
    }
  }
}


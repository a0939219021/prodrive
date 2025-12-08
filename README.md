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

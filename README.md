# TPS_DAQ
TPS/OSU flight-test data acquisition system
This repository will be the central communications portal for updates, master code, and issue tracking. 

For data post processing, download all *.m* files and use the Run_file.m to generate the sensor data structures you can use for further analysis

To flash using Windows, unzip bstl.exe and download the eeprom file. 

Command line usage:
bstl.exe -p 3 -d COMx TPS_MAIN.eeprom

where COMx is the detected com port in Windows, i.e. COM5

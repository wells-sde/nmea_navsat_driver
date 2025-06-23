### Usage
```python
source ./devel/setup.bash    
roslaunch nmea_navsat_driver nmea_serial_driver.launch port:=/dev/ttyACM0 baud:=115200
#topic /fix 即为定位消息

#record data
rosbag record /fix /vel /heading /time_reference

#optional, visualize gnss position online with topic /gps_odom and /gps_path
#only publish valid GNSS position(status > 0) 
python3 ./src/nmea_navsat_driver/src/libnmea_navsat_driver/nodes/navsatfix_to_odom.py
#only publish RTK Fix position
python3 ./src/nmea_navsat_driver/src/libnmea_navsat_driver/nodes/navsatfix_to_odom.py --ONLY_USE_RTK_FIX
rviz
```

Visualize GNSS log file with matplotlib offline
```python
#only draw position with gnss status equal to 4(RTK fix) or 5(RTk float)
#change file path to your GNSS log file
python3 ./src/nmea_navsat_driver/scripts/plot_nmea_data.py
```
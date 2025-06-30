import os
import re
import utm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import argparse

def parse_nmea_line(line):
    """Parse a NMEA GGA line and return the time, latitude, longitude, altitude, and fix type."""
    match = re.match(r'^\$..GGA,(\d{6}\.\d{2}),(\d{2})(\d{2}\.\d+),([NS]),(\d{3})(\d{2}\.\d+),([EW]),(\d),(\d+),\d+\.\d+,(\d+\.\d+),M', line)
    if match:
        time = match.group(1)
        lat_deg = float(match.group(2))
        lat_min = float(match.group(3))
        lat_dir = match.group(4)
        lon_deg = float(match.group(5))
        lon_min = float(match.group(6))
        lon_dir = match.group(7)
        fix_type = int(match.group(8))
        altitude = float(match.group(10))

        latitude = lat_deg + lat_min / 60.0
        if lat_dir == 'S':
            latitude = -latitude

        longitude = lon_deg + lon_min / 60.0
        if lon_dir == 'W':
            longitude = -longitude

        return time, latitude, longitude, altitude, fix_type
    return None

def read_nmea_file(filepath):
    """Read a NMEA file and extract time, latitude, longitude, altitude, and fix type."""
    times = []
    latitudes = []
    longitudes = []
    altitudes = []
    fix_types = []

    with open(filepath, 'r') as file:
        for line in file:
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                result = parse_nmea_line(line)
                if result:
                    time, latitude, longitude, altitude, fix_type = result
                    times.append(time)
                    latitudes.append(latitude)
                    longitudes.append(longitude)
                    altitudes.append(altitude)
                    fix_types.append(fix_type)

    return times, latitudes, longitudes, altitudes, fix_types

def convert_to_utm(latitudes, longitudes):
    """Convert latitude and longitude to UTM coordinates."""
    utm_coords = [utm.from_latlon(lat, lon) for lat, lon in zip(latitudes, longitudes)]
    eastings = [coord[0] for coord in utm_coords]
    northings = [coord[1] for coord in utm_coords]
    return eastings, northings

def plot_gps_track(eastings, northings, altitudes):
    """Plot GPS track in 3D using matplotlib."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(eastings, northings, altitudes, marker='.', linestyle='-')
    ax.set_xlabel('Easting (m)')
    ax.set_ylabel('Northing (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('3D GPS Track')
    ax.plot(eastings[0], northings[0], altitudes[0], marker='*', color='red')  # Start point
    ax.plot(eastings[-1], northings[-1], altitudes[-1], marker='*', color='green')  # End point
    plt.show()

if __name__ == '__main__':
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Process NMEA file and plot GPS data.')
    parser.add_argument('filepath', type=str, help='Path to the NMEA file')
    args = parser.parse_args()

    filepath = args.filepath
    times, latitudes, longitudes, altitudes, fix_types = read_nmea_file(filepath)

    # 过滤出有效的GPS定位数据
    # 0: invalid, 1: GPS fix, 2: DGPS fix, 4: RTK fixed, 5: RTK float
    valid_indices = [i for i, fix_type in enumerate(fix_types) if fix_type == 4 or fix_type == 5]   
    valid_latitudes = [latitudes[i] for i in valid_indices]
    valid_longitudes = [longitudes[i] for i in valid_indices]
    valid_altitudes = [altitudes[i] for i in valid_indices]
    valid_times = [times[i] for i in valid_indices]

    if valid_latitudes and valid_longitudes:
        # 以第一个有效的GPS定位作为参考点
        ref_easting, ref_northing, zone_number, zone_letter = utm.from_latlon(valid_latitudes[0], valid_longitudes[0])
        ref_height = valid_altitudes[0]

        # 转换为东北天坐标系
        eastings, northings = convert_to_utm(valid_latitudes, valid_longitudes)
        enu_x = [e - ref_easting for e in eastings]
        enu_y = [n - ref_northing for n in northings]
        enu_z = [a - ref_height for a in valid_altitudes]

        # 打印enu_x, enu_y, enu_z的前10行
        # for i in range(min(10, len(enu_x))):
        #     print(f"Time: {valid_times[i]}, ENU X: {enu_x[i]}, ENU Y: {enu_y[i]}, ENU Z: {enu_z[i]}")
        # 保存valid_times, enu_x, enu_y, enu_z到csv文件
        output_dir = os.path.dirname(filepath)
        output_file = os.path.join(output_dir, 'enu_coordinates.csv')
        with open(output_file, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['Time', 'ENU X', 'ENU Y', 'ENU Z'])
            for time, x, y, z in zip(valid_times, enu_x, enu_y, enu_z):
                csvwriter.writerow([time, x, y, z])

        plot_gps_track(enu_x, enu_y, enu_z)
import csv
import numpy as np
import time
from transforms3d.quaternions import qmult
from transforms3d.taitbryan import euler2quat, quat2euler


# Function to write data from line_ind to a .sto file
def quat2sto_single(sensor_data, sensors, file_dir, t_step, rate):
    num_sensors = len(sensor_data["raw_data"])
    header_text = "\t".join(["time"] + sensors) + "\n" 
    # header_text = "\t".join(["time", "universal_time"] + sensors) + "\n" 
    with open(file_dir, 'w') as f:
        # initial information to write to file
        f.write("DataRate={}\n".format(rate))
        f.write("DataType=Quaternion\n")
        f.write("version=3\n")
        f.write("OpenSimVersion=4.5\n")
        f.write("endheader\n")
        f.write(header_text)
        f.write("{}".format(t_step))
        # f.write("\t"+",".join(str(sensor_data["custom_data"])))
        for sensor in range(len(sensors)):
            f.write("\t"+",".join([str(sensor_data["raw_data"][sensor][f"Quat{i+1}"]) for i in range(4)]))
        f.write("\n")

# Function to read sto file to load fake real time data in numpy array
def sto2quat(file_dir, lines = 3, offset = 6, num_sensors=8):
    sensor_data = np.zeros((num_sensors,lines,4))
    times = np.zeros(lines)
    with open(file_dir, 'r') as f:
        for i in range(lines+offset):
            line = f.readline()
            if i >= offset:
                row_ind = i - offset
                sect_tab = line.split('\t')
                for j, sect in enumerate(sect_tab):
                    if j == 0:
                        times[row_ind] = float(sect)
                    else:
                        subsect = sect.split(',')
                        for k,val in enumerate(subsect):
                            sensor_data[j-1,row_ind,k] = float(val)
    return times, 

def construct_json_object(header, row):
    json_object = {}
    for h, v in zip(header, row):
        # Split the header to check if it ends with a number indicating sensor index
        parts = h.split('_')
        if parts[-1].isdigit():
            # If the header is for a sensor data, nest it under 'SensorIndex_x'
            sensor_index = parts[-1]
            sensor_key = '_'.join(parts[:-1])
            if f"SensorIndex_{sensor_index}" not in json_object:
                json_object[f"SensorIndex_{sensor_index}"] = {}
            json_object[f"SensorIndex_{sensor_index}"][sensor_key] = v
        else:
            json_object[h] = v
    return json_object

# Function to convert CSV to the desired JSON structure
def convert_csv_to_list_of_packets(csv_file_path):
    if not csv_file_path.lower().endswith('.csv'):
        raise ValueError("Wrong file type. This only works with .csv files for now. ")
    list_of_packets = []

    # Retrieve the creation time of the file

    with open(csv_file_path, mode='r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)  # Get the header row

        # Process each row in the CSV
        for row in csv_reader:
            # packet = {"raw_data": [], "custom_data": None}
            packet = {"raw_data": [], "custom_data": []}
            json_object = construct_json_object(header, row)
            
            # Iterate over each possible sensor index
            for i in range(1, 9):  # Assuming there are up to 8 sensors
                sensor_key = f"SensorIndex_{i}"
                if sensor_key in json_object:
                    packet["raw_data"].append(json_object[sensor_key])
            packet["custom_data"] = {'universal_time': json_object['universal_time']}
            if packet["raw_data"]:  # Only add the packet if there is raw data
                list_of_packets.append(packet)
    return list_of_packets

def transform_data(data):
    transform_quat = euler2quat(0,0,-np.pi/2)
    for sensor_idx, sensor in enumerate(data["raw_data"]):
        quat = [float(sensor[f"Quat{i+1}"]) for i in range(4)]
        # if sensor_idx == 2: transform_quat = euler2quat(0,-np.pi/2,-np.pi/2)
        # if sensor_idx == 3: transform_quat = euler2quat(np.pi/2,0,-np.pi/2)
        new_quat = qmult(transform_quat, quat)
        for i in range(4):
            data["raw_data"][sensor_idx][f"Quat{i+1}"] = new_quat[i]
    return data

def calculate_heading_error(data, sensors, transformations):
    angles = np.zeros(len(sensors))
    for sensor, sensor_name in enumerate(sensors):
        quat = [float(data["raw_data"][sensor][f"Quat{i+1}"]) for i in range(4)]
        angles[sensor] = quat2euler(quat)[2]
    return angles.mean()

def parallelIK(ikSolver, s0, ik, time_stamp):
    ikSolver.track(s0)
    ik.put([time.time()-time_stamp])
    time.sleep(0.005)
    





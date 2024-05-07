#!/usr/bin/python3
import opensim as osim
from opensim import Vec3
import numpy as np
from helper import (
    quat2sto_single,
    convert_csv_to_list_of_packets,
    transform_data
)
from DataStreamClient import DataStreamClient
from ConfigManager import Config
import argparse
from multiprocessing import Process, Queue

osim.Logger.setLevelString("Error")

def main(args):
    config = Config(args)
    q = Queue()  # queue for quaternion data

    if not config.offline:
        online_init(args, config, q)
    else:
        offline_init(config, q)

    time_sample, data = q.get()

    data = transform_data(data)

    if len(data["raw_data"]) != len(config.sensors):
        raise ValueError("Not the right number of sensors in the data.")
    quat2sto_single(data, config.sensors, config.sto_filename, time_sample, config.rate)

    model, ikReporter = initalize_model_and_reporter(config)

    s0, ikSolver = initialize_ik(config, model)

    # IK solver loop
    t = 0  # number of steps
    keep_running = True
    if not config.offline:
        while q.qsize() > 0:
            q.get()
    while keep_running:
        try:
            keep_running, t = update(config, q, model, s0, ikSolver, t)
        except KeyboardInterrupt:
            break
    save_kinematics(ikReporter, config)

def offline_init(config, q):
    packets = convert_csv_to_list_of_packets(
            str(config.offline_data_folder / config.offline_data_name)
        )
    for idx, packet in enumerate(packets):
        q.put([idx * 1/config.rate, packet])
    q.put("done")

def online_init(args, config, q):
    fields = ["Sampletime", "Quat1", "Quat2", "Quat3", "Quat4"]
    requested_data = [[i, sensor] for sensor in fields for i in range(len(config.sensors))]
    client = DataStreamClient(args.address, q, requested_data=requested_data)
    process = Process(target=client.run_forever)
    process.start()

def update(config, q, model, s0, ikSolver, t):
    dt = 1 / config.rate
    print(f'Elements in Queue: {q.qsize():05}', end="\r")
    for _ in range(100//config.rate):
        queue_values = q.get()
        if queue_values == "done":
            return False, t
    time_sample, data = queue_values
    data = transform_data(data)
    time_s = t * dt
    quat2sto_single(data, config.sensors, config.sto_filename, time_sample, config.rate)
    quatTable = osim.TimeSeriesTableQuaternion(config.sto_filename)
    orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
    rowVecView = orientationsData.getNearestRow(time_sample)
    ikSolver.updateOrientationData(time_s + dt, rowVecView)
    s0.setTime(time_s + dt)
    ikSolver.track(s0)
    if config.visualize:
        try:
            model.getVisualizer().show(s0)
        except RuntimeError:
            print("It seemed that you closed the visualizer window. Quitting now.")
            return False, t
    model.realizeReport(s0)
    t += 1
    return True, t

def save_kinematics(ikReporter, config):
    columns = list(ikReporter.getTable().getColumnLabels())
    columns.insert(0, 'Time')
    time_col = np.array(ikReporter.getTable().getIndependentColumn())
    angle_data = ikReporter.getTable().getMatrix().to_numpy()
    data = np.column_stack((time_col, angle_data))
    header = ','.join(columns)
    np.savetxt(config.output_filename, data, delimiter=',', header=header, comments='')

def initialize_ik(config, model):
    quatTable = osim.TimeSeriesTableQuaternion(config.sto_filename)
    orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(
        quatTable
    )
    oRefs = osim.BufferedOrientationsReference(orientationsData)
    init_state = model.initSystem()
    mRefs = osim.MarkersReference()
    coordinateReferences = osim.SimTKArrayCoordinateReference()
    if config.visualize:
        model.setUseVisualizer(True)
    model.initSystem()
    s0 = init_state
    ikSolver = osim.InverseKinematicsSolverRT(
        model, mRefs, oRefs, coordinateReferences, config.constraint_var
    )
    ikSolver.setAccuracy = config.accuracy
    s0.setTime(0.0)
    ikSolver.assemble(s0)
    if config.visualize:  # initialize visualization
        model.getVisualizer().show(s0)
        model.getVisualizer().getSimbodyVisualizer().setShowSimTime(False)
    return s0,ikSolver

def initalize_model_and_reporter(config):
    visualize_init = False
    sensor_to_opensim_rotations = Vec3(0, 0, 0)
    imuPlacer = osim.IMUPlacer()
    imuPlacer.set_model_file(str(config.uncal_model_filename))
    imuPlacer.set_orientation_file_for_calibration(config.sto_filename)
    imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
    imuPlacer.set_base_imu_label(config.base_imu)
    imuPlacer.set_base_heading_axis(config.base_imu_axis)
    imuPlacer.run(visualize_init)
    model = imuPlacer.getCalibratedModel()
    model.printToXML(str(config.model_filename))

    # Initialize model
    coordinates = model.getCoordinateSet()
    ikReporter = osim.TableReporter()
    ikReporter.setName("ik_reporter")
    for coord in coordinates:
        if config.log_data:
            ikReporter.addToReport(coord.getOutput("value"), coord.getName())
    model.addComponent(ikReporter)
    model.finalizeConnections()
    return model,ikReporter



if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Estimates kinematics from IMU data using a musculoskeletal model."
    )
    parser.add_argument("--address", type=str, help="IP address (e.g., 192.168.137.1)")
    parser.add_argument(
        "--config",
        type=str,
        help="Full path of config file. If not supplied, it will use the default config file",
    )
    args = parser.parse_args()
    main(args)


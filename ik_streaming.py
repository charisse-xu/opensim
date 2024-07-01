#!/usr/bin/python3
import time
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

def main(args):
    config = Config(args)
    osim.Logger.setLevelString(config.logging_level)
    q = Queue()  # queue for quaternion data

    if not config.offline:
        online_init(args, config, q)
    else:
        offline_init(config, q)

    time_sample, data = q.get()
    # print(data)

    data = transform_data(data)

    if len(data["raw_data"]) != len(config.sensors):
        raise ValueError("Not the right number of sensors in the data.")
    quat2sto_single(data, config.sensors, config.sto_filename, time_sample, config.rate)

    model, ikReporter = initalize_model_and_reporter(config)

    s0, ikSolver = initialize_ik(config, model)

    # IK solver loop
    t = 0  # number of steps why????
    time_IK = 0. # timing simulation
    time_stack = []
    keep_running = True
    start_sim_time = time.time()
    if not config.offline:
        while q.qsize() > 0:
            q.get()
    while keep_running:
        try:
            # print(q.get())
            keep_running, t, time_IK, time_stack = update(config, q, model, s0, ikSolver, t, time_IK, time_stack)
            print("Time used in IK:",time_IK,"Total time:",time.time()-start_sim_time)
        except KeyboardInterrupt:
            break
    save_time(time_stack, config)
    save_kinematics(ikReporter, config)

def offline_init(config, q):
    packets = convert_csv_to_list_of_packets(
            str(config.offline_data_folder / config.offline_data_name)
        )
    for idx, packet in enumerate(packets):
        if idx < 100: continue # 去掉刚开始的不对的数据
        q.put([(idx-100) * 1/config.rate, packet]) # 根据rate得到时间
        # q.put(packet) # packet里面有时间
    q.put("done")

def online_init(args, config, q):
    fields = ["Sampletime", "Quat1", "Quat2", "Quat3", "Quat4"]
    requested_data = [[i, sensor] for sensor in fields for i in range(len(config.sensors))]
    requested_data.append([None, "universal_time"])
    client = DataStreamClient(args.address, q, requested_data=requested_data)
    process = Process(target=client.run_forever)
    process.start()

def update(config, q, model, s0, ikSolver, t, time_IK, time_stack):
    dt = 1 / config.rate
    print(f'Elements in Queue: {q.qsize():05}', end="\r")
    for _ in range(100//config.rate): # ==5, why? 渲染速度
        queue_values = q.get()
        if queue_values == "done":
            return False, t, time_IK, time_stack
    time_sample, data = queue_values
    # data = queue_values
    # print(time_sample) # time_sample === 0？
    add_time = time.time()
    data = transform_data(data)
    collect_time = float(data['custom_data']['universal_time'][0])
    time_s = t * dt # 从0开始，跟time_sample一样？除非去掉了前几(5)个
    # print(time_s)
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
            return False, t, time_IK, time_stack
    model.realizeReport(s0)
    t += 1
    time_IK += time.time() - add_time
    time_delay = time.time() - collect_time # 这个有点问题再看看
    time_stack.append([time_s, time_delay])

    return True, t, time_IK, time_stack


def save_kinematics(ikReporter, config):
    columns = list(ikReporter.getTable().getColumnLabels())
    columns.insert(0, 'Time')
    time_col = np.array(ikReporter.getTable().getIndependentColumn())
    angle_data = ikReporter.getTable().getMatrix().to_numpy()
    data = np.column_stack((time_col, angle_data))
    header = ','.join(columns)
    np.savetxt(config.output_filename, data, delimiter=',', header=header, comments='')

def save_time(time_stack, config):
    data = time_stack
    header = 'sample time,delay time'
    np.savetxt(config.time_filename, data, delimiter=',', header=header, comments='')

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
    sensor_to_opensim_rotations = Vec3(0, 0, 0) ####
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


#!/usr/bin/python3
# Estimates kinematics from IMU data processed into quaternions using a musculoskeletal model
import opensim as osim
from opensim import Vec3
import numpy as np
from helper import (
    quat2sto_single,
    calculate_heading_error,
    convert_csv_to_list_of_packets,
    transform_data
)
from DataStreamClient import DataStreamClient
from ConfigManager import Config
import time
import argparse
from multiprocessing import Process, Queue

osim.Logger.setLevelString("Error")


def main(args):
    config = Config(args)
    script_live = True
    q = Queue()  # queue for quaternion data

    if not config.offline:
        fields = ["Sampletime", "Quat1", "Quat2", "Quat3", "Quat4"]
        requested_data = [[i, sensor] for sensor in fields for i in range(len(config.sensors))]
        client = DataStreamClient(args.address, q, requested_data=requested_data)
        process = Process(target=client.run_forever)
        process.start()
    dt = 1 / config.rate

    while script_live:
        while q.qsize() > 0:
            q.get()
        print("Ready to initialize...")
        if config.offline:
            packets = convert_csv_to_list_of_packets(
                str(config.offline_data / config.offline_data_name)
            )
            for idx, packet in enumerate(packets):
                q.put([idx * dt, packet])
            q.put("done")
        time_sample, data = q.get()
        # calibrate model and save
        data = transform_data(data)
        # data = add_synthetic_pelvis_imu(data, "trunk")
        if len(data["raw_data"]) != len(config.sensors):
            raise ValueError(
                "The number of sensors and the number of sensors in the streamed data don't jive."
            )
        quat2sto_single(data, config.sensors, config.sto_filename, time_sample, config.rate)
        visualize_init = False
        head_err = calculate_heading_error(data, config.sensors, None)
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
        rt_samples = int(10000 * config.rate)
        # kin_mat = np.zeros((rt_samples, 39)) # 39 is the number of joints stored in the .sto files accessible at each time step
        time_vec = np.zeros((rt_samples, 2))
        coordinates = model.getCoordinateSet()
        ikReporter = osim.TableReporter()
        ikReporter.setName("ik_reporter")
        for coord in coordinates:
            if config.log_data:
                ikReporter.addToReport(coord.getOutput("value"), coord.getName())
        model.addComponent(ikReporter)
        model.finalizeConnections

        # Initialize simulation
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

        # IK solver loop
        t = 0  # number of steps
        st = 0.0  # timing simulation
        temp_data = []
        add_time = 0.0
        running = True
        start_sim_time = time.time()
        while q.qsize() > 0:
            q.get()
        while running:
            for _ in range(100//config.rate):
                queue_values = q.get()
                if queue_values == "done":
                    exit()
            time_sample, data = queue_values
            data = transform_data(data)
            # data = add_synthetic_pelvis_imu(data, "trunk")
            add_time = time.time()
            time_s = t * dt
            quat2sto_single(
                data, config.sensors, config.sto_filename, time_sample, config.rate
            ) 
            quatTable = osim.TimeSeriesTableQuaternion(config.sto_filename)
            orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(
                quatTable
            )
            rowVecView = orientationsData.getNearestRow(time_sample)
            ikSolver.updateOrientationData(time_s + dt, rowVecView)
            s0.setTime(time_s + dt)
            ikSolver.track(s0)
            if config.visualize:
                try:
                    model.getVisualizer().show(s0)
                except RuntimeError:
                    print("It seemed that you closed the visualizer window. Quitting now.")
                    exit()
            model.realizeReport(s0)
            rowind = ikReporter.getTable().getRowIndexBeforeTime(
                (t + 1) * dt
            )  # most recent index in kinematics table
            kin_step = (
                ikReporter.getTable().getRowAtIndex(rowind).to_numpy()
            )  # joint angles for current time step as numpy array
            # see the header of the saved .sto files for the names of the corresponding joints.
            ### ADD CUSTOM CODE HERE FOR REAL-TIME APPLICATIONS ###

            st += time.time() - add_time
            time_vec[t, 0] = time_sample
            time_vec[t, 1] = time.time() - time_sample  # delay
            # print("Delay (ms):", 1000.*np.mean(time_vec[t-int(rate):t,1],axis=0))
            t += 1


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

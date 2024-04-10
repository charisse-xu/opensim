#!/usr/bin/python3
# Estimates kinematics from IMU data processed into quaternions using a musculoskeletal model
import opensim as osim
from opensim import Vec3
import numpy as np
from helper import quat2sto_single, sto2quat, calculate_heading_error, convert_csv_to_list_of_packets, transform_data, add_synthetic_pelvis_imu
from DataStreamClient import DataStreamClient
import time
import pathlib
import argparse
import tomli
from multiprocessing import Process, Queue

osim.Logger.setLevelString("Error")

def clear(q):
    try:
        while True:
            q.get_nowait()
    except:
        pass


def main(ws_url):
    
    parser = argparse.ArgumentParser(description="Estimates kinematics from IMU data using a musculoskeletal model.")
    parser.add_argument('--address', type=str, help='IP address (e.g., 192.168.137.1)', required=True)
    parser.add_argument('--config', type=str, help='Full path of config file. If not supplied, it will use the default config file')

    args = parser.parse_args()

    
    with open(args.config or "config.toml", "rb") as f:
        config_data = tomli.load(f)

    real_time = True # set to True for using the kinematics in the python script for real-time applications

    # Parameters for IK solver
    offline = False # True to run offline, False to record data and run online
    log_data = True # if true save all IK outputs, else only use those in reporter_list for easier custom coding
    home_dir = pathlib.Path(__file__).parent.resolve() # location of the main RealTimeKin folder
    uncal_model = 'Rajagopal_2015.osim'
    uncal_model_filename = home_dir / uncal_model
    model_filename = home_dir / ('calibrated_' + uncal_model)
    offline_data = home_dir / 'offline/'#test_data.npy'#'test_IMU_data.npy'#'MT_012005D6_009-001_orientations.sto'
    sto_filename = str(home_dir /'temp_file.sto')
    visualize = True
    rate = 50.0 # samples hz of IMUs
    accuracy = 0.01 # value tuned for accurate and fast IK solver
    constraint_var = 10.0 # value tuned for accurate and fast IK solver
    sensors = config_data["sensors"] # See config.toml for sensor definitions
    base_imu = config_data["base_imu"] # See config.toml for base imu definitions
    base_imu_axis = config_data["base_imu_axis"] # See config.toml for definitions
    offline_data_name = "Full.csv"

    # Initialize the quaternions
    file_cnt = 0
    save_dir_init = home_dir / 'recordings/' # appending folder name here
    save_file = '/recording_'
    ts_file = '/timestamp_'
    script_live = True
    t = 0
    q = Queue() # queue for IMU messages

    if not offline:
        fields = ["Sampletime", "Quat1", "Quat2", "Quat3", "Quat4"]
        requested_data = [[i, sensor] for sensor in fields for i in range(8)]
        client = DataStreamClient(args.address, q, requested_data=requested_data)
        process = Process(target=client.run_forever)
        process.start()
    dt = 1/rate

    while(script_live):
        while(q.qsize()>0): # clearing the queues that may have old messages
            q.get()
        break_timer = 0
        while q.get()[1]["raw_data"][0] == {}:
            q.get()
            break_timer += 1
            if break_timer > 1000:
                exit()
        print("Ready to initialize...")
        if offline:
            packets = convert_csv_to_list_of_packets(str(offline_data/offline_data_name))
            for idx, packet in enumerate(packets):
                q.put([idx*dt,packet])
            q.put("done")
        time_sample, data = q.get(timeout=4)
        # calibrate model and save
        data = transform_data(data)
        # data = add_synthetic_pelvis_imu(data, "trunk")
        if len(data["raw_data"]) != len(sensors):
            raise ValueError("The number of sensors and the number of sensors in the streamed data don't jive.")
        quat2sto_single(data, sensors, sto_filename, time_sample, rate)
        visualize_init = False
        head_err = calculate_heading_error(data, sensors, None)
        sensor_to_opensim_rotations = Vec3(0,0,0)
        imuPlacer = osim.IMUPlacer();
        imuPlacer.set_model_file(str(uncal_model_filename));
        imuPlacer.set_orientation_file_for_calibration(sto_filename);
        imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations);
        imuPlacer.set_base_imu_label(base_imu)
        imuPlacer.set_base_heading_axis(base_imu_axis)
        imuPlacer.run(visualize_init);
        model = imuPlacer.getCalibratedModel();
        model.printToXML(str(model_filename))

        # Initialize model
        rt_samples = int(10000*rate)
        #kin_mat = np.zeros((rt_samples, 39)) # 39 is the number of joints stored in the .sto files accessible at each time step
        time_vec = np.zeros((rt_samples,2))
        coordinates = model.getCoordinateSet()
        ikReporter = osim.TableReporter()
        ikReporter.setName('ik_reporter')
        for coord in coordinates:
            if log_data:
                ikReporter.addToReport(coord.getOutput('value'),coord.getName())
        model.addComponent(ikReporter)
        model.finalizeConnections

        # Initialize simulation
        quatTable = osim.TimeSeriesTableQuaternion(sto_filename)
        orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
        oRefs = osim.BufferedOrientationsReference(orientationsData)
        init_state = model.initSystem()
        mRefs = osim.MarkersReference()
        coordinateReferences = osim.SimTKArrayCoordinateReference()
        if visualize:
            model.setUseVisualizer(True)
        model.initSystem()
        s0 = init_state
        ikSolver = osim.InverseKinematicsSolverRT(model, mRefs, oRefs, coordinateReferences, constraint_var)
        ikSolver.setAccuracy = accuracy
        s0.setTime(0.)
        ikSolver.assemble(s0)
        if visualize: # initialize visualization
            model.getVisualizer().show(s0)
            model.getVisualizer().getSimbodyVisualizer().setShowSimTime(True)

        # IK solver loop
        t = 0 # number of steps
        st = 0. # timing simulation
        temp_data = []
        add_time = 0.
        running = True
        start_sim_time = time.time()
        print("Starting recording...")
        while(running):
            for _ in range(5):
                queue_values = q.get(timeout=2)
                if queue_values == "done":
                    exit()
            time_sample, data = queue_values
            data = transform_data(data)
            # data = add_synthetic_pelvis_imu(data, "trunk")
            add_time = time.time()
            time_s = t*dt
            quat2sto_single(data, sensors, sto_filename,time_sample, rate) # store next line of fake online data to one-line STO
            
            # IK
            quatTable = osim.TimeSeriesTableQuaternion(sto_filename)
            orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
            rowVecView = orientationsData.getNearestRow(time_sample)
            ikSolver.updateOrientationData(time_s+dt, rowVecView)
            s0.setTime(time_s+dt)
            ikSolver.track(s0)
            if visualize:
                model.getVisualizer().show(s0)
            model.realizeReport(s0)
            if real_time: # The previous kinematics are pulled here and can be used to implement any custom real-time applications
                rowind = ikReporter.getTable().getRowIndexBeforeTime((t+1)*dt) # most recent index in kinematics table
                kin_step = ikReporter.getTable().getRowAtIndex(rowind).to_numpy() # joint angles for current time step as numpy array
                # see the header of the saved .sto files for the names of the corresponding joints.
                ### ADD CUSTOM CODE HERE FOR REAL-TIME APPLICATIONS ###

            st += time.time() - add_time
            time_vec[t,0] = time_sample
            time_vec[t,1] = time.time()-time_sample # delay
                #print("Delay (ms):", 1000.*np.mean(time_vec[t-int(rate):t,1],axis=0))
            t += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Estimates kinematics from IMU data using a musculoskeletal model.")
    parser.add_argument('--address', type=str, help='IP address (e.g., 192.168.137.1)', required=True)
    args = parser.parse_args()

    main(args.address)
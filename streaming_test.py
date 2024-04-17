from multiprocessing import Process, Queue
from time import sleep
from DataStreamClient import DataStreamClient

def main():
    q = Queue()  # queue for quaternion data

    try:
        fields = ["Sampletime", "Quat1", "Quat2", "Quat3", "Quat4"]
        requested_data = [[i, sensor] for sensor in fields for i in range(8)]
        client = DataStreamClient("192.168.137.1", q, requested_data=requested_data)
        process = Process(target=client.run_forever)
        process.start()

        while True:
            print(q.qsize())
            sleep(3)

    except KeyboardInterrupt:
        print("Interrupted by user, stopping process...")
        process.terminate()
        process.join()  # Wait for the process to finish
        print("Process stopped.")

if __name__ == "__main__":
    main()
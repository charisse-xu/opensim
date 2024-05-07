# OpenSimRT

This is the repository for doing real-time inverse kinematics (IK) with OpenSim.

A big inspiration for this repository and work is Patrick Slade's work and repo here: https://github.com/pslade2/RealTimeKin

## Requirements

This only works with Python 3.10. You can download python from the [official python website](https://www.python.org/downloads/). If you use Anaconda, you can also create an environment with the required python version.
## Installation

To install OpenSimRT, clone the repository and install the required dependencies.

```bash
git clone https://github.com/RTnhN/OpenSimRT.git
cd OpenSimRT
pip install -r requirements.txt

```

You can download the required OpenSim binaries from [the releases section](https://github.com/RTnhN/opensim-core/releases/tag/4.5.1-alpha) of my [fork of the OpenSim repo](https://github.com/RTnhN/opensim-core). With the fork of the OpenSim repo, you may also compile it yourself using the scripts in the `scripts` directory. The binaries need to be placed in the C:/ directory under the name `opensim-core`. You can install the SDK for OpenSim by running the following commands:

```bash
cd C:/opensim-core/sdk/python
pip install .
``` 

You can then test the installation by running the following commands which will open a python interpreter and import the OpenSim module. If there are no errors, the installation was successful. If there are errors, please check the installation instructions again and make sure that you have the correct version of python installed. If you are still having issues, please open an issue on this repository. The commands are as follows:

``` bash
cd C:/
python
import opensim
```

## Usage

To use OpenSimRT, you can run the following command: 

``` bash
python ik_streaming.py --address 192.168.137.1
```
where `--address` is the address that you are using for connecting to the system.

Configuration of the system can be done in the `config.toml` file. See the `config.toml` file for more information on how to configure the system. Since it is so important, I will specifically mention the order of the sensors in the `toml` file here. The orders of the sensors in the `toml` file should be the same as the order of the sensors streamed from the system.

Make sure that you have the system streaming the data first by running an app. 

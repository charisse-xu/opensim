import tomli
import pathlib

class Config:
    def __init__(self, args):

        with open(args.config or "config.toml", "rb") as f:
            self.data = tomli.load(f)
        self.home_dir = pathlib.Path(__file__).parent.resolve()
        self.initialize()
        self.validate_online_mode(args)

    def initialize(self):
        # Required parameters
        self.validate_required_keys(['offline', 'accuracy', 'constraint_var', 'sensors', 'base_imu', 'base_imu_axis', 'output_filename'])
        self.visualize = self.data["visualize"]
        self.rate = self.data["rate"]
        self.offline = self.data['offline']
        self.accuracy = self.data['accuracy']
        self.constraint_var = self.data['constraint_var']
        self.sensors = self.data['sensors']
        self.base_imu = self.data['base_imu']
        self.base_imu_axis = self.data['base_imu_axis']
        self.uncal_model = "Rajagopal_2015.osim"
        self.uncal_model_filename = self.home_dir / self.uncal_model
        self.model_filename = self.home_dir / ("calibrated_" + self.uncal_model)
        self.offline_data_folder = self.home_dir / "offline/"
        self.sto_filename = str(self.home_dir / "temp_file.sto")
        self.save_dir_init = self.home_dir / "recordings/"
        self.offline_data_name = self.data["offline_data_name"] if self.offline else None
        self.log_data = self.data["log_data"]
        self.output_filename = self.data["output_filename"]

    def validate_required_keys(self, required_keys):
        missing_keys = [key for key in required_keys if key not in self.data]
        if missing_keys:
            raise KeyError(f"Missing required configuration keys: {', '.join(missing_keys)}")

    def validate_online_mode(self, args):
        if not args.address and not self.offline:
            raise ValueError("Online mode requires an address.")
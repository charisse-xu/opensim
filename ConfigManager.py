import tomli
import pathlib

class Config:
    def __init__(self, config_path="config.toml"):
        with open(config_path, "rb") as f:
            self.data = tomli.load(f)
        self.home_dir = pathlib.Path(__file__).parent.resolve()
        self.initialize_defaults()

    def initialize_defaults(self):
        self.offline = self.data.get("offline", False)
        self.log_data = True  # Default value, can also be set from config
        self.uncal_model = "Rajagopal_2015.osim"
        self.uncal_model_filename = self.home_dir / self.uncal_model
        self.model_filename = self.home_dir / ("calibrated_" + self.uncal_model)
        self.offline_data = self.home_dir / "offline/"
        self.sto_filename = str(self.home_dir / "temp_file.sto")
        self.visualize = True
        self.rate = 20.0
        self.accuracy = self.data.get("accuracy", 0.01)  # Default accuracy if not set
        self.constraint_var = self.data.get("constraint_var", [])
        self.sensors = self.data.get("sensors", [])
        self.base_imu = self.data.get("base_imu", "")
        self.base_imu_axis = self.data.get("base_imu_axis", "")
        self.offline_data_name = self.data["offline_data_name"] if self.offline else None
        self.save_dir_init = self.home_dir / "recordings/"

    def validate_online_mode(self, args):
        if not args.address and not self.offline:
            raise ValueError("Online mode requires an address.")
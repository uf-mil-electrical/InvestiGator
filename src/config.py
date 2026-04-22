import pathlib
import tomllib

def load_config():
    config_path = pathlib.Path(__file__).parent.parent / "config.toml"

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found at {config_path}. \nCopy config.example.toml to config.toml and update with appropriate values.")

    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    try:
        config["hardware"]["flight_controller_baud"]
        config["hardware"]["ground_control_baud"]
        config["hardware"]["flight_controller"]
        config["hardware"]["ground_control"]
        config["simulation"]["companion_computer"]
        config["simulation"]["ground_control"]

    except KeyError as e:
        raise KeyError(f"Missing required config value: {e}.\n"
                       "Check that all values from config.example.toml are present in config.toml") from e

    return config
import pathlib
import tomllib

def load_config():
    config_path = pathlib.Path(__file__).parent.parent / "config.toml"

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found at {config_path}. \nCopy config.example.toml to config.toml and update with appropriate values.")

    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    return config
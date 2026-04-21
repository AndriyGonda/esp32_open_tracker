import os

Import("env")

env_file = os.path.join(os.getcwd(), ".env")
if os.path.exists(env_file):
    with open(env_file) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                key, value = line.split("=", 1)
                env.Append(CPPDEFINES=[(key.strip(), f'\\"{value.strip()}\\"')])
                print(f"[load_env] loaded: {key.strip()}")
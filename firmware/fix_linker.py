import os
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()
libmicroros_path = os.path.join(env.subst("$PROJECT_LIBDEPS_DIR"), env["PIOENV"], "micro_ros_arduino", "src", "esp32", "libmicroros.a")

if os.path.exists(libmicroros_path):
    lib_dir = os.path.dirname(libmicroros_path)
    print("✅ Linking libmicroros.a manually:", libmicroros_path)
    env.Append(LIB_PATH=[lib_dir])
    env.Append(LIBS=["microros"])
else:
    print("⚠️ libmicroros.a not found, skipping manual link")

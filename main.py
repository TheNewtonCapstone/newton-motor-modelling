import math
from odrive.enums import AxisState

# the goal is to move the motor to a variety of positions
# while it does that, it should read position, velocity and current values (at a fixed frequency (e.g. 500Hz)); save them to a CSV file along with the time stamp, target position and steps since start


def main():
    import time
    import odrive

    odrv = odrive.find_any()

    odrv.clear_errors()
    odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

    odrv.axis0.controller.input_pos = 0

    # wait for the motor to be ready
    time.sleep(2)

    start_time = time.time()

    def run(current_state):
        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # set target position 
        current_state["target_position"] = (
            np.sin(2 * np.pi * current_state["frequency"] * current_state["time"]).item() * current_state["amplitude"]
        )
        odrv.axis0.controller.input_pos = current_state["target_position"]

        # read position, velocity and current values
        current_state["position"] = odrv.axis0.pos_estimate
        current_state["velocity"] = odrv.axis0.vel_estimate
        current_state["torque"] = odrv.axis0.motor.torque_estimate
        current_state["time"] = time.time() - start_time
        current_state["timestep"] += 1

        # record it to the state, for later saving
        current_state["data"].append(
            [
                current_state["timestep"],
                current_state["time"],
                current_state["target_position"],
                current_state["position"],
                current_state["velocity"],
                current_state["torque"],
                current_state["amplitude"],
                current_state["frequency"],
            ]
        )

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "frequency": 0.0,
        "data": [],
    }

    control_frequency = 400  # 500 Hz

    amplitude = 10  # maximum 9 turns on each side
    amplitude_step = 0.5
    frequency = 10  # 1 Hz
    frequency_step = 0.5

    import numpy as np

    try:
        for a in np.arange(amplitude_step, amplitude, amplitude_step):
            for f in np.arange(frequency_step, frequency, frequency_step):
                print(f"Amplitude: {a}, Frequency: {f}")

                current_state["amplitude"] = a
                current_state["frequency"] = f

                for i in range(0, control_frequency):
                    run(current_state)
                    time.sleep(1.0 / control_frequency)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        with open(f"data_{start_time}.csv", "w") as file:
            for key in current_state.keys():
                if key == "data":
                    continue

                file.write(f"{key},")

            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])

                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE


if __name__ == "__main__":
    main()

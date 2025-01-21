import math
from odrive.enums import AxisState

import threading

lock_state = threading.Lock()


# the goal is to move the motor to a variety of positions
# while it does that, it should read position, velocity and current values (at a fixed frequency (e.g. 500Hz)); save them to a CSV file along with the time stamp, target position and steps since start


def generate_ramp(t, amplitude, phase_time):
    # Create a triangular wave for position (gives ramp in velocity)
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        t = time.time() - start_time
        target_position = generate_ramp(
            t, current_state["amplitude"], current_state["phase_time"]
        )
        odrv.axis0.controller.input_pos = current_state["target_position"]

        # read current time,

        # read position, velocity and current values
        current_state["target_position"] = target_position
        current_state["position"] = odrv.axis0.pos_estimate
        current_state["velocity"] = odrv.axis0.vel_estimate
        current_state["torque"] = odrv.axis0.motor.torque_estimate
        current_state["time"] = t
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
                current_state["phase_time"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "frequency": 0.0,
        "phase_time": 1.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    sending_frequency = 100  # 50 Hz

    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    amplitude = 40  # maximum 9 turns on each side
    amplitude_step = 5
    frequency = 20  # 1 Hz
    frequency_step = 0.1
    phase_times = [0.5, 1.0, 2.0, 4.0]

    import numpy as np

    try:
        for amp in np.arange(0, amplitude, amplitude_step):
            for phase in phase_times:
                lock_state.acquire()
                current_state["amplitude"] = amp
                current_state["phase_time"] = phase

                if lock_state.locked():
                    lock_state.release()

                time.sleep(phase * 5)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()

        if lock_state.locked():
            lock_state.release()

        with open(f"vel_data_{start_time}.csv", "w") as file:
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
from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


from odrive.enums import AxisState
import threading
import time
import odrive
import numpy as np

lock_state = threading.Lock()


def step_function(t, t0, amplitude):
    """Step function at time t0"""
    return amplitude if t >= t0 else 0.0


def ramp_function(t, t0, slope):
    """Ramp function starting at t0"""
    return slope * (t - t0) if t >= t0 else 0.0


def generate_ramp(t, amplitude, phase_time):
    # ramp vel  = 2 * amplitude / phase_time , up and down
    cycle_position = t % phase_time
    half_phase = phase_time / 2

    if cycle_position < half_phase:
        # Ramp up
        return (cycle_position / half_phase) * amplitude
    else:
        # Ramp down
        return (2 - cycle_position / half_phase) * amplitude


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
        lock_state.acquire()

        # read target position
        if odrv.axis0.disarm_reason != 0:
            raise BaseException(str(odrv.axis0.disarm_reason))

        # Calculate current time relative to segment start
        t = time.time() - current_state["segment_start_time"]

        # Generate command based on test type
        if current_state["test_type"] == "step":
            current_state["target_position"] = step_function(
                t, current_state["t0"], current_state["amplitude"]
            )
        elif current_state["test_type"] == "ramp":
            current_state["target_position"] = ramp_function(
                t, current_state["t0"], current_state["velocity"]
            )

        # Set target position
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
                current_state["test_type"],
            ]
        )

        if lock_state.locked():
            lock_state.release()

    current_state = {
        "timestep": 0,
        "time": 0.0,
        "target_position": 0.0,
        "position": 0.0,
        "velocity": 0.0,
        "torque": 0.0,
        "amplitude": 0.0,
        "segment_start_time": start_time,
        "test_type": "none",
        "t0": 0.0,
        "data": [],
    }

    from timer import RepeatedTimer

    control_frequency = 500  # 500 Hz
    rt = RepeatedTimer(1.0 / control_frequency, run, current_state)

    import numpy as np

    try:
        # Step Response Tests
        amplitude = 50
        amplitude_step = 1.0
        step_duration = 2.0  # seconds
        reset_duration = 1.0  # seconds
        for amp in np.arange(0, amplitude, amplitude_step):
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "step"
            current_state["amplitude"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Step
            lock_state.acquire()
            current_state["amplitude"] = amp
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(step_duration)

        # Velocity Ramp Tests
        velocities = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]  # turns/s
        ramp_duration = 2.0  # seconds
        hold_duration = 3.0  # seconds

        for vel in velocities:
            # Reset
            lock_state.acquire()
            current_state["test_type"] = "ramp"
            current_state["velocity"] = 0.0
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(reset_duration)

            # Ramp
            lock_state.acquire()
            current_state["velocity"] = vel
            current_state["segment_start_time"] = time.time()
            current_state["t0"] = 0.0
            if lock_state.locked():
                lock_state.release()
            time.sleep(ramp_duration)

            # Hold velocity
            time.sleep(hold_duration)

    except KeyboardInterrupt:
        print("Cancelled")
    except BaseException as e:
        print("Error: " + str(e))
    finally:
        rt.stop()
        if lock_state.locked():
            lock_state.release()

        with open(f"motor_test_data_{start_time}.csv", "w") as file:
            # Write header
            header = "timestep,time,target_position,position,velocity,torque,amplitude,test_type\n"
            file.write(header)

            # Write data
            for data_row in current_state["data"]:
                string_row = ",".join([str(x) for x in data_row])
                file.write(f"{string_row}\n")

        # stop the motor driver
        odrv.axis0.requested_state = AxisState.IDLE

    main()
    main()
    main()
    main()


import sys
import random
import math
import struct
from controller import Supervisor

def run():
    sup = Supervisor()
    ts = int(sup.getBasicTimeStep())

    n_drones = 4
    X_MIN, X_MAX = -7.0,  7.0
    Y_MIN, Y_MAX = -7.0,  7.0

    root = sup.getRoot()
    field = root.getField('children')

    # Spawn doors
    door_positions = []
    for i in range(n_drones):
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        z = 0.0
        def_name = f"DOOR{i}"
        door_str = f'''
          DEF {def_name} Door {{
            translation {x} {y} {z}
            name "door{i}"
            position 0.0
            minStop 0.0
            maxStop 1.57
          }}
        '''
        field.importMFNodeFromString(-1, door_str)
        door_positions.append((x, y, 1.5))  # Drones hover at 1.5m above door

    # Spawn drones
    drone_defs = []
    for i in range(n_drones):
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        z = 0.5
        def_name = f"DRONE{i}"
        drone_str = f'''
          DEF {def_name} Mavic2Pro {{
            translation {x} {y} {z}
            name "drone{i}"
            controller "drone_controller"
          }}
        '''
        field.importMFNodeFromString(-1, drone_str)
        drone_defs.append(def_name)

    sup.step(ts * 3)  # Allow nodes to be created

    # Assign doors to drones randomly
    assignments = random.sample(range(n_drones), n_drones)
    for drone_idx, door_idx in enumerate(assignments):
        drone_node = sup.getFromDef(drone_defs[drone_idx])
        if drone_node is None:
            print(f"ERROR: Drone {drone_defs[drone_idx]} not found!")
            continue
        emitter = drone_node.getDevice("emitter")
        if emitter is None:
            print(f"ERROR: Drone {drone_defs[drone_idx]} missing emitter device!")
            continue
        target = door_positions[door_idx]
        msg = struct.pack('fff', *target)
        emitter.send(msg)
        print(f"Assigned drone{drone_idx} to door{door_idx} at {target}")

    print("All assignments sent.")
    while sup.step(ts) != -1:
        pass

if __name__ == '__main__':
    run()

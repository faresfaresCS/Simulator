# controllers/drone_spawner/drone_spawner.py - FINAL VERSION

import random
from controller import Supervisor

def run():
    sup = Supervisor()

    # --- CONFIGURATION ---
    n_drones = 5
    n_doors = 5
    X_MIN, X_MAX = -9.0, 9.0
    Y_MIN, Y_MAX = -9.0, 9.0
    root = sup.getRoot()
    children_field = root.getField('children')

    # --- SPAWN DOORS ---
    door_positions = []
    for i in range(n_doors):
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        def_name = f"DOOR{i}"
        door_str = f'DEF {def_name} Door {{ translation {x} {y} 0.0, name "door{i}" }}'
        children_field.importMFNodeFromString(-1, door_str)
        door_positions.append([x, y])

    if not door_positions:
        return

    # --- SPAWN DRONES & ASSIGN TARGETS VIA CONTROLLER ARGS ---
    for i in range(n_drones):
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        z = 0.5
        def_name = f"DRONE{i}"
        
        # Assign a random door to this drone
        target_door = random.choice(door_positions)
        target_x = target_door[0]
        target_y = target_door[1]

        # The supervisor now passes the target as controller arguments and then forgets about the drone.
        drone_str = f'''
          DEF {def_name} Mavic2Pro {{
            translation {x} {y} {z}
            name "drone{i}"
            controller "mavic"
            controllerArgs [ "{target_x}", "{target_y}" ]
            cameraSlot [ Camera {{ width 200, height 120 }} ]
          }}
        '''
        children_field.importMFNodeFromString(-1, drone_str)

    # The supervisor's job is done. It does not need to loop.

if __name__ == '__main__':
    run()
# supervisor_controller.py
import sys
from controller import Supervisor

def run():
    sup = Supervisor()
    ts  = int(sup.getBasicTimeStep())

    # decide how many EXTRA drones to create:
    #  sys.argv[1] will be your controllerArg[0] from the .wbt
    try:
        n_extra = int(sys.argv[1])
    except (IndexError, ValueError):
        n_extra = 10      # fallback default

    # define your floor bounds (make sure your Floor size covers this area)
    X_MIN, X_MAX = -9.0,  9.0
    Y_MIN, Y_MAX = -9.0,  9.0
    
    # get the root (or getFromDef('drone1') if you made drone1 the supervisor)
    root   = sup.getRoot()
    field  = root.getField('children')

    # spawn drones 2 through (1 + n_extra)
    for i in range(2, 2 + n_extra):
        # arrange them however you like; here we spread them on X:
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        z = 0.5
        drone_str = f'''
          Mavic2Pro {{
            translation {x} {y} {z}
            name "drone{i}"
            controller "<none>"
          }}
        '''
        field.importMFNodeFromString(-1, drone_str)
        print(f"Spawned drone{i} at ({x:.2f},{y:.2f},{z:.2f})")

    print(f"Done: spawned {n_extra} extra drones.")
    
    for i in range(2, 2 + n_extra):
        # arrange them however you like; here we spread them on X:
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        z = 0.0   # base of door on the floor
        door_str = f'''
          Door {{
            translation {x} {y} {z}
            name "door{i}"
            position {-2.0}
          }}
        '''
        field.importMFNodeFromString(-1, door_str)
        print(f"Spawned doors{i} at ({x:.2f},{y:.2f},{z:.2f})")

    print(f"Done: spawned {n_extra} extra doors.")

    # standard simulation loop
    while sup.step(ts) != -1:
        pass

if __name__ == '__main__':
    run()

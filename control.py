# goal: derive the blocks position(x_true) to a target position (x_target = 1.0)
# use the control law u = K_p (x_target - x_true)

# notes: data.qpos = gen positions we specified a single slide joint
#        data.qpos also maps to the blocks X coord
#        data.ctrl = the actuator control imports dicates the force output


import mujoco
import mujoco.viewer
import time 
import os 

# pull model and data 
script_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(script_dir, 'block.xml')

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# control parameters proportional controller, have a desired positon to reach 
K_p = 20.0
desired_position = 1.0

# launch passive viewer 
with mujoco.viewer.launch_passive(model, data) as viewer:

        # track sim time
        start_time = time.time()

        while viewer.is_running():
        
            step_start = time.time()

            # proportional control loop w/ error
            current_position = data.qpos

            error = desired_position - current_position

            # control signal force = K_p * error 
            control_force = K_p * error

            #apply the force to our actuator (index = 0 is "slide_motor")
            data.ctrl = control_force

            # step the physics forward 
            mujoco.mj_step(model, data)

            # sync the viewer with the physics data 
            viewer.sync()

            # maintain a clean real time simulation rate
            time_until_next_step = model.opt.timestep - time.time()
            if time_until_next_step > 0: 
                time.sleep(time_until_next_step)






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

# control parameters proportional controller
K_p = 20.0
desired_position = 1.0

# launch passive viewer 
with mujoco.viewer.launch_passive(model, data) as viewer:

        # track sim time
        start_time = time.time()

        while viewer.is_running():
        
            step_start = time.time()

            current_position = data.qpos

            error = desired_position - current_position

            control_force = K_p * error

            data.ctrl = control_force

            mujoco.mj_step(model, data)

            viewer.sync()

            time_until_next_step = model.opt.timestep - time.time()
            if time_until_next_step > 0: 
                time.sleep(time_until_next_step)






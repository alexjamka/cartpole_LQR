import mujoco
import mujoco.viewer
import time 
import os 

model = mujoco.MjModel.from_xml_path("tutorial.xml")

# mjModel contains the model desciption, do not change over time
# ie. model.ngeom = the number of geometrys in the scene 

# makes an error that tells you the allowed names 'green_sphere
# tells what the valid properties are
try: 
    model.geom()
except KeyError as e:
    print(e)

# read green sphere RGB values
model.geom('green_sphere').rgba

# can convert from id to name and back
print('id of "green+sphere": ', model.geom('green+sphere').id)
print('name of geom 1: ', model.geom(1).name)
print('name of body 0: ', model.body(0).name)
# ouputs: geom1: green_sphere, geom0: world

# can use this in python comparisons: 
[model.geom(i).name for i in range(mode.ngeom)]
# outputs a list of geoms: red_box, green_sphere

# mjData contains the state and quantities that depend on it the state
# is made of time, positions, velocities: data.time, data.qpos, data.qvel
# to make new data:
data = mujoco.mjData(model)

# also has the functions of the state: ie. cartesion positions 
print(data.geom.xpos)
# this prints form [[0. 0. 0.]]
#                      [0. 0. 0.]]

# the derived data needs to be propagated: 
mujoco.mj_kinematics(model, data)
print('\nnamed access:\n', data.geom('green_sphere').xpos)


# pos/vel
current_position = data.qpos
current_velocity = data.qvel
# summing junctions 
position_error = target_position - current_position

# controller math expressions gains into algebra
control_signal = (Kp * position_error) - (Kd * current_velocity)

# plant input writing to data.ctrl, into actuator of a plant
# send command to motor (force/toruque)
data.ctrl = control_signal 

# moving the clock, advances by model.opt.timestep
mujoco.mj_step(model, data)




import mujoco
import mujoco.viewer
import time
import numpy as np
import matplotlib.pyplot as plt

xml_string = """
<mujoco model="cart_pole">
    <compiler coordinate="local" inertiafromgeom="true"/>
    
    <worldbody>
        <light directional="true" diffuse=".8 .8 .8" pos="0 0 5" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="10 5 0.1" rgba=".8 .8 .8 1"/>
        
        <body name="cart" pos="0 0 0.2">
            <joint name="slider" type="slide" axis="1 0 0" limited="true" range="-3 3" frictionloss="0.0"/>
            <geom name="cart_geom" type="box" size="0.2 0.15 0.06" rgba="0 0.2 0.8 1" mass="2.0"/>
            
            <body name="pole" pos="0 0 0.06">
                <joint name="hinge" type="hinge" axis="0 1 0" pos="0 0 0" ref="0"/>
                <geom name="pole_geom" type="capsule" fromto="0 0 0 0 0 0.6" size="0.015" rgba="0.9 0.1 0.1 1" mass="0.5"/>
            </body>
        </body>
    </worldbody>

    <actuator>
        <motor name="cart_motor" joint="slider" gear="1" ctrlrange="-100 100" ctrllimited="true"/>
    </actuator>
</mujoco>
"""
# Load the model from the string string
model = mujoco.MjModel.from_xml_string(xml_string)
data = mujoco.MjData(model)

K1 = -3.1623  # p cart
K2 = -5.3667  #  10.0 d cart
K3 = 67.6470   # p pole
K4 = 13.0099    # d pole

disturbance_force = 0.0

time_history = []
cart_history = []
pole_history = []


def my_key_callback(keycode):
    global disturbance_force
    if keycode == 263:   
        disturbance_force = -30.0  
        print("Kicked Left!")
    elif keycode == 262: 
        disturbance_force = 30.0   
        print("Kicked Right!")

with mujoco.viewer.launch_passive(model, data, key_callback=my_key_callback) as viewer:
    while viewer.is_running():
        step_start = time.time()
        
        # 1. Tpull all the data for the cart and the pole
        x = data.qpos[0]
        x_dot = data.qvel[0]
        theta = -data.qpos[1]
        theta_dot = -data.qvel[1]


        K = np.array([K1, K2, K3, K4])
        state = np.array([x, x_dot, theta, theta_dot]) 
        
        u = float(-K @ state)

        #write to the actuator
        data.ctrl[0] = np.clip(u + disturbance_force, -100, 100)
        disturbance_force = 0

        
        time_history.append(data.time)
        cart_history.append(x)
        pole_history.append(theta)

        
        # Step the physics engine forward
        mujoco.mj_step(model, data)

        # Sync the visualizer
        viewer.sync()

        # Keep simulation timing steady
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


print("plotting")
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

#plot position 
ax1.plot(time_history, cart_history, label="Cart Position (x)", color="blue")
ax1.axhline(y=x_target, color="black", linestyle="--", label="Target")
ax1.set_ylabel("Position (meters)")
ax1.legend()
ax1.grid(True)

# Plot Pole Angle
ax2.plot(time_history, pole_history, label="Pole Angle (theta)", color="red")
ax2.axhline(y=theta_target, color="black", linestyle="--", label="Target")
ax2.set_xlabel("Time (seconds)")
ax2.set_ylabel("Angle (radians)")
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()

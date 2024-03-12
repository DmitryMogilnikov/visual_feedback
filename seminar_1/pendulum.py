import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np

# TASKS FROM 01.03.2024:
# 1  set maxTime +
# 2  plots (pos, vel) +
# 3  position control based on p.VELOCITY_CONTROL (proportional regulator)
# 4  position control based on p.TORQUE_CONTROL (PI-regulator)
# 5* compare plots of pybullet and our own odeint and figure out the source of errors and fix it +
# 6* figure out how to add control to our own integration script

dt = 1/240 # pybullet simulation step
q0 = 0.5   # starting position (radian)
maxTime = 5 # 1  set maxTime
currentTime = 0
logTime = np.arange(0.0, maxTime, dt)

Kp = 40  # Пропорциональный коэффициент
Ki = 100    # Интегральный коэффициент
Kd = 1   # Дифференциальный коэффициент

# Инициализируем переменные для PID-регулятора
integral = 0
prev_error = 0

physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
# planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("seminar_1/simple.urdf", useFixedBase=True,)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
# p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
# for _ in range(1000):
#     p.stepSimulation()

# turn off the motor for the free motion
# p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

# 3  position control based on p.VELOCITY_CONTROL (proportional regulator)
# p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=-20*np.sin(q0), controlMode=p.VELOCITY_CONTROL, force=0)

jointPositions = []
jointVelocities = []

# for i in range(logTime.shape[0]):
#     p.stepSimulation()
#     time.sleep(dt)

#     jointState = p.getJointState(bodyUniqueId=boxId, jointIndex=1)
#     jointPositions.append(jointState[0])
#     jointVelocities.append(jointState[1])


# 4  position control based on p.TORQUE_CONTROL (PI-regulator)
for i in range(logTime.shape[0]):
    jointState = p.getJointState(bodyUniqueId=boxId, jointIndex=1)
    jointPosition = jointState[0]
    target_q = np.pi
    error = target_q - jointPosition 
    integral += error * dt
    torque = Kp * error + Ki * integral + Kd * (error - prev_error) / dt
    
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, force=torque, controlMode=p.TORQUE_CONTROL)
    p.stepSimulation()
    prev_error = error
    # time.sleep(dt)

    jointPositions.append(jointPosition)


# np.save(rf"seminar_1\result\array\09_pybullet_theta_torque_control.npy", jointPositions)
# np.save(rf"seminar_1\result\array\10_pybullet_omega_torque_control.npy", jointVelocities)

# 2  plots (pos, vel)
plt.grid()
plt.plot(logTime, jointPositions, label = "pybullet_theta")
plt.title("reverse_pendulum")
plt.legend()
plt.savefig(rf"seminar_1\result\plot\09_pybullet_theta_torque_control.png")
plt.show()

# plt.grid()
# plt.plot(logTime, jointVelocities, label = "pybullet_omega")
# plt.title("reverse_pendulum")
# plt.legend()
# plt.savefig(rf"seminar_1\result\plot\10_pybullet_omega_torque_control.png")
# plt.show()

# while currentTime < maxTime:
#     p.stepSimulation()
#     time.sleep(dt)
#     currentTime += dt
p.disconnect()
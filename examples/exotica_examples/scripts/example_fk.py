#!/usr/bin/env python

import pyexotica as exo

solver = exo.Setup.load_solver(
    '{exotica_examples}/resources/configs/ik_solver_demo.xml')
scene = solver.get_problem().get_scene()

# Update the scene before calling FK or Jacobian!
scene.update([0]*7)
print("FK for the end effector:")
print(scene.fk('lwr_arm_7_link'))

print("Jacobian for the end effector:")
print(scene.jacobian('lwr_arm_7_link'))

print("FK for a relative frame:")
scene.update([0.5]*7)
print(scene.fk('lwr_arm_7_link', 'lwr_arm_3_link'))
print("FK for a relative frame with custom offsets:")
print(scene.fk('lwr_arm_7_link', exo.KDLFrame(
    [0, 0, 0.2, 0, 0, 0, 1]), 'lwr_arm_3_link', exo.KDLFrame()))

# FK and Jacobian have the same overloads

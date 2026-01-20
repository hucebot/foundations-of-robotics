from __future__ import annotations
import time
import numpy as np
import viser
import yourdfpy
import ik as inverse_kinematics
from scipy.spatial.transform import Rotation as R
import threading
import viser_utils
from viser.extras import ViserUrdf


def main(load_meshes: bool = True, load_collision_meshes: bool = False) -> None:
    server = viser.ViserServer()

    urdf = yourdfpy.URDF.load("panda.urdf")
    viser_urdf = ViserUrdf(server, urdf_or_path=urdf, load_meshes=load_meshes, load_collision_meshes=load_collision_meshes, collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5))

    q0 = np.array([0., -0.7, 0., -2.1, 0., 1.4, 0.])
    q = q0.copy()

    ik = inverse_kinematics.inverse_kinematics(urdf.write_xml_string(), q)
    ik.update(q)

    T0 = ik.model.getPose("fp3_link8")
    Tref = T0.copy()
    quat = R.from_matrix(T0.linear).as_quat()  # x, y, z, w

    qref = q0.copy()

    viser_urdf.update_cfg(q0)

    # shared state container and lock
    lock = threading.Lock()
    # store a Pose-like object (assumed copyable)
    shared = {'Tref': Tref.copy(), 'qref': qref.copy(), 'reset_flag': False, 'reg': 1e-4}

    # interactive marker for Cartesian task
    target = server.scene.add_transform_controls("/ik_target", position=(T0.translation[0], T0.translation[1], T0.translation[2]), wxyz=(quat[3], quat[0], quat[1], quat[2]), scale=0.5)
    target.on_update(viser_utils.on_target_move(target, shared, lock))

    # sliders for postural task
    with server.gui.add_folder("Postural position control"):
        sliders = viser_utils.create_robot_control_sliders(server, viser_urdf, q0)
        for i in range(len(sliders)):
            sliders[i].on_update(viser_utils.make_joint_slider_callback(i, shared, lock, sliders[i]))
        posture_reset_button = server.gui.add_button("Reset")
        posture_reset_button.on_click(viser_utils.posture_reset_on_click(shared, lock, sliders, q0))

    # slider for regularization
    reg_slider = viser_utils.create_regularization_slider(server, initial_value=shared['reg'])
    reg_slider.on_update(viser_utils.make_regularization_slider_callback(shared, lock, reg_slider))

    # button to reset the IK
    reset_button = server.gui.add_button("Reset IK")
    reset_button.on_click(viser_utils.reset_button_on_click(shared, lock))

    trimesh_scene = viser_urdf._urdf.scene or viser_urdf._urdf.collision_scene
    server.scene.add_grid("/grid", width=2, height=2, position=(0.0, 0.0, trimesh_scene.bounds[0, 2] if trimesh_scene is not None else 0.0))

    control_dt = 0.001  # control loop dt

    q_plot = viser_utils.joint_plot(title="Joint positions", size=ik.model.getNq(), legend_label="q", server=server, dt=control_dt)
    dq_plot = viser_utils.joint_plot(title="Joint velocities", size=ik.model.getNv(), legend_label="dq", server=server, dt=control_dt)

    print("Open your browser to http://localhost:8080")
    print("Press Ctrl+C to exit")
    while True:
        t0 = time.time()

        # Copy shared state quickly under lock
        with lock:
            local_Tref = shared['Tref']  # dict with translation and rotation_matrix
            local_qref = shared['qref'].copy()
            reg = shared['reg']
            do_reset = shared['reset_flag']
            if do_reset:
                shared['reset_flag'] = False

        if do_reset:
            local_Tref.translation = np.array([T0.translation[0], T0.translation[1], T0.translation[2]])
            local_Tref.linear = T0.linear.copy()
            local_qref = q0.copy()
            reg = 1e-3
            q = q0.copy()

            quat = R.from_matrix(T0.linear).as_quat()
            target.position = (T0.translation[0], T0.translation[1], T0.translation[2])
            target.wxyz = (quat[3], quat[0], quat[1], quat[2])
            # update sliders visually
            for s, init_q in zip(sliders, q0):
                s.value = init_q

        pose_like = ik.model.getPose("fp3_link8")  # create a fresh pose-like object base
        pose_like.translation = local_Tref.translation.copy()
        pose_like.linear = local_Tref.linear.copy()

        ik.pose_ref = pose_like
        ik.q_ref = local_qref

        ik.update(q)
        dq = ik.solve(reg=reg)
        q = q + dq

        viser_urdf.update_cfg(q)

        q_plot.update(q)
        dq_plot.update(dq)

        elapsed = time.time() - t0
        sleep_time = max(0.0, control_dt - elapsed)
        time.sleep(sleep_time)

if __name__ == "__main__":
    main()

import viser
from viser.extras import ViserUrdf
import numpy as np
from xbot2_interface import Affine3
from scipy.spatial.transform import Rotation as R

def create_robot_control_sliders(server: viser.ViserServer, viser_urdf: ViserUrdf, initial_pos):
    slider_handles: list[viser.GuiInputHandle[float]] = []
    i = 0
    for joint_name, (lower, upper,) in viser_urdf.get_actuated_joint_limits().items():
        lower = lower if lower is not None else -np.pi
        upper = upper if upper is not None else np.pi
        slider = server.gui.add_slider(label=joint_name, min=lower, max=upper, step=1e-3, initial_value=initial_pos[i])
        slider_handles.append(slider)
        i += 1
    return slider_handles

def make_joint_slider_callback(idx, shared, lock, slider):
    def _(_):
        with lock:
            shared['qref'][idx] = slider.value
    return _


def create_regularization_slider(server: viser.ViserServer, initial_value: float = 0.001):
    slider = server.gui.add_slider(label="IK Regularization", min=1e-4, max=0.1, step=1e-4, initial_value=initial_value)
    return slider

def make_regularization_slider_callback(shared, lock, slider):
    def _(_):
        with lock:
            shared['reg'] = slider.value
    return _

def on_target_move(target, shared, lock):
    def _(_):
        p = target.position
        o = target.wxyz

        new_T = Affine3()
        new_T.translation = np.array([p[0], p[1], p[2]])
        r = R.from_quat([o[1], o[2], o[3], o[0]])  # scipy order x,y,z,w -> we built w,x,y,z
        new_T.linear = r.as_matrix()
        with lock:
            # overwrite the shared Tref representation
            shared['Tref'] = new_T
    return _

def posture_reset_on_click(shared, lock, sliders, q0):
    def _(_):
        with lock:
            shared['qref'] = q0.copy()
        for s, init_q in zip(sliders, q0):
            s.value = init_q
    return _

def reset_button_on_click(shared, lock):
    def _(_):
        with lock:
            shared['reset_flag'] = True
    return _


class joint_plot:
    def __init__(self, title, size, legend_label, server, dt, max_len=1000):
        self.size = size
        self.max_len = max_len
        self.dt = dt

        # One shared time axis
        self.x = self.dt * np.arange(max_len, dtype=np.float64)

        # Y series: one array per joint
        self.ys = [np.zeros(max_len, dtype=float) for _ in range(size)]

        # uPlot data format: (x, y0, y1, ..., yN)
        self.data = (self.x, *self.ys)

        # Series definition: one series per array in data

        self.series = (
            viser.uplot.Series(label="t"),
            *[viser.uplot.Series(label=f"{legend_label}{i}", stroke=self.make_color(i, size), width=2) for i in range(size)])

        self.plot_handle = server.gui.add_uplot(
            data=self.data,
            series=self.series,
            title=title,
            scales={"x": viser.uplot.Scale(time=False, auto=True), "y": viser.uplot.Scale(auto=True)},
            legend=viser.uplot.Legend(show=True),
            aspect=2.0,
        )

    def make_color(self, i, n):
        # HSV equally spaced around the hue circle
        h = i / n
        s = 0.65
        v = 0.85
        import colorsys
        r, g, b = colorsys.hsv_to_rgb(h, s, v)
        return f"rgb({int(r * 255)}, {int(g * 255)}, {int(b * 255)})"

    def update(self, new_data):
        # Append new data
        self.x = np.roll(self.x, -1)
        self.x[-1] = self.x[-2] + self.dt

        for i in range(self.size):
            self.ys[i] = np.roll(self.ys[i], -1)
            self.ys[i][-1] = new_data[i]

        self.plot_handle.data = (self.x.tolist(), *[y.tolist() for y in self.ys])

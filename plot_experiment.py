#!/usr/bin/env python3
"""Generate PNG graphs and a PDF report from one replay's recorded telemetry."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from scipy.spatial.transform import Rotation


def plot_experiment(output):
    output = Path(output)
    with np.load(output / 'telemetry.npz', allow_pickle=False) as saved:
        data = dict(saved)
    result = json.loads((output / 'result.json').read_text())
    names = data['joint_names'].tolist()
    time = data['time']
    if len(time) == 0 or np.any(np.diff(time) < 0):
        raise ValueError('Telemetry needs samples in increasing simulation time')
    robot = ET.parse(output / 'robot.urdf')
    effort_limits = np.array([float(robot.find(f"joint[@name='{name}']/limit").get('effort'))
                              for name in names])
    velocity_limits = np.array([float(robot.find(f"joint[@name='{name}']/limit").get('velocity'))
                                for name in names])
    plots = output / 'plots'
    plots.mkdir(exist_ok=True)
    duration = result['trajectory_seconds']
    context = (f"{result['payload_kg']:g} kg payload · {result['robot_mass_kg']:.2f} kg total · "
               f"{result['outcome'].replace('_', ' ')}")
    plt.rcParams.update({'font.size': 9, 'axes.spines.top': False,
                         'axes.spines.right': False, 'grid.alpha': .2})

    def decorate(axis, ylabel):
        axis.grid(True)
        axis.set_ylabel(ylabel)
        axis.set_xlim(time[0], time[-1])
        axis.axvspan(time[0], min(0., time[-1]), color='.8', alpha=.25)
        if time[-1] > duration:
            axis.axvspan(duration, time[-1], color='.8', alpha=.25)
        axis.axvline(0., color='.4', linewidth=.7)
        axis.axvline(duration, color='.4', linewidth=.7)

    with PdfPages(plots / 'report.pdf') as pdf:
        def save(figure, filename, title):
            figure.suptitle(f'{title}\n{context}', fontsize=13)
            figure.supxlabel('Simulation time from motion start [s] · shaded regions: posture hold')
            figure.tight_layout(rect=(0, .025, 1, .94))
            figure.savefig(plots / f'{filename}.png', dpi=150, bbox_inches='tight')
            pdf.savefig(figure, bbox_inches='tight')
            plt.close(figure)

        for key, ref, scale, label, title, limits in (
            ('torque', None, 1., 'Torque [Nm]', 'Applied joint torques', effort_limits),
            ('v', 'v_ref', 1., 'Velocity [rad/s]', 'Joint velocities', velocity_limits),
            ('q', 'q_ref', 180 / np.pi, 'Position [deg]', 'Joint positions', None),
        ):
            figure, axes = plt.subplots(4, 3, figsize=(14, 10), sharex=True)
            for index, axis in enumerate(axes.flat):
                axis.plot(time, data[key][:, index] * scale, color='#167b9b',
                          linewidth=1., label='Applied' if key == 'torque' else 'Measured')
                if ref:
                    axis.plot(time, data[ref][:, index] * scale, '--', color='#e58b2a',
                              linewidth=1., label='Replay reference')
                if limits is not None:
                    for sign in (-1, 1):
                        axis.axhline(sign * limits[index] * scale, color='#bc3e43',
                                     linestyle=':', linewidth=.9,
                                     label='URDF limit' if sign == 1 else None)
                axis.set_title(names[index].replace('_', ' '))
                decorate(axis, label)
            axes[0, 0].legend(fontsize=8, loc='upper right')
            save(figure, {'torque': 'torques', 'v': 'velocities', 'q': 'positions'}[key], title)

        figure, axes = plt.subplots(4, 3, figsize=(14, 10), sharex=True)
        for index, axis in enumerate(axes.flat):
            axis.plot(time, np.rad2deg(data['q'][:, index] - data['q_ref'][:, index]),
                      color='#167b9b', linewidth=1.)
            axis.axhline(0, color='.4', linewidth=.7)
            axis.set_title(names[index].replace('_', ' '))
            decorate(axis, 'Error [deg]')
        save(figure, 'tracking_errors', 'Joint position errors: measured minus replay reference')

        figure, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True)
        measured_angles = np.rad2deg(np.unwrap(
            Rotation.from_quat(data['base_quaternion']).as_euler('xyz'), axis=0))
        desired_angles = np.rad2deg(np.unwrap(
            Rotation.from_quat(data['base_quaternion_ref']).as_euler('xyz'), axis=0))
        for index in range(3):
            for row, measured, desired, labels, unit in (
                (0, data['base_position'], data['base_ref'], ('Base x', 'Base y', 'Base height'), 'Position [m]'),
                (1, measured_angles, desired_angles, ('Base roll', 'Base pitch', 'Base yaw'), 'Angle [deg]'),
            ):
                axis = axes[row, index]
                axis.plot(time, measured[:, index], color='#167b9b', label='Measured')
                axis.plot(time, desired[:, index], '--', color='#e58b2a', label='Replay reference')
                axis.set_title(labels[index])
                decorate(axis, unit)
        axes[0, 0].legend()
        save(figure, 'base_motion', 'Base position and attitude')

        power = data['torque'] * data['v']
        figure, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
        for leg, axis in enumerate(axes.flat):
            for joint in range(3):
                index = leg * 3 + joint
                axis.plot(time, power[:, index], linewidth=.9,
                          label=names[index].rsplit('_', 1)[0].replace('_', ' '))
            axis.set_title(f'Leg {leg + 1}')
            decorate(axis, 'Joint mechanical power [W]')
        axes[0, 0].legend()
        save(figure, 'mechanical_power', 'Joint mechanical power: applied torque × measured velocity')
    print(f'Graphs saved to {plots} (PNG images and report.pdf)', flush=True)
    return plots


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='Experiment directory containing telemetry.npz')
    plot_experiment(parser.parse_args().output)

"""doodle-bot: SO-101 arm as a pen plotter.

Layers, bottom to top:

  servo       raw Feetech STS3215 bus protocol (ping/read/write/sync)
  arm         named joints, ticks <-> radians, safety limits, sim backend
  kinematics  SO-101 planar FK/IK with a pen tool
  calibration joint zero offsets + paper frame, touch-off refinement fit
  shapes      Drawing / Stroke model and the test-asset generators
  planner     Drawing -> toolpath -> timed joint trajectory
  executor    streams a trajectory to the arm with monitoring and abort
  preview     renders toolpaths to PNG for review before drawing
  cli         the `doodle` command
"""
__version__ = "0.1.0"

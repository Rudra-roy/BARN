"""Offline RL training for the BARN end-to-end and hybrid policies.

This package is deliberately independent of ROS: the fast 2-D trainer runs
without a ROS installation. Trained policies are exported to ONNX and consumed
at runtime by the ROS package ``barn_rl_runtime``.
"""

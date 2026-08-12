"""
test_transformations.py

Unit tests for kinematics/transformations.py.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import pytest

from kinematics import transformations as tf


def test_rotation_z_90_degrees():
    R = tf.rotation_z(np.pi / 2)
    # Rotating the X axis by 90 deg about Z should give the Y axis.
    result = R @ np.array([1, 0, 0])
    assert np.allclose(result, [0, 1, 0], atol=1e-9)


def test_rotation_x_90_degrees():
    R = tf.rotation_x(np.pi / 2)
    result = R @ np.array([0, 1, 0])
    assert np.allclose(result, [0, 0, 1], atol=1e-9)


def test_rotation_y_90_degrees():
    R = tf.rotation_y(np.pi / 2)
    result = R @ np.array([0, 0, 1])
    assert np.allclose(result, [1, 0, 0], atol=1e-9)


def test_rotation_matrices_are_orthonormal():
    for angle in [0.0, 0.3, 1.5, np.pi, -2.1]:
        for R in [tf.rotation_x(angle), tf.rotation_y(angle), tf.rotation_z(angle)]:
            assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
            assert np.isclose(np.linalg.det(R), 1.0, atol=1e-9)


def test_rotation_about_axis_matches_principal_axes():
    angle = 0.7
    assert np.allclose(
        tf.rotation_about_axis([1, 0, 0], angle), tf.rotation_x(angle), atol=1e-9
    )
    assert np.allclose(
        tf.rotation_about_axis([0, 1, 0], angle), tf.rotation_y(angle), atol=1e-9
    )
    assert np.allclose(
        tf.rotation_about_axis([0, 0, 1], angle), tf.rotation_z(angle), atol=1e-9
    )


def test_rotation_about_axis_rejects_zero_axis():
    with pytest.raises(ValueError):
        tf.rotation_about_axis([0, 0, 0], 1.0)


def test_translation_matrix():
    T = tf.translation(1.0, 2.0, 3.0)
    assert np.allclose(T[:3, 3], [1.0, 2.0, 3.0])
    assert np.allclose(T[:3, :3], np.eye(3))
    assert np.allclose(T[3, :], [0, 0, 0, 1])


def test_homogeneous_transform_composition():
    R = tf.rotation_z(np.pi / 2)
    p = np.array([1.0, 0.0, 0.0])
    T = tf.homogeneous_transform(R, p)
    assert np.allclose(tf.extract_rotation(T), R)
    assert np.allclose(tf.extract_position(T), p)
    assert tf.is_valid_transform(T)


def test_is_valid_transform_rejects_bad_matrix():
    bad = np.eye(4)
    bad[3, 3] = 2.0  # invalid bottom row
    assert not tf.is_valid_transform(bad)

    bad2 = np.eye(4)
    bad2[:3, :3] = np.array([[2, 0, 0], [0, 1, 0], [0, 0, 1]])  # not orthonormal
    assert not tf.is_valid_transform(bad2)


def test_chained_transforms_compose_correctly():
    # Two successive translations should add.
    T1 = tf.translation(1, 0, 0)
    T2 = tf.translation(0, 1, 0)
    T = T1 @ T2
    assert np.allclose(tf.extract_position(T), [1, 1, 0])

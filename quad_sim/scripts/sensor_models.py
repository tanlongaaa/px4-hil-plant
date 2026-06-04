#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import numpy as np
from pymavlink import mavutil
from tf.transformations import quaternion_from_euler

from plant_6dof import rot_enu_from_flu


EARTH_RADIUS_M = 6378137.0


def now_usec():
    return time.monotonic_ns() // 1000


def clamp_int16(value):
    return int(max(-32768, min(32767, round(value))))


def enu_to_ned(vec_enu):
    return np.array([vec_enu[1], vec_enu[0], -vec_enu[2]], dtype=float)


def ned_to_enu(vec_ned):
    return np.array([vec_ned[1], vec_ned[0], -vec_ned[2]], dtype=float)


def enu_position_to_wgs84(p_enu, origin_lat_deg, origin_lon_deg, origin_alt_m):
    lat0 = math.radians(origin_lat_deg)
    d_north = float(p_enu[1])
    d_east = float(p_enu[0])

    lat = origin_lat_deg + math.degrees(d_north / EARTH_RADIUS_M)
    lon = origin_lon_deg + math.degrees(d_east / (EARTH_RADIUS_M * max(math.cos(lat0), 1.0e-6)))
    alt = origin_alt_m + float(p_enu[2])
    return lat, lon, alt


class HilSensorModel:
    """
    Convert Quad6DOFPlant state into MAVLink HIL messages.

    Plant frame convention:
      world ENU, body FLU.

    MAVLink HIL convention:
      local velocity NED, body FRD. PX4's HIL_SENSOR expects IMU axes in body FRD,
      so body y/z signs are flipped from FLU.
    """

    HIL_SENSOR_UPDATED_ALL = 0x1FFF

    def __init__(self, config=None):
        config = config or {}
        origin = config.get("origin", {})

        self.origin_lat_deg = float(origin.get("lat_deg", 47.397742))
        self.origin_lon_deg = float(origin.get("lon_deg", 8.545594))
        self.origin_alt_m = float(origin.get("alt_m", 488.0))

        self.pressure_msl_hpa = float(config.get("pressure_msl_hpa", 1013.25))
        self.temperature_c = float(config.get("temperature_c", 20.0))
        self.gyro_dither_rad_s = float(config.get("gyro_dither_rad_s", 5.0e-4))
        self.mag_dither_gauss = float(config.get("mag_dither_gauss", 1.0e-4))
        self.pressure_dither_hpa = float(config.get("pressure_dither_hpa", 0.02))
        self.mag_gauss_ned = np.array(
            config.get("mag_gauss_ned", [0.215, 0.0, 0.427]),
            dtype=float
        )
        self.mag_yaw_offset_rad = float(config.get("mag_yaw_offset_rad", math.pi))

        self.eph_cm = int(config.get("eph_cm", 80))
        self.epv_cm = int(config.get("epv_cm", 120))
        self.satellites_visible = int(config.get("satellites_visible", 12))

    @staticmethod
    def _body_flu_to_frd(vec_flu):
        return np.array([vec_flu[0], -vec_flu[1], -vec_flu[2]], dtype=float)

    @staticmethod
    def _dither_vec(time_us, amplitude, freqs_hz, phases_rad):
        if amplitude <= 0.0:
            return np.zeros(3, dtype=float)

        t = float(time_us) * 1.0e-6
        return np.array([
            amplitude * math.sin(2.0 * math.pi * freqs_hz[0] * t + phases_rad[0]),
            amplitude * math.sin(2.0 * math.pi * freqs_hz[1] * t + phases_rad[1]),
            amplitude * math.sin(2.0 * math.pi * freqs_hz[2] * t + phases_rad[2]),
        ], dtype=float)

    def hil_sensor(self, plant):
        st = plant.get_state()
        time_us = now_usec()

        accel_frd = self._body_flu_to_frd(plant.get_specific_force_body_flu())
        gyro_frd = self._body_flu_to_frd(st["omega_flu"])
        gyro_frd += self._dither_vec(
            time_us,
            self.gyro_dither_rad_s,
            (17.0, 23.0, 29.0),
            (0.0, 1.7, 3.1)
        )
        c = math.cos(self.mag_yaw_offset_rad)
        s = math.sin(self.mag_yaw_offset_rad)
        mag_ned = np.array([
            c * self.mag_gauss_ned[0] - s * self.mag_gauss_ned[1],
            s * self.mag_gauss_ned[0] + c * self.mag_gauss_ned[1],
            self.mag_gauss_ned[2],
        ], dtype=float)
        mag_enu = ned_to_enu(mag_ned)
        mag_flu = rot_enu_from_flu(st["roll"], st["pitch"], st["yaw"]).T @ mag_enu
        mag_frd = self._body_flu_to_frd(mag_flu)
        mag_frd += self._dither_vec(
            time_us,
            self.mag_dither_gauss,
            (7.0, 11.0, 13.0),
            (0.3, 2.1, 4.2)
        )

        altitude_m = float(st["p_enu"][2])
        abs_pressure = self.pressure_msl_hpa * math.pow(
            max(0.01, 1.0 - altitude_m / 44330.0),
            5.255
        )
        abs_pressure += self.pressure_dither_hpa * math.sin(
            2.0 * math.pi * 5.0 * float(time_us) * 1.0e-6
        )

        return mavutil.mavlink.MAVLink_hil_sensor_message(
            time_us,
            float(accel_frd[0]),
            float(accel_frd[1]),
            float(accel_frd[2]),
            float(gyro_frd[0]),
            float(gyro_frd[1]),
            float(gyro_frd[2]),
            float(mag_frd[0]),
            float(mag_frd[1]),
            float(mag_frd[2]),
            float(abs_pressure),
            0.0,
            float(altitude_m),
            float(self.temperature_c),
            self.HIL_SENSOR_UPDATED_ALL
        )

    def hil_gps(self, plant):
        st = plant.get_state()
        time_us = now_usec()

        lat_deg, lon_deg, alt_m = enu_position_to_wgs84(
            st["p_enu"],
            self.origin_lat_deg,
            self.origin_lon_deg,
            self.origin_alt_m
        )

        v_ned = enu_to_ned(st["v_enu"])
        vel_ms = float(np.linalg.norm(v_ned))
        cog_rad = math.atan2(float(v_ned[1]), float(v_ned[0]))
        cog_cdeg = int((math.degrees(cog_rad) * 100.0) % 36000.0) if vel_ms > 0.05 else 65535

        return mavutil.mavlink.MAVLink_hil_gps_message(
            time_us,
            3,
            int(round(lat_deg * 1.0e7)),
            int(round(lon_deg * 1.0e7)),
            int(round(alt_m * 1000.0)),
            self.eph_cm,
            self.epv_cm,
            int(round(vel_ms * 100.0)),
            int(round(v_ned[0] * 100.0)),
            int(round(v_ned[1] * 100.0)),
            int(round(v_ned[2] * 100.0)),
            cog_cdeg,
            self.satellites_visible
        )

    def hil_state_quaternion(self, plant):
        st = plant.get_state()
        time_us = now_usec()

        # Convert ENU/FLU roll-pitch-yaw into NED/FRD attitude used by MAVLink.
        q_enu_flu = quaternion_from_euler(st["roll"], st["pitch"], st["yaw"])
        q = [
            float(q_enu_flu[3]),
            float(q_enu_flu[0]),
            float(-q_enu_flu[1]),
            float(-q_enu_flu[2]),
        ]

        lat_deg, lon_deg, alt_m = enu_position_to_wgs84(
            st["p_enu"],
            self.origin_lat_deg,
            self.origin_lon_deg,
            self.origin_alt_m
        )
        v_ned = enu_to_ned(st["v_enu"])
        accel_frd = self._body_flu_to_frd(plant.get_specific_force_body_flu())
        gyro_frd = self._body_flu_to_frd(st["omega_flu"])

        return mavutil.mavlink.MAVLink_hil_state_quaternion_message(
            time_us,
            q,
            float(gyro_frd[0]),
            float(gyro_frd[1]),
            float(gyro_frd[2]),
            int(round(lat_deg * 1.0e7)),
            int(round(lon_deg * 1.0e7)),
            int(round(alt_m * 1000.0)),
            int(round(v_ned[0] * 100.0)),
            int(round(v_ned[1] * 100.0)),
            int(round(v_ned[2] * 100.0)),
            0,
            0,
            clamp_int16(accel_frd[0] / 9.80665 * 1000.0),
            clamp_int16(accel_frd[1] / 9.80665 * 1000.0),
            clamp_int16(accel_frd[2] / 9.80665 * 1000.0),
        )

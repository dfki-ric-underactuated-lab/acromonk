import numpy as np


class StateEstimator:
    def __init__(self, pr):
        self.pr = pr

    async def get_shoulder_state(self, brach_sign, th_2, th_2_vel):
        # brach_sign determines which arm is the support arm
        th_1 = None
        th_1_dot = None
        omega_x = None

        quat_wxyz, vel_xyz, acc_xyz, euler_xyz = await self.read_imu_data()

        th_imu = euler_xyz[0]
        omega_x, _, _ = np.deg2rad(vel_xyz)

        if brach_sign == 1:  # odd
            # get shoulder angle
            th_imu_0_to_2pi = th_imu + np.pi
            th_2_0_to_2pi = th_2 + np.pi
            th_1 = np.mod(np.pi - th_2_0_to_2pi + th_imu_0_to_2pi, 2 * np.pi) - np.pi

            # get shoulder velocity
            th_1_dot = omega_x - th_2_vel

        elif brach_sign == -1:  # even
            th_1 = self.wrap_pi2pi(np.pi - euler_xyz[0])
            th_1 *= -1.0
            th_1_dot = omega_x

        return th_1, th_1_dot, th_imu, omega_x

    async def read_imu_data(self):
        imu = await self.pr.cycle([], request_attitude=True)
        imu_data = imu[0]
        quat_wxyz = (
            imu_data.attitude.w,
            imu_data.attitude.x,
            imu_data.attitude.y,
            imu_data.attitude.z,
        )
        vel_xyz = imu_data.rate_dps.x, imu_data.rate_dps.y, imu_data.rate_dps.z
        acc_xyz = (
            imu_data.accel_mps2.x,
            imu_data.accel_mps2.y,
            imu_data.accel_mps2.z,
        )
        euler_xyz = (
            imu_data.euler_rad.roll,
            imu_data.euler_rad.pitch,
            imu_data.euler_rad.yaw,
        )
        return quat_wxyz, vel_xyz, acc_xyz, euler_xyz

    @staticmethod
    def wrap_pi2pi(x):
        # wrapping x to [-pi, pi]
        angle = x % (2 * np.pi)
        if angle > np.pi:
            angle -= 2 * np.pi
        return angle


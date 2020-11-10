"""

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Atsushi Sakai (@Atsushi_twi)

"""
import matplotlib.pyplot as plt
import cvxpy
import math
import numpy as np


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None

class ModelPredictiveController(object):
    TAU = 0.1 # gain for simulatin SVEA's ESC

    NX = 4  # x = x, y, v, yaw
    NU = 2  # a = [accel, steer]
    T = 10  # horizon length

    # mpc parameters
    R = np.diag([0.1, 0.1])  # input cost matrix
    Rd = np.diag([0.01, 100.0])  # input difference cost matrix
    Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
    Qf = Q  # state final matrix
    GOAL_DIS = 0.5  # goal distance
    STOP_SPEED = 0.5 / 3.6  # stop speed
    MAX_TIME = 500.0  # max simulation time

    # iterative paramter
    MAX_ITER = 10  # Max iteration
    DU_TH = 0.02  # iteration finish param

    TARGET_SPEED = 0.6  # [m/s] target speed
    N_IND_SEARCH = 10  # Search index number

    # DT = 0.02  # [s] time tick # TODO: Hardcoded,

    # Vehicle parameters
    WB = 0.32  # [m]

    MAX_STEER = np.deg2rad(40.0)  # maximum steering angle [rad]
    MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
    MAX_SPEED = 1.5  # maximum speed [m/s]
    MIN_SPEED = -1.5  # minimum speed [m/s]
    MAX_ACCEL = 1.0  # maximum accel [m/ss]

    def __init__(self, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.traj_yaw = []
        self.sp = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.last_index = 0
        self.is_finished = False
        self.target_ind =None

    # def compute_control(self, state, target=None):
    def compute_control(self,state):
        """
        Simulation

        cx: course x position list
        cy: course y position list
        cy: course yaw position list
        sp: speed profile
        dl: course tick [m]

        """
        cx = self.traj_x
        cy = self.traj_y
        cyaw = self.traj_yaw
        goal = [cx[-1], cy[-1]]

        # # initial yaw compensation
        # if state.yaw - cyaw[0] >= math.pi:
        #     state.yaw -= math.pi * 2.0
        # elif state.yaw - cyaw[0] <= -math.pi:
        #     state.yaw += math.pi * 2.0
        if self.target_ind == None:
            self.target_ind, _ = self.calc_nearest_index(state, 0)

        self.odelta, self.oa = None, None

        cyaw = self.smooth_yaw(cyaw)

        xref, self.target_ind, dref = self.calc_ref_trajectory(
            state, self.target_ind)

        # self.target = (cx[self.target_ind],cy[self.target_ind])
        # self.target = (xref[0,:],xref[1,:])
        self.target = (xref[0,0],xref[1,0]) # FOR RVIZ

        x0 = [state.x, state.y, state.yaw, state.v]  # current state

        self.oa, self.odelta, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(
            xref, x0, dref, self.oa, self.odelta)

        if self.odelta is not None:
            di, ai = self.odelta[0], self.oa[0]

        # state = update_state(state, ai, di)

        if self.check_goal(state, goal, self.target_ind, len(cx)):
            print("Goal")
            self.is_finished = True
        steering = di
        # velocity = ov[1]
        vd = self.TAU*self.oa[0] + state.v
        velocity = vd
        return steering, velocity

    def pi_2_pi(self,angle):
        while(angle > math.pi):
            angle = angle - 2.0 * math.pi

        while(angle < -math.pi):
            angle = angle + 2.0 * math.pi

        return angle


    def get_linear_model_matrix(self, phi, v , delta):

        A = np.zeros((self.NX, self.NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 3] = self.DT * math.cos(phi)
        A[0, 2] = - self.DT * v * math.sin(phi)
        A[1, 3] = self.DT * math.sin(phi)
        A[1, 2] = self.DT * v * math.cos(phi)
        A[2, 3] = self.DT * math.tan(delta) / self.WB

        B = np.zeros((self.NX, self.NU))
        B[2, 1] = self.DT * v / (self.WB * math.cos(delta) ** 2)
        B[3, 0] = self.DT

        C = np.zeros(self.NX)
        C[0] = self.DT * v * math.sin(phi) * phi
        C[1] = - self.DT * v * math.cos(phi) * phi
        C[2] = - self.DT * v * delta / (self.WB * math.cos(delta) ** 2)

        return A, B, C


    def update_state(self,state, a, delta, ov):

        # input check
        if delta >= self.MAX_STEER:
            delta = self.MAX_STEER
        elif delta <= -self.MAX_STEER:
            delta = -self.MAX_STEER

        state.x += state.v * math.cos(state.yaw) * self.DT
        state.y += state.v * math.sin(state.yaw) * self.DT
        state.yaw += state.v / self.WB * math.tan(delta) * self.DT
        state.v += a * self.DT

        if state. v > self.MAX_SPEED:
            state.v = self.MAX_SPEED
        elif state. v < self.MIN_SPEED:
            state.v = self.MIN_SPEED

        return state


    def get_nparray_from_matrix(self,x):
        return np.array(x).flatten()


    def calc_nearest_index(self,state, pind):
        cx = self.traj_x
        cy = self.traj_y
        cyaw = self.traj_yaw

        dx = [state.x - icx for icx in cx[pind:(pind + self.N_IND_SEARCH)]]
        dy = [state.y - icy for icy in cy[pind:(pind + self.N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        mind = math.sqrt(mind)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = self.pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind


    def predict_motion(self, x0, oa, od, xref, ov):
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[2], v=x0[3])
        for (ai, di, vi, i) in zip(oa, od, ov, range(1, self.T + 1)):
            state = self.update_state(state, ai, di, vi)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.yaw
            xbar[3, i] = state.v
        # self.target = (xbar[0,:],xbar[1,:])

        return xbar


    def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):
        """
        MPC contorl with updating operational point iteraitvely
        """

        if oa is None or od is None or ov is None:
            oa = [0.0] * self.T
            od = [0.0] * self.T
            ov = [0.0] * self.T

        for i in range(self.MAX_ITER):
            xbar = self.predict_motion(x0, oa, od, xref, ov)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= self.DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov


    def linear_mpc_control(self, xref, xbar, x0, dref):
        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angle
        """

        x = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))

        cost = 0.0
        constraints = []

        for t in range(self.T):
            cost += cvxpy.quad_form(u[:, t], self.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.Q)

            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                                self.MAX_DSTEER * self.DT]

        cost += cvxpy.quad_form(xref[:, self.T] - x[:, self.T], self.Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[3, :] <= self.MAX_SPEED]
        constraints += [x[3, :] >= self.MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= self.MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= self.MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = self.get_nparray_from_matrix(x.value[0, :])
            oy = self.get_nparray_from_matrix(x.value[1, :])
            oyaw = self.get_nparray_from_matrix(x.value[2, :])
            ov = self.get_nparray_from_matrix(x.value[3, :])
            oa = self.get_nparray_from_matrix(u.value[0, :])
            odelta = self.get_nparray_from_matrix(u.value[1, :])

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov


    def calc_ref_trajectory(self, state, pind):
        cx = self.traj_x
        cy = self.traj_y
        cyaw = self.traj_yaw
        sp = self.sp

        xref = np.zeros((self.NX, self.T + 1))
        dref = np.zeros((1, self.T + 1))
        ncourse = len(cx)

        ind, _ = self.calc_nearest_index(state, pind)

        if pind >= ind:
            ind = pind

        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = cyaw[ind]
        xref[3, 0] = sp[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(self.T + 1):
            travel += abs(state.v) * self.DT
            dind = int(round(travel / self.dl))

            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = cyaw[ind + dind]
                xref[3, i] = sp[ind + dind]
                dref[0, i] = 0.0
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = cyaw[ncourse - 1]
                xref[3, i] = sp[ncourse - 1]
                dref[0, i] = 0.0

        return xref, ind, dref


    def check_goal(self, state, goal, tind, nind):

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        d = math.hypot(dx, dy)

        isgoal = (d <= self.GOAL_DIS)

        if abs(tind - nind) >= 5:
            isgoal = False

        isstop = (abs(state.v) <= self.STOP_SPEED)

        if isgoal and isstop:
            return True

        return False


    def smooth_yaw(self, yaw):

        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        return yaw


def main():
    pass

if __name__ == '__main__':
    main()

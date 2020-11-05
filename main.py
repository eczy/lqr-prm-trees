import argparse
import pdb
import numpy as np
import scipy.linalg


class ParticleNumberLine:
    def __init__(self, x_lim=5, v_lim=1, goal_eps=(0.1, 0.1), seed=None, field=False):
        self.seed = seed
        self.x_lim = x_lim
        self.v_lim = v_lim
        self.goal_eps = goal_eps

        self.A = np.array([[1, 1], [0, 1]])
        self.B = np.array([[1 / 2], [1]])

        self.state = np.zeros(2)
        self.restart()

    def restart(self):
        state = self.sample_state()
        state[-1, 0] = 0
        self.state = state

    def step(self, s, u):
        return self._A @ s + self._B.dot(u)

    def goal_reached(self, s):
        dx, dv = self.goal_eps
        return np.linalg.norm(s) <= np.linalg.norm(np.array([dx, dv]))

    def sample_state(self):
        return np.random.rand(2, 1) * np.array([self.x_lim, self.v_lim]).reshape(-1, 1)


def lqr(A, B, Q, R):
    X = scipy.linalg.solve_discrete_are(A, B, Q, R)
    K = scipy.linalg.inv(R) * (B.T * X)
    evals, _ = scipy.linalg.eig(A - B * K)
    return K, X, evals


def prm(env):
    V = [env.state]
    E = []
    A = env.A
    B = env.B
    while True:
        s_new = env.sample_state()
        # no obstacles, so s trivially in configuration space
        for s_j in V:
            A = np.column_stack((env.A, s_j))
            A = np.vstack((A, np.zeros(A.shape[-1])))
            A[-1, -1] = 1

            B = env.B
            B = np.vstack((B, np.zeros(B.shape[-1])))
            import pdb
            pdb.set_trace()
            K, X, evals = lqr(A, B, np.diag([1, 1, 1]), np.diag([1]))
            u = -K.dot(env.state)


def main():
    env = ParticleNumberLine()
    prm(env)

    # np.random.seed(0)
    # # States
    # x = np.zeros(2)
    # x_min = -5
    # x_max = -x_min
    # v_min = -1
    # v_max = -v_min

    # # Actions
    # a = 0

    # # System Dynamics
    # A = np.array([[1, 1], [0, 1]])
    # B = np.array([[1 / 2], [1]])

    # def step(s, u):
    #     return A @ s + B.dot(u)

    # # Goal State
    # s_f = np.array([[0], [0]])
    # print(s_f)
    # # Goal Space
    # allowable_dx = 0.5
    # allowable_dv = 0.1
    # # def goal_reached(s_curr):
    # # 	if (np.abs(s_curr[0,0] - s_f[0,0]) < allowable_dx and np.abs()):

    # def goal_reached(s, s_f, dx=0.5, dy=0.1):
    #     return np.linalg.norm(s - s_f) <= np.linalg.norm(np.array([dx, dy]))

    # # Initial State
    # s_i = np.zeros((2, 1))

    # s_i[0, 0] = np.random.uniform(x_min, x_max)
    # s_i[1, 0] = np.random.uniform(v_min, v_max)
    # print(s_i)

    # # Random Sample
    # def random_sample(p=0.05):
    #     theta = np.random.sample()
    #     if theta < p:
    #         return s_f
    #     s_rand = np.zeros((2, 1))
    #     s_rand[0, 0] = np.random.uniform(x_min, x_max)
    #     s_rand[1, 0] = np.random.uniform(v_min, v_max)
    #     return s_rand

    # V = [s_i]
    # E = []

    # def drive_to(v, u):
    #     """Drive from v to u."""
    #     delta_p = v[0] - u[0]
    #     if delta_p > 0:
    #         a = -1
    #     elif delta_p < 0:
    #         a = 1
    #     else:
    #         a = 0
    #     return step(v, a)

    # print("Starting simulation.")
    # while True:
    #     s_rand = random_sample()
    #     nearest_index, s_near, distance = nearest_vertex(s_rand, V)
    #     s_new = drive_to(s_near, s_rand)
    #     V.append(s_new)
    #     E.append((nearest_index, (len(V) - 1)))
    #     if goal_reached(s_new, s_f):
    #         return V, E


if __name__ == "__main__":
    main()

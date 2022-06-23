import sys
import rospy
import numpy as np
import moveit_commander
from forward_kinematics import GenerateFK

moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("arm")


class AnalyticIK:
    def __init__(self, target, genFK=False):
        self.target = target
        if genFK:
            group.go(np.zeros(4), wait=True)  # required to get correct initial trans values
            self.fkGenerator = GenerateFK()
            phi, x, y, z = self.fkGenerator.forward_kinematics()
        # self.coeffs = [-0.2, -0.24, 0.4385, 1.3248]  # wrt base_link
        self.coeffs = [0.2, 0.25, 0.1415, -0.175]  # wrt arm_base
        # x = -0.2 * cos(q1) - 0.2 * cos(q1 - q2) - 0.25 * cos(q1 - q2 + q3) + 0.4385
        # y = -0.2 * sin(q1) - 0.2 * sin(q1 - q2) - 0.25 * sin(q1 - q2 + q3)
        # self.coeffs are the coefficients of cos(q1), cos(q1-q2+q3) and constant term as
        # seen from the forward_kinematics expression. Just look at the generated x in the above line. The first two
        # terms of the coefficients are same in the expression for y

    def get_q3(self, q2, coeffs=None):
        xt, yt, zt = self.target
        if coeffs:
            a1, a2, a3 = coeffs[0], coeffs[1], coeffs[2]
        else:
            a1, a2, a3 = self.coeffs[0], self.coeffs[1], self.coeffs[2]
        theta = np.arctan2((1 + np.cos(q2)), np.sin(q2))
        c = np.sqrt(2 * (1 + np.cos(q2))) * 2 * a1 * a2
        a = (xt - a3) ** 2 + yt ** 2 - 2 * a1*a1 * np.cos(q2) - 2 * a1 ** 2 - a2 ** 2
        bb = np.arcsin(a / c)
        q3 = [bb - theta, np.pi - bb -theta, bb - theta + 2*np.pi,  3*np.pi - bb -theta]
        validQ3 = [num for num in q3 if self.isSolutionValid(num)]
        return validQ3

    def get_q1(self, q2, q3, coeffs=None):
        xt, yt, zt = self.target
        if coeffs:
            a1, a2, a3 = coeffs[0], coeffs[1], coeffs[2]
        else:
            a1, a2, a3 = self.coeffs[0], self.coeffs[1], self.coeffs[2]
        beta = np.arctan2((xt - a3), yt)
        K = (xt - a3) ** 2 + yt ** 2 + a2 ** 2 - 2*a1*a1 * (1+np.cos(q2))
        den = 2 * a2 * np.sqrt((xt - a3) ** 2 + yt ** 2)
        bb = np.arcsin(K / den)
        tt = q2 - q3 - beta
        q1 = [bb + tt, np.pi - bb + tt, bb + tt + 2 * np.pi, 3 * np.pi - bb + tt]
        validQ1 = [num for num in q1 if self.isSolutionValid(num)]
        return validQ1

    def get_q1_2(self, q2, q3, coeffs=None):
        xt, yt, zt = self.target
        if coeffs:
            a1, a2, a3 = coeffs[0], coeffs[1], coeffs[2]
        else:
            a1, a2, a3 = self.coeffs[0], self.coeffs[1], self.coeffs[2]
        beta = np.arctan2((xt - a3), yt)
        K = ((xt - a3) ** 2 + yt ** 2 - a2 ** 2 - 2 * a1 * a2 * np.cos(q2 - q3))
        den = 2 * a1 * np.sqrt((xt - a3) ** 2 + yt ** 2)
        bb = np.arcsin(K / den)
        q1 = [bb - beta + q2, np.pi - bb - beta + q2, bb - beta + 2 * np.pi + q2, 3 * np.pi - bb - beta + q2]
        validQ1 = [num for num in q1 if self.isSolutionValid(num)]
        return validQ1

    def get_q1_1(self, q3, coeffs=None):
        xt, yt, zt = self.target
        if coeffs:
            a1, a2, a3 = coeffs[0], coeffs[1], coeffs[2]
        else:
            a1, a2, a3 = self.coeffs[0], self.coeffs[1], self.coeffs[2]
        beta = np.arctan2((xt - a3), yt)
        K = ((xt - a3) ** 2 + yt ** 2 - a2 ** 2 - 2 * a1 * a2 * np.cos(q3))
        den = (2 * a1) * np.sqrt((xt - a3) ** 2 + yt ** 2)
        kk = K / (2 * a1)
        dd = den / (2 * a1)
        ss = [K/den, kk/dd, np.arcsin(kk / dd)]
        bb = np.arcsin(K / den)
        print(np.arcsin(kk / dd) - np.arcsin(K / den), "###")
        q1 = [bb - beta, np.pi - bb - beta, bb - beta + 2 * np.pi, 3 * np.pi - bb - beta]
        validQ1 = [num for num in q1 if self.isSolutionValid(num)]
        return validQ1

    def isSolutionValid(self, q):
        # 2.01 bcoz, I dont want to go near the joint_limits
        if -np.pi / 2.01 <= q <= np.pi / 2.01:
            return True
        else:
            return False

    def analytic_solution(self, coefficients=None):
        if coefficients:
            coeffs = coefficients
        else:
            coeffs = self.coeffs
        q2 = np.linspace(-np.pi / 2.01, np.pi / 2.01, 100)
        solution, q1 = list(), list()
        for i in q2:
            q3 = self.get_q3(q2=i, coeffs=coeffs)
            for q in q3:
                q1 = self.get_q1(q2=i, q3=q, coeffs=coeffs)
                for qq in q1:
                    solution.append([qq, i, q])
        d = self.coeffs[3] - self.target[2]
        solution = np.insert(np.array(solution), 0, d, axis=1)
        return solution


if __name__ == "__main__":
    # run the simulated robot in tmux
    rospy.init_node('analyticIK_node', anonymous=True)
    # target = [0.1, 0.25, 1.1]  # in base_link
    target = [0.45, -0.25, -0.40]  # in arm_base
    sik = AnalyticIK(target=target, genFK=True)
    sols = sik.analytic_solution()

    

import tf
import rospy
import numpy as np
import sympy as sp


class GenerateFK:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.links = ['arm_base', 'arm_link_0', 'arm_link_1', 'arm_link_2',
                      'arm_link_3', 'arm_end_effector']

        self.trans = []
        for i in range(len(self.links) - 1):
            self.trans.append(self.get_trans_rot(self.links[i], self.links[i + 1])[0])

        self.cos = sp.cos
        self.sin = sp.sin
        self.d, self.q1, self.q2, self.q3 = sp.symbols('d, q1 q2 q3')
        self.trans_rbsab = self.trans[0]  # translation from robot base (base_link) to arm_base
        self.trans_sabl0 = [l1 - l2 for (l1, l2) in
                            zip(self.trans[1], [0.0, 0.0, self.d])]  # translation from arm_base to arm_link_0
        self.trans_l0l1 = self.trans[2]  # translation from arm_link_0 to arm_link_1
        self.trans_l1l2 = self.trans[3]  # translation from arm_link_1 to arm_link_2
        self.trans_l2l3 = self.trans[4]  # translation from arm_link_2 to arm_link_3
        self.trans_l3Eff = self.trans[5]  # translation from arm_link_3 to arm_gripper_tip
        self.tfMat_rbsab = self.trans_z(np.pi, self.trans_rbsab)  # tranformation matrix from base_link to arm_base
        self.tfMat_sabl0 = self.trans_z(0, self.trans_sabl0)  # tranformation matrix from arm_base to link0
        self.tfMat_l0l1 = self.trans_z(self.q1, self.trans_l0l1)
        self.tfMat_l1l2 = self.trans_z(self.q2, self.trans_l1l2)
        self.tfMat_l2l3 = self.trans_z(self.q3, self.trans_l2l3)
        self.tfMat_l3Eff = self.trans_z(0, self.trans_l3Eff)
        self.tfMat_rbEff = self.tfMat_sabl0 * self.tfMat_l0l1 * self.tfMat_l1l2 * self.tfMat_l2l3 * self.tfMat_l3Eff

    def get_trans_rot(self, source="arm_base", target="arm_end_effector"):
        self.listener.waitForTransform(source, target, rospy.Time(0), rospy.Duration(2.0))
        trans, qrt = self.listener.lookupTransform(source, target, rospy.Time(0))
        transMat = self.listener.fromTranslationRotation(trans, qrt)  # 4x4 transformation matrix
        return trans, qrt, transMat

    def forward_kinematics(self):
        x = self.tfMat_rbEff[0, 3].rewrite(sp.exp).simplify().expand().rewrite(sp.cos).simplify()
        y = self.tfMat_rbEff[1, 3].rewrite(sp.exp).simplify().expand().rewrite(sp.cos).simplify()
        z = self.tfMat_rbEff[2, 3].rewrite(sp.exp).simplify().expand().rewrite(sp.cos).simplify()

        x_rbEff = sp.nsimplify(x, tolerance=1e-10, rational=True)
        y_rbEff = sp.nsimplify(y, tolerance=1e-10, rational=True)
        z_rbEff = sp.nsimplify(z, tolerance=1e-10, rational=True)
        a = self.tfMat_rbEff[0, 0].rewrite(sp.exp).simplify().expand().rewrite(sp.cos).simplify()
        a = sp.nsimplify(a, tolerance=1e-10,
                         rational=True)  # gives -cos(q1 - q2 + q3) implies cos(180 + (q1 - q2 + q3))
        phi = sp.pi + self.q1 - self.q2 + self.q3  # orientation of the arm_gripper_tip
        return phi, x_rbEff, y_rbEff, z_rbEff

    def trans_z(self, q, t):
        return sp.Matrix([[self.cos(q), -self.sin(q), 0, t[0]],
                          [self.sin(q), self.cos(q), 0, t[1]],
                          [0, 0, 1, t[2]],
                          [0, 0, 0, 1]])
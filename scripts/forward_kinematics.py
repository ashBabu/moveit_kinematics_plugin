import tf
import rospy
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

        self.tfMat_sabl0 = self.trans_z(0, self.trans[0])         # symbolic tranformation matrix from arm_base to arm_link_0
        self.tfMat_l0l1 = self.trans_z(self.q1, self.trans[1])    # symbolic tranformation matrix from arm_link_0 to arm_link_1
        self.tfMat_l1l2 = self.trans_z(self.q2, self.trans[2])
        self.tfMat_l2l3 = self.trans_z(self.q3, self.trans[3])
        self.tfMat_l3Eff = self.trans_z(0, self.trans[4])

        self.tfMat_abEff = self.tfMat_abl0 * self.tfMat_l0l1 * self.tfMat_l1l2 * self.tfMat_l2l3 * self.tfMat_l3Eff  # Symbolic Forward Kinematics 

    def get_trans_rot(self, source="arm_base", target="arm_end_effector"):
        self.listener.waitForTransform(source, target, rospy.Time(0), rospy.Duration(2.0))
        trans, qrt = self.listener.lookupTransform(source, target, rospy.Time(0))
        transMat = self.listener.fromTranslationRotation(trans, qrt)  # 4x4 transformation matrix
        return trans, qrt, transMat

    def forward_kinematics(self):
        x = self.tfMat_abEff[0, 3].rewrite(sp.exp).simplify().expand().rewrite(sp.cos).simplify()
        y = self.tfMat_abEff[1, 3].rewrite(sp.exp).simplify().expand().rewrite(sp.cos).simplify()
        z = self.tfMat_abEff[2, 3].rewrite(sp.exp).simplify().expand().rewrite(sp.cos).simplify()

        x_abEff = sp.nsimplify(x, tolerance=1e-10, rational=True)
        y_abEff = sp.nsimplify(y, tolerance=1e-10, rational=True)
        z_abEff = sp.nsimplify(z, tolerance=1e-10, rational=True)
        a = self.tfMat_abEff[0, 0].rewrite(sp.exp).simplify().expand().rewrite(sp.cos).simplify()
        a = sp.nsimplify(a, tolerance=1e-10, rational=True)  # gives cos(q1 + q2 + q3) 
        phi = self.q1 + self.q2 + self.q3  # orientation of the arm_end_effector wrt arm_base
        return phi, x_abEff, y_abEff, z_abEff

    def trans_z(self, q, t):
        return sp.Matrix([[self.cos(q), -self.sin(q), 0, t[0]],
                          [self.sin(q), self.cos(q), 0, t[1]],
                          [0, 0, 1, t[2]],
                          [0, 0, 0, 1]])
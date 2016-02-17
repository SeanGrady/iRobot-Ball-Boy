import unittest
from roomba_node import DriveNode

class RoombaNodeTests(unittest.TestCase):

    def setUp(self):
        self.dn = DriveNode()

    def test_do_nothing(self):
        self.dn.left_total = 10
        self.dn.right_total = 20
        self.dn.get_encoder_counts = lambda: (10, 20)
        self.assertEqual(self.dn.handle_requestAngle(None), 0)

    def test_right(self):
        self.dn.left_total = 10.
        self.dn.right_total = 20.
        self.dn.get_encoder_counts = lambda: (20., 10.)
        self.assertEqual(int(self.dn.handle_requestAngle(None)), 2)

    def test_left(self):
        self.dn.left_total = 10.
        self.dn.right_total = 20.
        self.dn.get_encoder_counts = lambda: (0., 30.)
        self.assertEqual(int(self.dn.handle_requestAngle(None)), -2)

    def test_full_turn_right(self):
        self.dn.left_total = 32035.
        self.dn.right_total = 27964.
        self.dn.get_encoder_counts = lambda: (30000., 30000.)
        self.assertEqual(int(self.dn.handle_requestAngle(None)), -443)

    def test_full_turn_left(self):
        self.dn.left_total = 27964.
        self.dn.right_total = 32035.
        self.dn.get_encoder_counts = lambda: (30000., 30000.)
        self.assertEqual(int(self.dn.handle_requestAngle(None)), 443)

    def test_right_rollover(self):
        self.dn.left_total = 65530.
        self.dn.right_total = 20.
        self.dn.get_encoder_counts = lambda: (5., 10.)
        self.assertEqual(int(self.dn.handle_requestAngle(None)), 2)

if __name__ == "__main__":
    unittest.main()

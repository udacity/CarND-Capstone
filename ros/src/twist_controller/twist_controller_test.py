import unittest
from twist_controller import Controller

class TestControllerMethods(unittest.TestCase):
    
    def test_make_controller(self):
        c = Controller()
        self.assertIsNotNone(c, 'Failed to create controller.')
        
    def test_controller_control_returns3values(self):
        c = Controller()
        throttle, brake, steer = c.control()
        self.assertIsNotNone(throttle, 'Throttle not returned')
        self.assertIsNotNone(brake, 'Brake not returned')
        self.assertIsNotNone(steer, 'Steering not returned')
        
    def test_controller_default_values(self):
        c = Controller()
        self.assertEqual(c.gainp, 1, 'GainP not initialized')
        self.assertEqual(c.gaini, 0, 'Gaini not initialized')
        self.assertEqual(c.gaind, 0, 'Gaind not initialized')
        
if __name__=='__main__':
    unittest.main()
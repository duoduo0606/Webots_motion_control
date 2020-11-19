import numpy as np


class TargetGenerator(object):
    def __init__(self):
        self.y_forward = 20.00
        self.y_backward = 20.00
        self.control_period = 0.01
        self.l1 = 70.0
        self.l2 = 50.0
        self.l3 = 24.0
        self.target_pos = {'rf': [self.l1, self.l2 + self.y_forward, self.l3],
                           'rh': [self.l1, self.l2 + self.y_backward, self.l3],
                           'lf': [self.l1, self.l2 + self.y_backward, self.l3],
                           'lh': [self.l1, self.l2 + self.y_forward, self.l3]}
        self.origin_target_pos = {'rf': [self.l1, self.l2 + self.y_forward, self.l3],  # To save the temp data
                           'rh': [self.l1, self.l2 + self.y_backward, self.l3],
                           'lf': [self.l1, self.l2 + self.y_backward, self.l3],
                           'lh': [self.l1, self.l2 + self.y_forward, self.l3]}
        self.stance_start_point = {'rf': [self.l1, self.l2 + self.y_backward, self.l3],  # To save the stance point start
                           'rh': [self.l1, self.l2 + self.y_forward, self.l3],
                           'lf': [self.l1, self.l2 + self.y_forward, self.l3],
                           'lh': [self.l1, self.l2 + self.y_backward, self.l3]}
        self.status = {'rf': 0, 'rh': 0, 'lf': 0, 'lh': 0}  # 0: steady 1: stance 2: detachment 3: attachment
        self.step_height = 50.00
        self.attachment_time = 0.75
        self.detachment_time = 1.00
        self.pattern = 'wave'
        self.status_graph = {'wave': np.array([[2, 1, 1, 1], [3, 0, 0, 0],
                                               [1, 2, 1, 1], [0, 3, 0, 0],
                                               [1, 1, 2, 1], [0, 0, 3, 0],
                                               [1, 1, 1, 2], [0, 0, 0, 3]]),
                             'trot': np.array([[2, 1, 1, 2], [3, 0, 0, 3],
                                               [1, 2, 2, 1], [0, 3, 3, 0]])}
        self.moving_time = self.detachment_time * 3
        self.stance_velocity = {'rf': 0, 'rh': 0, 'lf': 0, 'lh': 0}
    def set_pattern(self, pattern):
        if pattern == 'wave':
            self.pattern = pattern
            self.moving_time = self.detachment_time * 3
        elif pattern == 'trot':
            self.pattern = pattern
            self.moving_time = self.detachment_time

    def trajectory_generate(self, leg_tag, status, status_time):  # 0: steady 1: stance 2: detachment 3: attachment
        if status == 0:
            pass
        elif status == 1:
            pass
        elif status == 2:
            pass
        elif status == 3:
            pass








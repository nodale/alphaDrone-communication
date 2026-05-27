from dataclasses import dataclass
from multiprocessing import shared_memory

import numpy as np
import torch
import socket


@dataclass
class QuickHongi:
    def __init__(self):

    def _reorder_np(self, K):
        idx = [0, 2, 4, 1, 3, 5, 6, 7, 8, 9, 10, 11]
        return np.asarray(K)[:, idx]

    def _reorder_torch(self, K):
        idx = torch.tensor([0, 2, 4, 1, 3, 5, 6, 7, 8, 9, 10, 11], device=K.device)
        return K.index_select(dim=1, index=idx)

    def _reorder_tuple(self, K):
        idx = [0, 2, 4, 1, 3, 5, 6, 7, 8, 9, 10, 11]
        return [[row[i] for i in idx] for row in K]

    def _np_enu_to_ned(self, K_enu):
        col = np.array([1, 0, 2, 4, 3, 5, 6, 7, 8, 9, 10, 11])
        sign = np.array([1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1])
        return np.asarray(K_enu)[:, col] * sign

    def _torch_enu_to_ned(self, K_enu):
        col = torch.tensor([1, 0, 2, 4, 3, 5, 6, 7, 8, 9, 10, 11], device=K_enu.device)
        sign = torch.tensor([1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1],
                            dtype=K_enu.dtype, device=K_enu.device)
        return K_enu.index_select(dim=-1, index=col) * sign

    def _remap_actuation(self):

    def get_action(self, states, setpoints):
        normalized_feedback_law=[[  0.11080865,   0.19855924,   0.0916755 ,   0.16409858,  -2.39842276,
                                   -8.28777724,  -1.37845782,   1.66624166,   2.82357001,  -0.84908283,
                                    1.02735811,  11.24748391],
                                 [ -0.11080865,  -0.19855924 ,  0.0916755 ,   0.16409858,  -2.39842276,
                                   -8.28777724,  -1.37845782,  -1.66624166,  -2.82357001,  -0.84908283,
                                   -1.02735811, -11.24748391]  ,
                                 [ -0.11080865,  -0.19855924,  -0.0916755 ,  -0.16409858,  -2.39842276,
                                   -8.28777724,   1.37845782,  -1.66624166,   2.82357001,   0.84908283,
                                   -1.02735811,  11.24748391],
                                 [  0.11080865,   0.19855924,  -0.0916755 ,  -0.16409858,  -2.39842276,
                                   -8.28777724,   1.37845782,   1.66624166,  -2.82357001,   0.84908283,
                                    1.02735811, -11.24748391]]
        hover = 9.28795
        A_MAX = 5.76155

        _K_torch = self._reorder_torch(normalized_feedback_law)
        _K_torch = self._torch_enu_to_ned(_K_torch)
        
        tracking_error = setpoints - states

        #action = normalized_feedback_law @ np.asarray(tracking_error)
        action = tracking_error @ _K_torch
        action = np.clip(action, -1, 1)
        applied_action =  hover + action * A_MAX
        return applied_action

        #ORDER = ['X', 'XDOT', 'Y', 'YDOT', 'Z', 'ZDOT', 'PHI', 'THETA', 'PSI', 'P', 'Q', 'R']

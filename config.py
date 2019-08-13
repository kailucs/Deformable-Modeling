import math

class DMConfig(object):

    def __init__(self, MODE):
        # ----------- global setting
        self.mode = MODE
        self.step = 'y'
        self.robot_model_path = "robot_model_data/ur5Blocks.xml"
        self.ee_link_number = 7
        self.calibration_xform_path = 'calibrated_transforms/calibrated_camera_xform.txt'
        self.merge_cmd = './utils/merge_cpp_ws/Merge_pcd'
        self.exp_path = './experiment_data/'
        self.probe_type = 'point'
        self.use_mkdir_exp = False

        # ----------- experiment params
        self.exp_number = 0 #this will reset in the main function.       
        
        self.tableHeight = 0.865
        self.probeLength = 0.09 
        self.probe_line_theta_bias = 0.0 # to the local z axis. old usage.
        
        self.probe_transform = [[1,0,0,self.probeLength],
                                [0,0.707,-0.707,0],
                                [0,0.707,0.707,0],
                                [0,0,0,1]]
        '''
        self.probe_transform = [[1,0,0,self.probeLength],
                                [0,1,0,0],
                                [0,0,1,0],
                                [0,0,0,1]]
        '''
        self.forceLimit = 3 # F=2N, depends on object hardness
        self.dt=0.004
        self.moveStep=0.002*self.dt   #2mm /s

        self.robot_host = '10.10.1.106'

        self.num_pcd = 5
        
        self.drawFlag = True

        # note: now this is only used for get PCDs
        self.home_config2 = [-1.082104,-0.8105309,1.6866862,
                            0.692372114,1.567305668,2.8312731126, 0]
        
        self.intermediateConfig_old = [-1.0812161604510706, -0.5610864919475098,1.6372855345355433, 
                                    0.49511925756420894, 1.5732531547546387, 2.8483853340148926, 0]
        
        self.intermediateConfig = [-1.08305692024,-0.698302360067,1.67848698749,
                            0.58852538422,1.56781803046,2.83162462606 - 1.57, 0]

        self.longServoTime = 3.5
        self.shortServoTime = 1.5
        self.IKErrorTolerence = 1.57 
        self.maxDev = 3.14
        self.EEZLimit = 0.9 #TODO:0.956
        
        self.use_curvature = False

    def list_all_member(self):
        params = {}
        for name,value in vars(self).items():
            params[name] = value
        return params
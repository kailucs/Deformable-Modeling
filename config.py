class DMConfig(object):

    def __init__(self, MODE):
        # ----------- global setting
        self.mode = MODE
        self.step = 'n'
        self.robot_model_path = "robot_model_data/ur5Blocks.xml"
        self.ee_link_number = 7
        self.calibration_xform_path = 'calibrated_transforms/calibrated_camera_xform.txt'
        self.merge_cmd = './utils/merge_cpp_ws/Merge_pcd'
        self.exp_path = './experiment_data/'
        self.probe_type = 'point'

        # ----------- experiment params
        self.exp_number = 0 #this will reset in the main function.       
        
        self.tableHeight = 0.86
        self.probeLength = 0.09 #TODO: may need to change name to probeLength_point
        self.probeLength_Line = 0.09 #TODO: old usage.
        self.probe_line_theta_bias = 0.0 # to the local z axis. old usage.

        self.probe_transform = [[1,0,0,self.probeLength],
                                [0,1,0,0],
                                [0,0,1,0],
                                [0,0,0,1]]
                                
        self.forceLimit = 2 # F=2N, depends on object hardness
        self.dt=0.004
        self.moveStep=0.002*self.dt   #2mm /s

        self.robot_host = '10.10.1.106'

        self.num_pcd = 5
        
        self.drawFlag = False

        self.home_config2 = [-1.08105692024,-0.866032360067,1.67516698749,
                            0.762010738422,1.57201803046,2.84750462606 , 0]
        
        self.intermediateConfig = [-1.0812161604510706, -0.5610864919475098,1.6372855345355433, 
                                    0.49511925756420894, 1.5732531547546387, 2.8483853340148926, 0]
        
        self.longServoTime = 3
        self.shortServoTime = 1.5
        self.IKErrorTolerence = 4  
        self.maxDev = 1.2
        self.EEZLimit = 0.956
        
    def list_all_member(self):
        params = {}
        for name,value in vars(self).items():
            params[name] = value
        return params

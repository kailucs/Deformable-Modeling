class DMConfig(object):

    def __init__(self, FLAGS):

        self.batch_size = 16
        # self.random_start = 30
        self.cnn_format = 'NCHW'
        self.discount = 0.6 # epsilon in RL (decay index)
        self.target_q_update_step = 1 * self.scale
        self.learning_rate = 0.001
        self.learning_rate_minimum = 0.00025
        self.learning_rate_decay = 0.96
        self.learning_rate_decay_step = 4 * self.scale

        self.ep_end = 0.20
        self.ep_start = 1.
        self.ep_end_t = 4. * self.memory_size # encourage some new actions

        self.history_length = 4
        self.train_frequency = 4
        self.learn_start = 4. * self.scale
        
        if FLAGS.use_gpu == False:
            self.cnn_format = 'NHWC'
        else:
            self.cnn_format = 'NCHW'

        if FLAGS.is_train == False:
            self.is_train = False

        if FLAGS.is_sim == False:
            self.is_sim = False
            
    def list_all_member(self):
        params = {}
        for name,value in vars(self).items():
            params[name] = value
        return params

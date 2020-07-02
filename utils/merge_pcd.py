import os

def merge_pcd(config):
    os.system('rm -r '+os.path.join(config.exp_path,'tmp'))
    os.system('mkdir '+os.path.join(config.exp_path,'tmp'))
    os.system('cp -rf '+os.path.join(config.exp_path,'exp_'+str(config.exp_number)+'/processed ')
                +os.path.join(config.exp_path,'tmp/processed'))
    os.system(config.merge_cmd)
    os.system('rm '+os.path.join(config.exp_path,'exp_'+str(config.exp_number)+'/TSDF_result.ply'))
    os.system('cp -f '+os.path.join(config.exp_path,'tmp/TSDF_result.ply ')
                +os.path.join(config.exp_path,'exp_'+str(config.exp_number)+'/TSDF_result.ply'))
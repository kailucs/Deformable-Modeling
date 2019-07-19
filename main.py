import os
import argparse
import calibration_testing

from config import DMConfig

from utils.data_collection_get_PCD import run_collection_PCD
from utils.data_collection_process_pcd import run_collection_process_PCD
from utils.convert_ply import run_convert_ply
from utils.data_collection_process_pcd_with_curvature import run_calculation

from experiment.data_collection_probing import run_poking

def run_all(config):
    """
    this is core function of modeling and poking.
    """
    #modeling
    run_collection_PCD(config)    
    run_collection_process_PCD(config)
    
    os.system('cp -rf '+os.path.join(config.exp_path,'exp_'+str(config.exp_number)+'/processed ')
                +os.path.join(config.exp_path,'tmp/processed'))
    os.system(config.merge_cmd)
    os.system('cp -f '+os.path.join(config.exp_path,'tmp/TSDF_result.ply ')
                +os.path.join(config.exp_path,'exp_'+str(config.exp_number)+'/TSDF_result.ply'))

    run_convert_ply(config)
    run_calculation(config)
    
    # poking
    run_poking(config)

def physical_mode(config):
    """
    physical mode: robot acting.
    """
    if not os.path.exists(config.exp_path):
        os.mkdir(config.exp_path)

    config.exp_number = len(os.listdir(config.exp_path))

    os.mkdir(os.path.join(config.exp_path,'exp_'+str(config.exp_number)))
    print('[*] Experiment '+str(config.exp_number)+' created')

    run_all(config)

def debugging_mode(config):
    """
    debugging mode: disconnect robot, just data testing.
    """
    if not os.path.exists(os.path.join(config.exp_path,'exp_'+str(config.exp_number))):
        print('[!]experiment '+str(config.exp_number)+' no found.')
    else:
        print('[*]experiment '+str(config.exp_number)+' debug start.')
    
    run_all(config)

def step_mode(config):
    """
    this is to process step by step
    """
    print('[*]Step mode start')
    print('%d experiments exist'%len(os.listdir(config.exp_path)))
    config.exp_number = input('experiment number?')
    if input('Collection PCD?') == 1: #TODO: if string, python2 need raw_input
        run_collection_PCD(config)
    if input('Process PCD?') == 1:
        run_collection_process_PCD(config)
    if input('Merge PCD?') == 1:
        os.system('cp -rf '+os.path.join(config.exp_path,'exp_'+str(config.exp_number)+'/processed ')
                    +os.path.join(config.exp_path,'tmp/processed'))
        os.system(config.merge_cmd)
        os.system('cp -f '+os.path.join(config.exp_path,'tmp/TSDF_result.ply ')
                    +os.path.join(config.exp_path,'exp_'+str(config.exp_number)+'/TSDF_result.ply'))
    if input('Convert to ply?') == 1:
        run_convert_ply(config)
        run_calculation(config)
    if input('Poking?') == 1:
        run_poking(config)

def main():
    """
    this is the main function of modeling and data collecting.
    """
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '--process',
        default='physical',
        dest='process',
        type=str
    )
    argparser.add_argument(
        '--gpus',
        nargs='+',
        default='0',
        dest='gpus',
        type=str
    )
    argparser.add_argument(
        '--exp',
        default=0,
        dest='exp',
        type=int
    )
    argparser.add_argument(
        '--step',
        default='n',
        dest='step',
        type=str
    )
    argparser.add_argument(
        '--probe-type',
        default='point',
        dest='probe_type',
        type=str
    )


    args = argparser.parse_args()

    config = DMConfig(args.process)
    config.probe_type = args.probe_type
    
    # Check if the vector of GPUs passed are valid.
    for gpu in args.gpus:
        try:
            int(gpu)
        except ValueError:  # Reraise a meaningful error.
            raise ValueError("GPU is not a valid int number")

    if args.process is not None:
        if args.process == 'calibration':
            # calibration
            print('[*]Calibration start')
            calibration_testing.calibration()

        elif args.step == 'y':
            step_mode(config)

        elif args.step == 'n':
            if args.process == 'physical':
                print('[*]Physical mode start')
                physical_mode(config)

            elif args.process == 'debugging':
                print('[*]Debugging mode start')
                config.exp_number = args.exp
                debugging_mode(config)

        else:
            raise Exception("Invalid name for single process, chose from (calibration, physical, debugging)")

    else:
        raise Exception("Must set a process")

if __name__ == "__main__":
    main()

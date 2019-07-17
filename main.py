import os
import calibration_testing

from utils.data_collection_get_PCD import run_collection_PCD
from utils.data_collection_process_pcd import run_collection_process_PCD
from utils.convert_ply import run_convert_ply
from utils.data_collection_process_pcd_with_curvature import run_calculation

from experiment.data_collection_probing import run_poking

import config
import argparse

exp_path = './experiment_data'
merge_cmd = './utils/merge_cpp_ws/Myprogram'
#TODO: need to convert to config.

def main():
    """
    this is the main function of modeling and data collecting.
    """

    argparser.add_argument(
        '--process',
        default='physical',
        dest='process',
        type=str
    )
    argparser.add_argument(
        '--gpus',
        nargs='+',
        dest='gpus',
        type=str
    )
    args = argparser.parse_args()

    # Check if the vector of GPUs passed are valid.
    for gpu in args.gpus:
        try:
            int(gpu)
        except ValueError:  # Reraise a meaningful error.
            raise ValueError("GPU is not a valid int number")

    if args.single_process is not None:
        if args.single_process == 'calibration':
            # calibration
            # calibration_testing.calibration()
        elif args.single_process == 'physical':
            # modeling
            os.mkdir(exp_path)
            run_collection_PCD()
            run_collection_process_PCD()
            os.system(merge_cmd) #TODO: bugs.
            run_convert_ply()
            run_calculation()

            # poking
            run_poking()
        elif args.single_process == 'simulation':
            # add code

        else:
            raise Exception("Invalid name for single process, chose from (calibration, physical, simulation)")

    else:
        raise Exception("Must set a process")

if __name__ == "__main__":
    main()




    


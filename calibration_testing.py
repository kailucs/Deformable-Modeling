from utils.calibration_calculation_color import calculation
from utils.generate_marker_transform_color import generation
from utils.take_calibration_pictures import picture

def calibration():
    """
    calibration function 
    """
    calculation()
    generation()
    picture()
    print('calibration successful.')
    
if __name__ == "__main__":
    calibration()
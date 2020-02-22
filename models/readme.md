Open3d a bit different at 0.9. The code in Sept 2019 used 0.8. e.g. from open3d import *. Then all the functions can just directly be used. (The documentation does this too)

This is not the case for 0.9 anymore. The official doc also starts to do e.g. o3d.geometry.PointCloud.remove_statistical_outlier... SOme functions have also become slightly different.
Only line_demo.py has adopted these changes......

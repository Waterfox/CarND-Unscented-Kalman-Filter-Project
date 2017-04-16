
# Unscented Kalman Filter Project

Build the project from build directory
```
$cmake..&& make
```
Run the kalman Filter from the build directory
```
./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt ../results/data1_out.txt

```

## Dataset 1
Accuracy - RMSE:
- x: 0.77
- y: 0.86
- vx:  0.58
- vy:  0.59
- std_a_ = 3.0
- std_yawdd_ = 0.6

The following plots show the output for dataset 1. Interestingly, the velocity is tracked as negative. The RMSE would increase slightly if the velocity and yaw were flipped after prediction, perhaps the yaw rate also had to be managed.

![alt text](\results\data1_1.png "Logo Title Text 1")
![alt text](\results\data1_2.png "Logo Title Text 1")

The following plots show the NIS for the laser and radar data. It appears the process noise for the laser was estimated conservatively, while the noise the radar was estimated appropriately.

![alt text](\results\data1_NIS_laser.png "Logo Title Text 1")
![alt text](\results\data1_NIS_radar.png "Logo Title Text 1")


## Dataset 2
Accuracy - RMSE:
- x:  0.193477
- y:  0.189355
- vx:  0.524904
- vy:   0.50964
- std_a_ = 2.5
- std_yawdd_ = 0.5


![alt text](\results\data2_1.png "Logo Title Text 1")
![alt text](\results\data2_2.png "Logo Title Text 1")

The following plots show the NIS for the laser and radar data. It appears the process noise for the laser was estimated conservatively, while the noise the radar was estimated more appropriately.

![alt text](\results\data2_NIS_laser.png "Logo Title Text 1")
![alt text](\results\data2_NIS_radar.png "Logo Title Text 1")

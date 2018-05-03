# Unscented Kalman Filter Project

Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   - On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./UnscentedKF `

## Accuracy

Based on the rubric points for accuracy, `RMSE [px, py, vx, vy]` must be less than `[.09, .10, .40, .30]`.  

> From the rubric:
>
> “Your algorithm will be run against Dataset 1 in the simulator which is the same as "[data/obj_pose-laser-radar-synthetic-input.txt]()" in the repository. We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].”

The obtained accuracy is
``` C++
>> Accuracy | rmse_px -> 0.0966757  | rmse_py -> 0.0971482  | rmse_vx -> 0.213195   | rmse_vy -> 0.25725
```
## License

[MIT License]().
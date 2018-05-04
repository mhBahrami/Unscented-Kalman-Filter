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
> “Your algorithm will be run against Dataset 1 in the simulator which is the same as "[data/obj_pose-laser-radar-synthetic-input.txt](https://github.com/mhBahrami/Unscented-Kalman-Filter/blob/master/data/obj_pose-laser-radar-synthetic-input.txt)" in the repository. We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].”

The accuracy has been calculated in 3 different situation.
1. The UKF only uses RADAR sensor measurement data:
   - Set [`use_laser_ = false`](https://github.com/mhBahrami/Unscented-Kalman-Filter/blob/master/src/ukf.cpp#L40) and [`use_radar_ = true`](https://github.com/mhBahrami/Unscented-Kalman-Filter/blob/master/src/ukf.cpp#L43).
   - The accuracy is:
        ``` C++
        >> Accuracy | rmse_px -> 0.0966757  | rmse_py -> 0.0971482  | rmse_vx -> 0.213195   | rmse_vy -> 0.25725
        ```
2. The UKF only uses LIDAR sensor measurement data:
   - Set [`use_laser_ = true`](https://github.com/mhBahrami/Unscented-Kalman-Filter/blob/master/src/ukf.cpp#L40) and [`use_radar_ = false`](https://github.com/mhBahrami/Unscented-Kalman-Filter/blob/master/src/ukf.cpp#L43).
   - The accuracy is:
        ``` C++ 
        >> Accuracy | rmse_px -> 0.0877133  | rmse_py -> 0.0968485  | rmse_vx -> 0.200318   | rmse_vy -> 0.250548  
        ```
3. The UKF uses both RADAR and LIDAR sensors measurement data:
   - Set [`use_laser_ = true`](https://github.com/mhBahrami/Unscented-Kalman-Filter/blob/master/src/ukf.cpp#L40) and [`use_radar_ = true`](https://github.com/mhBahrami/Unscented-Kalman-Filter/blob/master/src/ukf.cpp#L43).
   - The accuracy is:
        ``` C++
        >> Accuracy | rmse_px -> 0.0611963  | rmse_py -> 0.0838687  | rmse_vx -> 0.145292   | rmse_vy -> 0.284606
        ```

**As you can see, the best accuracy has been obtained if UKF uses the measurement data of both sensors.**
## License

[MIT License](https://github.com/mhBahrami/Unscented-Kalman-Filter/blob/master/LICENSE).
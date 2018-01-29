# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project, students are asked to implement the unsented Kalman Filter to estimate the state of a moving object of interest with noisy LiDAR and RADAR measurements.

## Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build and Run Instructions
To build the program, run the following sequence of command line instructions.

```bash
$ cd /path/to/cloned/repo
$ mkdir build
$ cd build
$ cmake ..
$ make
```

To see the program in action, make sure to download the [Term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases), run it, and choose the first simulation option. Then, execute the program by executing the `UnscentedKF` program in the `build` folder.

```bash
$ ./UnscentedKF
```

## Project Information
The Kalman filter is an algorithm that predicts an object's movement by using sensor data. Because sensor data is noisy and has a degree of uncertainty, the Kalman filter is used to provide a more accurate prediction of the object's actual movement. For more information on the Kalman filter, please see my [README for the Extended Kalman Filter Project](https://github.com/willhnguyen/CarND-Extended-Kalman-Filter-Project#kalman-filter).

### Unscented Kalman Filter
The unscented Kalman filter works with non-linear sensors similar to the extended Kalman filter. However, the unscented Kalman filter assumes that the distributions are always Gaussian. To proceed with that assumption, it chooses points around the mean (called sigma points) that it uses to further predict the object's location.

The amount of sigma points chosen is dependent on the size of the state vector. Sigma points include the current state (the mean) and points one unit away in each state parameter.

![Number of Sigma Points](https://latex.codecogs.com/gif.latex?n_%5Csigma%20%3D%202%20n_x%20&plus;%201)

Sigma points are chosen to be one covariant unit away from the mean in either direction of a particular state parameter. The ![lambda](https://latex.codecogs.com/gif.latex?%5Clambda) term is a tunable parameter that determines how far the chosen sigma points are from the mean point. It is usually kept at ![lambda equation](https://latex.codecogs.com/gif.latex?%5Clambda%20%3D%203%20-%20n_x). The following equation is how to calculate all sigma points—each column of the matrix should be represent a single sigma point.

![Sigma Derivation](https://latex.codecogs.com/gif.latex?X_%7Bk%7Ck%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20x_%7Bk%7Ck%7D%20%26%20x_k%20&plus;%20%5Csqrt%7B%28%5Clambda%20&plus;%20n_x%29%20P_%7Bk%7Ck%7D%7D%20%26%20x_k%20-%20%5Csqrt%7B%28%5Clambda%20&plus;%20n_x%29%20P_%7Bk%7Ck%7D%7D%29%20%5Cend%7Bbmatrix%7D)

[nu]: https://latex.codecogs.com/gif.latex?%5Cnu

If necessary, an augmented state vector that includes the noise term ![nu][nu] should be generated prior to sigma point selection. This is to ensure next state calculations are calculated properly. The CTRV model used requires this augmented state vector.

Once the sigma states are obtained, they are used in the prediction step and the measurement update step. In both steps, the state vector is updated by calculating the weighted average of each sigma point. The weights for each point are calculated using the following equation where a represents the number of augmented state parameters.

![Weight of mean state](https://latex.codecogs.com/gif.latex?w_%7B0%7D%20%3D%20%5Cfrac%7B%5Clambda%7D%7B%5Clambda%20&plus;%20n_a%7D)

![Weight of sigma states](https://latex.codecogs.com/gif.latex?w_%7Bi%20%5Cin%20%5Cleft%20%5C%7B%201%2C2%2C...%2Ca-1%20%5Cright%20%5C%7D%7D%20%3D%20%5Cfrac%7B1%7D%7B2%28%5Clambda%20&plus;%20n_a%29%7D)

The measurement update equation also requires new equations to accomodate for the usage of sigma points. The biggest part of this is the cross-correlation matrix T which affects how the Kalman gain matrix is calculated. All other equations should follow the standard Kalman filter algorithm's equations.

![](https://latex.codecogs.com/gif.latex?T_%7Bt&plus;1%7Ct%7D%20%3D%20%5Csum_%7Bi%3D1%7D%5E%7Bn_%5Csigma%7D%20w_i%20%28X_%7Bt&plus;1%7Ct%2Ci%7D%20-%20x_%7Bt&plus;1%7Ct%7D%29%28Z_%7Bt&plus;1%7Ct%2Ci%7D%20-%20z_%7Bt&plus;1%7Ct%7D%29%5ET)

![](https://latex.codecogs.com/gif.latex?K_%7Bt&plus;1%7Ct%7D%20%3D%20T_%7Bt&plus;1%7Ct%7D%20S_%7Bt&plus;1%7Ct%7D%5E%7B-1%7D)

![](https://latex.codecogs.com/gif.latex?x_%7Bt&plus;1%7Ct&plus;1%7D%20%3D%20x_%7Bt&plus;1%7Ct%7D%20&plus;%20K_%7Bt&plus;1%7Ct%7D%28z_%7Bt&plus;1%7D%20-%20z_%7Bt&plus;1%7Ct%7D%29)

![](https://latex.codecogs.com/gif.latex?P_%7Bt&plus;1%7Ct&plus;1%7D%20%3D%20P_%7Bt&plus;1%7Ct%7D%20-%20K_%7Bt&plus;1%7Ct%7D%20S_%7Bt&plus;1%7Ct%7DK_%7Bt&plus;1%7Ct%7D%5ET)

[psi]: https://latex.codecogs.com/gif.latex?%5Cpsi
[psi dot]: https://latex.codecogs.com/gif.latex?%5Cdot%20%5Cpsi

### The CTRV Model
For this project, the constant turn rate and velocity model (CTRV) is used. In the previous project, the constant velocity model is used. For CTRV, the state includes the object's angle of movement ![psi][psi] and its turn rate ![psi dot][psi dot].

![CTRV state vector](https://latex.codecogs.com/gif.latex?x%20%3D%20%5Cbegin%7Bbmatrix%7D%20p_x%5C%5C%20p_y%5C%5C%20v%5C%5C%20%5Cpsi%5C%5C%20%5Cdot%20%5Cpsi%20%5Cend%7Bbmatrix%7D)

The prediction step will take the turn rate into account. This requires the augmented state vector which include the ![nu][nu] terms. However, the output vector should not include the ![nu][nu] terms.

![Prediction Equation](https://latex.codecogs.com/gif.latex?x_%7Bt&plus;1%7D%20%3D%20x_t%20&plus;%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7Bv_k%7D%7B%5Cdot%20%5Cpsi_k%7D%20%28sin%20%28%5Cpsi_k%20&plus;%20%5Cdot%20%5Cpsi_k%20%5CDelta%20t%29%20-%20sin%28%5Cpsi_k%29%29%29%5C%5C%20%5Cfrac%7Bv_k%7D%7B%5Cdot%20%5Cpsi_k%7D%20%28-cos%20%28%5Cpsi_k%20&plus;%20%5Cdot%20%5Cpsi_k%20%5CDelta%20t%29%20&plus;%20cos%28%5Cpsi_k%29%29%5C%5C%200%5C%5C%20%5Cdot%20%5Cpsi_k%20%5CDelta%20t%5C%5C%200%20%5Cend%7Bbmatrix%7D%20&plus;%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7B1%7D%7B2%7D%28%5CDelta%20t%29%5E2%20cos%28%5Cpsi_k%29%5Cnu_%7Ba%2Ck%7D%5C%5C%20%5Cfrac%7B1%7D%7B2%7D%28%5CDelta%20t%29%5E2%20sin%28%5Cpsi_k%29%5Cnu_%7Ba%2Ck%7D%5C%5C%20%5CDelta%20t%20%5Ccdot%20%5Cnu_%7Ba%2Ck%7D%5C%5C%20%5Cfrac%7B1%7D%7B2%7D*%28%5CDelta%20t%29%5E2%20%5Cnu_%7B%5Cddot%20%5Cpsi%2C%20k%7D%5C%5C%20%5CDelta%20t%20%5Ccdot%20%5Cnu_%7B%5Cddot%20%5Cpsi%2Ck%7D%20%5Cend%7Bbmatrix%7D)

If ![psi][psi] is 0, then the following equation is used. Otherwise, a divide-by-zero error may occur.

![Prediction Equation psi 0](https://latex.codecogs.com/gif.latex?x_%7Bt&plus;1%7D%20%3D%20x_t%20&plus;%20%5Cbegin%7Bbmatrix%7D%20v_k%20cos%28%5Cpsi_k%29%5CDelta%20t%5C%5C%20v_k%20sin%28%5Cpsi_k%29%5CDelta%20t%5C%5C%200%5C%5C%20%5Cdot%20%5Cpsi_k%20%5CDelta%20t%5C%5C%200%20%5Cend%7Bbmatrix%7D%20&plus;%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7B1%7D%7B2%7D%28%5CDelta%20t%29%5E2%20cos%28%5Cpsi_k%29%5Cnu_%7Ba%2Ck%7D%5C%5C%20%5Cfrac%7B1%7D%7B2%7D%28%5CDelta%20t%29%5E2%20sin%28%5Cpsi_k%29%5Cnu_%7Ba%2Ck%7D%5C%5C%20%5CDelta%20t%20%5Ccdot%20%5Cnu_%7Ba%2Ck%7D%5C%5C%20%5Cfrac%7B1%7D%7B2%7D*%28%5CDelta%20t%29%5E2%20%5Cnu_%7B%5Cddot%20%5Cpsi%2C%20k%7D%5C%5C%20%5CDelta%20t%20%5Ccdot%20%5Cnu_%7B%5Cddot%20%5Cpsi%2Ck%7D%20%5Cend%7Bbmatrix%7D)

## Project Implemenation Details
Following the provided starter code, the `src/ukf.cpp` and `src/tools.cpp` files were modified as necessary. For this project, `src/ufk.h` was also modified to add an extra class variable.

The updated `src/ukf.cpp` file includes variable initialization and code that controls how RADAR and LiDAR data are processed using the unscented Kalman filter algorithm.

The updated `src/tools.cpp` file includes a root mean square error calculator.

Research Computing Coursework 1
===============================

Build and Execution
===================
The build can be in-source or out-of-source. ITK has been removed as a dependency because it is not used. Boost is used, although it no longer has a new copy installed and rebuilt with the project. Should this be necessary, the relevant Cmake changes can be reverted.

To execute either of the command-line utilities, run `bin/PointMatchingCmd` or `bin/SurfaceBasedRegistrationCmd` with command-line arguments as documented in the unit tests. To run unit tests, cd to `bin`, then execute `./PointMatchingTest`.

Point-Based Registration
==================
Point-based registration is implemented according to Arun et al (1987), with this project mostly taking an imperative/functional approach. Several small functions are used (and reused) in combination to achieve the desired end result. This seemed appropriate for a small, numerically-focused piece of software. The relevant high-level function here is `estimate_rigid_transform`, which takes two point clouds as input, and returns a 4x4 estimated transformation matrix. Point clouds are stored as `Eigen::MatrixXd` types -- i.e. 3xN Eigen matrices of 3x1 vectors. This allows the use of the various Eigen numerical functions, with little computational overhead. `Eigen::MatrixXd` is dynamically-allocated, allowing the project to avoid manual memory allocation for the most part. These point clouds are, of course, passed by const reference. This is important because it prevents unnecessary copying and undesired side-effects.

Unit tests are implemented in `bin/PointMatchingTest`. These tests serve as the main documentation for the program. There is also a command-line interface to the point-based registration, through PointMatchingCmd. If one uses the registration on the provided test-data (as is done in the relevant unit test), one gets the expected result:

|----------|-------------|------------|--------------|
| 0.743145 | 3.06888e-07 |  -0.669131 | -2.45377e-05 |
| 0.421098 |    0.777146 |  -0.467676 |   2.7305e-05 |
| 0.520012 |     0.62932 |   0.577532 |  0.000109916 |
|        0 |           0 |          0 |            1 |
|----------|-------------|------------|--------------|

This implementation of point-based registration is relatively naive, making equal use of every point in the SVD. As the number of points grows very large, the program can become slow to find the residuals matrix (the 3x3 matrix on which the SVD is performed) -- especially when the available memory is exceeded, necessitating paging. If it were necessary to handle large numbers of points, it might be wise to take a representative subset of points, and register these. Alternatively, evaluation of the residuals matrix prior to SVD might be parallelised.

Surfaced-Based Registration
===========================
Surface-based registration is implemented in a similar fashion to (and making use of) the aforementioned point-based registration, as suggested in the briefing sheet. The relevant higher-level function here is `register_surfaces`, which takes two point clouds and (optionally) an initial transformation as input. As with the point-based registration, it returns an estimated transformation as a 4x4 matrix. Surfaces are assumed to be represented by point clouds, in which there is no prior knowledge as to which points correspond. At every step, the estimated transform is applied, and then there is an exhausive search for the closest "floating" point to each "fixed" point. This correspondence is then used to update the estimated transform.

The exhaustive search for closest points is quite naive, and prone to local minima, and a better approach might be to use a stochastic global optimisation. It performs adequately for this example, however. Relatedly, providing a good initialisation transform is important for avoiding local minima. This would be less important if a coarse subset of points were globally optimised to initially estimate the correspondence and transform. Currently, using a good initialisation, the example data can be successfully registered with high accuracy in under ten seconds on a modern laptop.

Unit-testing of the whole surface-based registration (as opposed to a smaller unit) is a form of integration testing. This is carried out within the previously-discussed tests. 

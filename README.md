Requirements:

Covariance Tuning:

For Spherical Coordinate:
R(0, 0) = 1e-5;
R(1, 1) = 5e-2;
R(2, 2) = 5e-2;

If all robots have their own absolute position measurement, with rate >2hz
Q.block(0, 0, 3, 3) = 1e-4*Eigen::MatrixXf::Identity(3, 3); // position
Q.block(3, 3, 3, 3) = 1e-3*Eigen::MatrixXf::Identity(3, 3); // velocity

R(0, 0) = 1e-5;
R(1, 1) = 1e-2;
R(2, 2) = 1e-2;

If only one robot has its own absolute position measurement, with rate = 10hz
Q.block(0, 0, 3, 3) = 1e-4*Eigen::MatrixXf::Identity(3, 3); // position
Q.block(3, 3, 3, 3) = 12e-2*Eigen::MatrixXf::Identity(3, 3); // velocity
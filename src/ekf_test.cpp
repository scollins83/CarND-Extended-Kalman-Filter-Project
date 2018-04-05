//
// Created by Sara Collins on 3/29/18.
//
#include "gtest/gtest.h"
#include "FusionEKF.h"
#include "Eigen/Dense"
#include <vector>

TEST(FusionEKF, test_polar_to_cartesian_conversion){
    FusionEKF test_ekf;
    float rho = 1.014892e+00;
    float phi = 5.543292e-01;
    float rho_dot = 4.892807e+00;
    Eigen::VectorXd actual_cartesian = VectorXd(4);
    actual_cartesian = test_ekf.convert_radar_from_polar_to_cartesian(rho, phi, rho_dot);
    Eigen::VectorXd expected_cartesian = VectorXd(4);
    expected_cartesian << 0, 0, 0, 0;
    ASSERT_EQ(actual_cartesian[0], expected_cartesian[0]);
}
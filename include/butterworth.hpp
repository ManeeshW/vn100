#ifndef BUTTERWORTH_HPP
#define BUTTERWORTH_HPP

#include <Eigen/Dense>

/**
 * @brief Butterworth low-pass filter class.
 *
 * This class implements a digital Butterworth low-pass filter using the bilinear transform.
 * It supports arbitrary filter orders and can filter multi-dimensional vectors by applying
 * the filter independently to each dimension. The class is designed to be modular and
 * self-contained, requiring only the Eigen library for vector operations.
 *
 * Usage example:
 *   // Create a 2nd-order filter with 10 Hz cutoff at 200 Hz sampling, for 3D data
 *   ButterworthFilter filter(2, 10.0, 200.0, 3);
 *
 *   // Filter a data vector
 *   Eigen::VectorXd input(3);
 *   input << 1.0, 2.0, 3.0;
 *   Eigen::VectorXd output = filter.update(input);
 */
class ButterworthFilter {
private:
    Eigen::VectorXd b, a;
    int order;
    Eigen::MatrixXd x_hist, y_hist;

public:
    ButterworthFilter();  // New default constructor

    /**
     * @brief Construct a filter with given numerator and denominator coefficients.
     * @param b_ Numerator coefficients (b0, b1, ..., bn).
     * @param a_ Denominator coefficients (a0=1, a1, ..., an).
     * @param dim Dimension of the input data vectors.
     */
    ButterworthFilter(const Eigen::VectorXd& b_, const Eigen::VectorXd& a_, int dim);

    /**
     * @brief Construct a Butterworth low-pass filter.
     * @param order_ Filter order (positive integer >=1).
     * @param cutoff Cutoff frequency in Hz.
     * @param fs Sampling frequency in Hz.
     * @param dim Dimension of the input data vectors.
     */
    ButterworthFilter(int order_, double cutoff, double fs, int dim);

    /**
     * @brief Apply the filter to new data.
     * @param data Input vector (must match the dimension specified at construction).
     * @return Filtered output vector.
     */
    Eigen::VectorXd update(const Eigen::VectorXd& data);
};

#endif
#include "butterworth.hpp"
#include <cmath>
#include <vector>
#include <stdexcept>

ButterworthFilter::ButterworthFilter() : order(0) {}  // New default constructor (empty filter)

namespace {

std::vector<double> poly1_sign_v(int power, int sign) {
  if (power == 0) return {1.0};
  std::vector<double> poly(static_cast<size_t>(power) + 1, 0.0);
  poly[0] = 1.0;
  poly[1] = static_cast<double>(sign);
  for (int p = 2; p <= power; ++p) {
    std::vector<double> new_poly(static_cast<size_t>(power) + 1, 0.0);
    for (size_t i = 0; i < static_cast<size_t>(p); ++i) {
      new_poly[i] += poly[i];
      if (i + 1 < new_poly.size()) new_poly[i + 1] += static_cast<double>(sign) * poly[i];
    }
    poly = std::move(new_poly);
  }
  return poly;
}

std::vector<double> poly_mult(const std::vector<double>& p1, const std::vector<double>& p2) {
  size_t d1 = p1.size() - 1;
  size_t d2 = p2.size() - 1;
  std::vector<double> res(d1 + d2 + 1, 0.0);
  for (size_t i = 0; i <= d1; ++i) {
    for (size_t j = 0; j <= d2; ++j) {
      res[i + j] += p1[i] * p2[j];
    }
  }
  return res;
}

} // namespace

ButterworthFilter::ButterworthFilter(const Eigen::VectorXd& b_, const Eigen::VectorXd& a_, int dim)
    : b(b_), a(a_), order(static_cast<int>(b.size() - 1)), x_hist(Eigen::MatrixXd::Zero(dim, order)), y_hist(Eigen::MatrixXd::Zero(dim, order)) {
  if (b.size() != a.size() || order < 1 || std::abs(a(0) - 1.0) > 1e-10) {
    throw std::invalid_argument("Invalid filter coefficients: b and a must be same size, order >=1, and a[0] == 1.0");
  }
}

ButterworthFilter::ButterworthFilter(int order_, double cutoff, double fs, int dim)
    : order(order_) {
  if (order < 1) {
    throw std::runtime_error("Order must be at least 1.");
  }
  if (cutoff <= 0 || fs <= 0 || cutoff >= fs / 2) {
    throw std::invalid_argument("Invalid cutoff or sampling frequency");
  }
  double omega = std::tan(M_PI * cutoff / fs);

  // Compute analog polynomial coefficients for normalized Butterworth (cutoff=1)
  std::vector<double> analog_poly{1.0};
  if (order % 2 == 1) {
    std::vector<double> real_factor{1.0, 1.0};  // (s + 1)
    analog_poly = poly_mult(analog_poly, real_factor);
  }
  for (int k = 1; k <= order / 2; ++k) {
    double alpha = (2.0 * k - 1.0) * M_PI / (2.0 * order);
    double coeff = 2.0 * std::sin(alpha);
    std::vector<double> quad{1.0, coeff, 1.0};  // s^2 + coeff s + 1
    analog_poly = poly_mult(analog_poly, quad);
  }

  // Compute denominator polynomial using bilinear transform
  std::vector<double> den_poly(static_cast<size_t>(order) + 1, 0.0);
  for (int k = 0; k <= order; ++k) {
    double scale = analog_poly[static_cast<size_t>(k)] * std::pow(omega, -k);
    auto p1 = poly1_sign_v(k, -1);  // (1 - z)^k
    auto p2 = poly1_sign_v(order - k, 1);  // (1 + z)^{order - k}
    auto term_poly = poly_mult(p1, p2);
    for (size_t i = 0; i <= static_cast<size_t>(order); ++i) {
      den_poly[i] += scale * term_poly[i];
    }
  }

  double den0 = den_poly[0];
  if (std::abs(den0) < 1e-10) {
    throw std::runtime_error("Denominator constant term is zero.");
  }

  a = Eigen::VectorXd(order + 1);
  a(0) = 1.0;
  for (int i = 1; i <= order; ++i) {
    a(i) = den_poly[static_cast<size_t>(i)] / den0;
  }

  auto num_poly = poly1_sign_v(order, 1);  // (1 + z)^order
  double scale_b = 1.0 / den0;
  b = Eigen::VectorXd(order + 1);
  for (int i = 0; i <= order; ++i) {
    b(i) = scale_b * num_poly[static_cast<size_t>(i)];
  }

  x_hist = Eigen::MatrixXd::Zero(dim, order);
  y_hist = Eigen::MatrixXd::Zero(dim, order);
}

Eigen::VectorXd ButterworthFilter::update(const Eigen::VectorXd& data) {
  if (data.size() != x_hist.rows()) {
    throw std::invalid_argument("Input data dimension mismatch");
  }
  Eigen::VectorXd new_x = data;
  Eigen::VectorXd new_y = b(0) * new_x;
  for (int i = 1; i <= order; ++i) {
    new_y += b(i) * x_hist.col(i - 1);
    new_y -= a(i) * y_hist.col(i - 1);
  }
  for (int i = order - 1; i > 0; --i) {
    x_hist.col(i) = x_hist.col(i - 1);
    y_hist.col(i) = y_hist.col(i - 1);
  }
  x_hist.col(0) = new_x;
  y_hist.col(0) = new_y;
  return new_y;
}
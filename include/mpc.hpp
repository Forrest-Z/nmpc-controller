/**
 * Copyright (c) 2021 Ashwin A Nayar
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <cppad/cppad.hpp>
#include <eigen3/Eigen/Core>

/**
 * Utilities/helpers for NMPC
 */
namespace mpc::utils {
    /**
     * Evaluate a polynomial
     *
     * @param coeffs: Coefficients of the polynomial, constant term first
     * @param x: The X value at which the polynomial is to be evaluated
     *
     * @return Result
     */
    double polyeval(const Eigen::VectorXd &coeffs, double x);

    /**
     * Find best fit polynomial coefficients
     *
     * @param xvals: The x coordinates
     * @param yvals: The y coordinates
     * @param order: Order of the polynomial
     *
     * @return The coefficients
     */
    Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals,
                            const Eigen::VectorXd &yvals, int order);
} // namespace mpc::utils

namespace mpc {
    struct Params
    {
        struct
        {
            /// Number of timesteps in the prediction horizon
            size_t timesteps;
            /// Sample time of the controller
            double dt;
        } forward;

        /// Desired errors and velocity, plant will try to achieve these / stay close
        /// to these values
        struct
        {
            double cte;
            double etheta;
            double vel;
        } desired;

        /// This stores the constraints
        struct
        {
            struct
            {
                double min, max;
            } omega, throttle;
        } limits;

        /// This will be used as the default constraint
        const double BOUND_VALUE;

        /// Weights for the cost function
        struct Weights
        {
            double vel, cte, etheta, omega, acc, omega_d, acc_d;
        } weights;

        Params();
    };

    /// Helper struct to store indices of variables
    struct VarIndices
    {
        size_t x_start, y_start, theta_start;
        size_t v_start, omega_start, acc_start;
        size_t cte_start, etheta_start;

        /**
         * Constructor
         *
         * Initialises all indicies based on the timesteps
         */
        explicit VarIndices(size_t timesteps);
    };

    /// Main class for MPC implementation
    class MPC
    {
    public:
        typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

        /**
         * Constructor
         *
         * @param params: The parameters for the MPC
         * @param coeffs: The coefficients of the best fit polynomial
         */
        MPC(const Params &params, const Eigen::VectorXd &coeffs);

        /// Destructor
        ~MPC();

        void operator()(ADvector &fg, const ADvector &vars) const;

        /**
         * Solve the NLP
         *
         * @param state: Current state of the model
         *
         * @return Vector of manipulated variables and other parameters like cost
         */
        std::vector<double> solve(Eigen::VectorXd &state);

    private:
        const Params m_Params;
        const Eigen::VectorXd m_Coeffs;
        const VarIndices m_VarIndices;
    };
} // namespace mpc

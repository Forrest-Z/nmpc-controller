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

#include "model.hpp"
#include "mpc.hpp"

#include <benchmark/benchmark.h>

static void BM_NMPCloop(benchmark::State &bmState)
{
    mpc::Params params;

    model::DifferentialDrive dModel;

    params.forward.timesteps = 12;
    params.forward.dt = 0.1;

    params.desired.vel = 0.5;
    params.desired.cte = 0.0;
    params.desired.etheta = 0.0;

    params.limits.omega = {-2.0, 2.0};
    params.limits.throttle = {-1.0, 1.0};

    params.weights.cte = 87.859183;
    params.weights.etheta = 99.532785;
    params.weights.vel = 54.116644;
    params.weights.omega = 47.430096;
    params.weights.acc = 2.185306;
    params.weights.omega_d = 4.611500;
    params.weights.acc_d = 66.870729;

    dModel.setSampleTime(params.forward.dt);
    dModel.setInitState(model::State({-8.0, 1.5, -0.6, 0.0, 0.0, 0.0}));

    std::array<double, 40> ptsx;
    std::array<double, 40> ptsy;

    const model::State state = dModel.getState();

    for (size_t i = 0; i < 40; i++)
    {
        ptsx[i] = state.x + i * 0.1;
        ptsy[i] = 0.0;
    }

    double px = state.x;
    double py = state.y;
    double theta = state.theta;
    double v = state.linVel;
    double omega = state.angVel;
    double throttle = state.throttle;

    double shift_x, shift_y;
    for (size_t i = 0; i < ptsx.size(); i++)
    {
        shift_x = ptsx[i] - px;
        shift_y = ptsy[i] - py;
        ptsx[i] = shift_x * cos(-theta) - shift_y * sin(-theta);
        ptsy[i] = shift_x * sin(-theta) + shift_y * cos(-theta);
    }

    double *ptrx = &ptsx[0];
    Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

    double *ptry = &ptsy[0];
    Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

    const Eigen::VectorXd &coeffs =
        mpc::utils::polyfit(ptsx_transform, ptsy_transform, 3);

    double cte = mpc::utils::polyeval(coeffs, 0);
    double etheta = -atan(coeffs[1]);

    double dt = params.forward.dt;
    double current_px = 0.0 + v * dt;
    double current_py = 0.0;
    double current_theta = 0.0 + omega * dt;
    double current_v = v + throttle * dt;
    double current_cte = cte + v * sin(etheta) * dt;
    double current_etheta = etheta - current_theta;

    Eigen::VectorXd model_state(6);
    model_state << current_px, current_py, current_theta, current_v, current_cte,
        current_etheta;

    mpc::MPC _mpc(params, coeffs);

    for (auto _ : bmState)
    {
        // This code gets timed
        std::vector<double> solns = _mpc.solve(model_state);
    }
}

// Register the function as a benchmark
BENCHMARK(BM_NMPCloop);

// Run the benchmark
BENCHMARK_MAIN();
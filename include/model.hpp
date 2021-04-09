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

#include <vector>

namespace model
{
    /**
     * Struct for holding the state info of the model at a given time
     */
    struct State
    {
        double x, y, theta, linVel, angVel, throttle;
    };

    class DifferentialDrive
    {
    public:
        /// Constructor
        DifferentialDrive();

        /**
         * Set the sample time for each step of the model
         * 
         * @param sampletime: The sampletime
         */
        void setSampleTime(double sampletime);

        /**
         * Update the model state by one timestep
         * 
         * @param speed: The speed of the model
         * @param omega: The angular velocity
         */
        void step(double speed, double omega);

        /**
         * Get current state of the model
         * 
         * @return Current state
         */
        State getState() const;

        void setInitState(const State &state);

        /**
         * Reset model state
         */
        void reset();

    private:
        /// Current state of the model
        State m_state;

        /// Store initial values for reset later
        State m_initState;

        double m_sampleTime;
    };
} // namespace model

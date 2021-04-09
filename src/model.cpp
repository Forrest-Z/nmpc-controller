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
#include <cmath>

namespace model
{    
    DifferentialDrive::DifferentialDrive()
    {
    }

    void DifferentialDrive::setSampleTime(double sampletime)
    {
        m_sampleTime = sampletime;
    }

    void DifferentialDrive::step(double speed, double omega)
    {
        m_state.x += m_state.linVel * cos(m_state.theta) * m_sampleTime;
        m_state.y += m_state.linVel * sin(m_state.theta) * m_sampleTime;

        m_state.theta += omega * m_sampleTime;
        m_state.throttle = (speed - m_state.linVel) / m_sampleTime;
        m_state.linVel = speed;
        m_state.angVel = omega;
    }

    State DifferentialDrive::getState() const
    {
        return m_state;
    }

    void DifferentialDrive::setInitState(const State &state)
    {
        m_state = state;
        m_initState = state;
    }

    void DifferentialDrive::reset()
    {
        m_state = m_initState;
    }
} // namespace model
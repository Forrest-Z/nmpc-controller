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

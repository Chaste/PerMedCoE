/*

Copyright (c) 2005-2025, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "AbstractForce.hpp"
#include "SimulationTime.hpp"

template <unsigned int DIM>
class InstantaneousVelocityForce : public AbstractForce<DIM> {
    private:
        c_vector<double, DIM> velocity;

    public:
        // Default constructor
        InstantaneousVelocityForce() {
            for (unsigned int i = 0; i < DIM; i++) {
                velocity[i] = 0.0;
            }
        }
        
        // Constructor with velocity
        InstantaneousVelocityForce(const c_vector<double, DIM> velocity) : velocity(velocity) {};
    
        void SetVelocity(const c_vector<double, DIM> velocity) {
            for (unsigned int i = 0; i < DIM; i++) {
                this->velocity[i] = velocity[i];
            }
        }

        void AddForceContribution(AbstractCellPopulation<DIM>& cellPopulation) override {

            // Only apply the force if this is the first timestep
            const unsigned int stepsElapsed = SimulationTime::Instance()->GetTimeStepsElapsed();

            if (stepsElapsed == 0) {
                // Apply the force
                for (unsigned int i = 0; i < cellPopulation.GetNumNodes(); i++) {
                    cellPopulation.GetNode(i)->AddAppliedForceContribution(velocity);
                }
            }
        }
   
        void OutputForceParameters(out_stream& paramsFile) override {

        }
};
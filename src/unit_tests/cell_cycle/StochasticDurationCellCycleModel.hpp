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

#ifndef STOCHASTICDURATIONCELLCYCLEMODEL_HPP_
#define STOCHASTICDURATIONCELLCYCLEMODEL_HPP_

#include "AbstractSimplePhaseBasedCellCycleModel.hpp"

// "Simple" cell-cycle model: phase durations are set when the model is created.
class StochasticDurationCellCycleModel : public AbstractSimplePhaseBasedCellCycleModel
{
private:
    // For archiving (saving or loading) the cell-cycle model object in a cell-based simulation.  
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version);

    void SetPhaseDurations();
    void SetG1Duration() override;

public:
    StochasticDurationCellCycleModel();

    // Override builder method for new copies of the cell-cycle model.
    AbstractCellCycleModel* CreateCellCycleModel() override;
    
    // Override to start phase from G1 and reset durations after each cycle.
    void UpdateCellCyclePhase() override;
};

// Provides a unique identifier for the custom cell-cycle model.
// Needed for archiving and for writing out parameters file.
// Simulations will throw errors if missing.
#include "SerializationExportWrapper.hpp"
CHASTE_CLASS_EXPORT(StochasticDurationCellCycleModel)

#endif // STOCHASTICDURATIONCELLCYCLEMODEL_HPP_

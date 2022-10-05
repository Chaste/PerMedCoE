/*

Copyright (c) 2005-2022, University of Oxford.
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

#include <algorithm>
#include "AbstractSimpleGenerationalCellCycleModel.hpp"
#include "RandomNumberGenerator.hpp"

#include "StochasticDurationCellCycleModel.hpp"

template<class Archive>
void StochasticDurationCellCycleModel::serialize(Archive & archive, const unsigned int version)
{
    // Archive cell-cycle model using serialization code from AbstractSimpleGenerationalCellCycleModel
    archive & boost::serialization::base_object<AbstractSimpleGenerationalCellCycleModel>(*this);
    
    // Archive RandomNumberGenerator singleton.
    // Must be done carefully: first serialize directly, then via pointer.
    // This prevents tripping an assertion when a second class instance is created on de-serialization.
    RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();
    archive & *p_gen; // First serialize directly
    archive & p_gen; // Then serialize via pointer
}

void StochasticDurationCellCycleModel::SetG1Duration()
{
    assert(mpCell != NULL);  // Make sure cell exists
    
    mG1Duration = RandomNumberGenerator::Instance()->NormalRandomDeviate(7.0, 0.7);
}

AbstractCellCycleModel* StochasticDurationCellCycleModel::CreateCellCycleModel()
{
    // Create a new cell-cycle model
    StochasticDurationCellCycleModel* p_model = new StochasticDurationCellCycleModel();

    // Inherit values from parent
    p_model->SetBirthTime(mBirthTime);
    p_model->SetGeneration(mGeneration);
    p_model->SetMaxTransitGenerations(mMaxTransitGenerations);

    // Set phase durations
    double g1Duration = p_model->GetG1Duration();
    p_model->SetStemCellG1Duration(g1Duration);
    p_model->SetTransitCellG1Duration(g1Duration);

    double sDuration = RandomNumberGenerator::Instance()->NormalRandomDeviate(6.0, 0.6);
    double g2Duration = RandomNumberGenerator::Instance()->NormalRandomDeviate(3.0, 0.3);
    double mDuration = RandomNumberGenerator::Instance()->NormalRandomDeviate(2.0, 0.2);
    p_model->SetSDuration(sDuration);
    p_model->SetG2Duration(g2Duration);
    p_model->SetMDuration(mDuration);
    
    p_model->SetMinimumGapDuration(std::min(g1Duration, g2Duration));

    // Notes:
    // Already initialized in constructor: mBirthTime, mCurrentCellCyclePhase, mReadyToDivide.
    // Not set: mDimension. Spatial dimension not required. SetDimension() will trigger exception. 
    // InitialiseDaughterCell() can set/overwrite member variables e.g. called from Divide().

    return p_model;
}

#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(StochasticDurationCellCycleModel)

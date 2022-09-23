/*

Copyright (c) 2005-2021, University of Oxford.
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

#include "CheckpointArchiveTypes.hpp"
#include "SmartPointers.hpp"
#include "Exception.hpp"
#include "RandomNumberGenerator.hpp"

#include "AbstractSimpleGenerationalCellCycleModel.hpp"

// This cell-cycle model is simple i.e. the duration of each phase is determined when the cell-cycle model is created.
// It is also generation-based i.e. it keeps track of the generation of the corresponding cell, and sets the cell type accordingly.
class StochasticDurationCellCycleModel : public AbstractSimpleGenerationalCellCycleModel
{
private:
    // We wish to archive (save or load) the cell-cycle model object in a cell-based simulation. 
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        // Archive the cell cycle model using the serialization code defined in AbstractSimpleGenerationalCellCycleModel.
        archive & boost::serialization::base_object<AbstractSimpleGenerationalCellCycleModel>(*this);
        
        // Archive the RandomNumberGenerator singleton.
        // Must be done with care. First serialize directly, then via pointer.
        // This prevents tripping an assertion when a second instance of the class is created on de-serialization.
        RandomNumberGenerator* p_gen = RandomNumberGenerator::Instance();
        archive & *p_gen; // First serialize directly
        archive & p_gen; // Then serialize via pointer
    }

    // Override phase duration methods
    void SetG1Duration()
    {
        assert(mpCell != NULL);  // Make sure cell exists
        
        mG1Duration = RandomNumberGenerator::Instance()->NormalRandomDeviate(7.0, 0.7);
    }

    void SetSDuration()
    {
        assert(mpCell != NULL);  // Make sure cell exists

        mSDuration = RandomNumberGenerator::Instance()->NormalRandomDeviate(6.0, 0.6);
    }

    void SetG2Duration()
    {
        assert(mpCell != NULL);  // Make sure cell exists

        mG2Duration = RandomNumberGenerator::Instance()->NormalRandomDeviate(3.0, 0.3);
    }
    
    void SetMDuration()
    {
        assert(mpCell != NULL);  // Make sure cell exists

        mMDuration = RandomNumberGenerator::Instance()->NormalRandomDeviate(2.0, 0.2);
    }

public:

    StochasticDurationCellCycleModel()
    {}

    // Override CreateCellCycleModel(). 
    // This is a builder method to create new copies of the cell-cycle model.
    // We first create a new cell-cycle model, then set each member variable of the new cell-cycle model that inherits its value from the parent.

    // Some inherited member variables are not set here. This is for two main reasons:
    // First, some of the new cell-cycle model's member variables (namely mBirthTime, mCurrentCellCyclePhase, mReadyToDivide) will already have been correctly initialized in the new cell-cycle model's constructor. 
    // Second, the member variable mDimension remains unset, since this cell-cycle model does not need to know the spatial dimension, so if we were to call SetDimension() on the new cell-cycle model an exception would be triggered; 
    // hence we do not set this member variable. It is also worth noting that in a simulation, one or more of the new cell-cycle model's member variables may be set/overwritten as soon as InitialiseDaughterCell() is called on the new cell-cycle model; 
    // this occurs when the associated cell has called its Divide() method.

    AbstractCellCycleModel* CreateCellCycleModel()
    {
        StochasticDurationCellCycleModel* p_model = new StochasticDurationCellCycleModel();

        p_model->SetBirthTime(mBirthTime);
        p_model->SetStemCellG1Duration(mG1Duration);
        p_model->SetTransitCellG1Duration(mG1Duration);
        //p_model->SetMinimumGapDuration(3.0);
        p_model->SetGeneration(mGeneration);
        p_model->SetMaxTransitGenerations(mMaxTransitGenerations);

        return p_model;
    }
};

// We want to archive (save or load) the cell-cycle model object in a cell-based simulation.
// We also need this for writing out the parameters file describing the settings for a simulation - 
// it provides the unique identifier for our new cell-cycle model. 
// Thus every cell-cycle model class must provide this, or you'll get errors when running simulations.
#include "SerializationExportWrapper.hpp"
CHASTE_CLASS_EXPORT(StochasticDurationCellCycleModel)

// This is the same as above, but  for newer versions of the Boost libraries
#include "SerializationExportWrapperForCpp.hpp"
CHASTE_CLASS_EXPORT(StochasticDurationCellCycleModel)

#endif /*STOCHASTICDURATIONCELLCYCLEMODEL_HPP_*/

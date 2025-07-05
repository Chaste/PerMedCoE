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

#include "StoppingVelocityModifier.hpp"

template<unsigned DIM>
StoppingVelocityModifier<DIM>::StoppingVelocityModifier()
    : AbstractCellBasedSimulationModifier<DIM>(),
      mMinSeparation(1.0)
{
}

template<unsigned DIM>
StoppingVelocityModifier<DIM>::~StoppingVelocityModifier()
{
}

template<unsigned DIM>
void StoppingVelocityModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    ChasteCuboid<DIM> bounds = rCellPopulation.rGetMesh().CalculateBoundingBox();
    double x_min = bounds.rGetLowerCorner()[0];
    double x_max = bounds.rGetUpperCorner()[0];

    double width = x_max - x_min;
    
    if (width < mMinSeparation)
    {
        for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
             cell_iter != rCellPopulation.End();
             ++cell_iter)
        {
            cell_iter->GetCellData()->SetItem("x_velocity", 0.0);
        }
    }
}

template<unsigned DIM>
void StoppingVelocityModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
}


template<unsigned DIM>
void StoppingVelocityModifier<DIM>::SetMinSeparation(double MinSeparation)
{
    mMinSeparation = MinSeparation;
}

template<unsigned DIM>
double StoppingVelocityModifier<DIM>::GetMinSeparation()
{
    return mMinSeparation;
}

template<unsigned DIM>
void StoppingVelocityModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<MinSeparation>" << mMinSeparation << "</MinSeparation>\n";

    // Next, call method on direct parent class
    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class StoppingVelocityModifier<1>;
template class StoppingVelocityModifier<2>;
template class StoppingVelocityModifier<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(StoppingVelocityModifier)
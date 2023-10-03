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

#include <cmath>

#include "GrowthModifier.hpp"

template<unsigned DIM>
GrowthModifier<DIM>::GrowthModifier()
    : AbstractCellBasedSimulationModifier<DIM>()
{
}

template<unsigned DIM>
GrowthModifier<DIM>::~GrowthModifier()
{
}

template<unsigned DIM>
void GrowthModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void GrowthModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
    /*
     * We must update CellData in SetupSolve(), otherwise it will not have been
     * fully initialised by the time we enter the main time loop.
     */
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void GrowthModifier<DIM>::UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    rCellPopulation.Update(); // Make sure the cell population is updated

    // Get volume for each cell
    for (typename AbstractCellPopulation<DIM>::Iterator pCell = rCellPopulation.Begin();
         pCell != rCellPopulation.End();
         ++pCell)
    {
        double cellVolume = rCellPopulation.GetVolumeOfCell(*pCell);
        pCell->GetCellData()->SetItem("volume", cellVolume);
    }
    
    // Set target radius for each cell
    for (typename AbstractCellPopulation<DIM>::Iterator pCell = rCellPopulation.Begin();
         pCell != rCellPopulation.End();
         ++pCell)
    {
        double initialTargetArea = 0.5; // todo: fix magic number from target area modifier
        double targetArea = pCell->GetCellData()->GetItem("target area");
        double sizeFactor = targetArea / initialTargetArea;

        double initialRadius = 0.5; // todo: fix magic number from test setup
        double initialVolume = (4.0 * M_PI * initialRadius * initialRadius * initialRadius) / 3.0;
        double targetVolume = initialVolume * sizeFactor;
        double targetRadius = std::cbrt((3.0 * targetVolume) / (4.0 * M_PI));

        pCell->GetCellData()->SetItem("Radius", targetRadius);
    }

    rCellPopulation.Update(); // Update node radii from cell radii
}

template<unsigned DIM>
void GrowthModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class GrowthModifier<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(GrowthModifier)
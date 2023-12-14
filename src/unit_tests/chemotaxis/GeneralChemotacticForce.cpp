/*

Copyright (c) 2005-2023, University of Oxford.
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

#include "GeneralChemotacticForce.hpp"
#include "CellwiseDataGradient.hpp"
#include "CellLabel.hpp"

template<unsigned DIM>
GeneralChemotacticForce<DIM>::GeneralChemotacticForce()
    : AbstractForce<DIM>(),
      mStrengthParameter(1.0),
      mVariableName("variable"),
      mMoveLabeledCells("true")
{
}

template<unsigned DIM>
GeneralChemotacticForce<DIM>::~GeneralChemotacticForce()
{
}


template<unsigned DIM>
double GeneralChemotacticForce<DIM>::GetStrengthParameter()
{
    return mStrengthParameter;
}

template<unsigned DIM>
void GeneralChemotacticForce<DIM>::SetStrengthParameter(double strengthParameter)
{
    mStrengthParameter = strengthParameter;
}

template<unsigned DIM>
std::string GeneralChemotacticForce<DIM>::GetVariableName()
{
    return mVariableName;
}

template<unsigned DIM>
void GeneralChemotacticForce<DIM>::SetVariableName(std::string variableName)
{
    mVariableName = variableName;
}

template<unsigned DIM>
bool GeneralChemotacticForce<DIM>::GetMoveLabeledCells() const
{
    return mMoveLabeledCells;
}

template<unsigned DIM>
void GeneralChemotacticForce<DIM>::SetMoveLabeledCells(bool moveLabeledCells)
{
    mMoveLabeledCells = moveLabeledCells;
}

template<unsigned DIM>
double GeneralChemotacticForce<DIM>::GetGeneralChemotacticForceMagnitude(const double concentration, const double concentrationGradientMagnitude) const
{
    return mStrengthParameter; // temporary force law - can be changed to something realistic
}

template <unsigned DIM>
void GeneralChemotacticForce<DIM>::AddForceContribution(AbstractCellPopulation<DIM>& rCellPopulation)
{

    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
         cell_iter != rCellPopulation.End();
         ++cell_iter)
    {
        // We either move labeled cells or non-labeled cells; if the truth values are different, there is no applied force
        // due to chemotaxis.
        if (mMoveLabeledCells != cell_iter->template HasCellProperty<CellLabel>())
        {
            continue;
        }

        const unsigned node_global_index = rCellPopulation.GetLocationIndexUsingCell(*cell_iter);

        c_vector<double, DIM> gradient = zero_vector<double>(DIM);

        if constexpr (DIM == 2)
        {
            gradient(0) = cell_iter->GetCellData()->GetItem(mVariableName + "_grad_x");
            gradient(1) = cell_iter->GetCellData()->GetItem(mVariableName + "_grad_y");
        }
        else if constexpr (DIM == 3)
        {
            gradient(0) = cell_iter->GetCellData()->GetItem(mVariableName + "_grad_x");
            gradient(1) = cell_iter->GetCellData()->GetItem(mVariableName + "_grad_y");
            gradient(2) = cell_iter->GetCellData()->GetItem(mVariableName + "_grad_z");
        }
        else
        {
            NEVER_REACHED;
        }

        const double nutrient_concentration = cell_iter->GetCellData()->GetItem(mVariableName);
        const double magnitude_of_gradient = norm_2(gradient);

        const double force_magnitude = GetGeneralChemotacticForceMagnitude(nutrient_concentration, magnitude_of_gradient);

        // force += chi * gradC/|gradC|
        if (magnitude_of_gradient > 0)
        {
            c_vector<double, DIM> force = force_magnitude * gradient / magnitude_of_gradient;
            rCellPopulation.GetNode(node_global_index)->AddAppliedForceContribution(force);
        }
    }
}

template<unsigned DIM>
void GeneralChemotacticForce<DIM>::OutputForceParameters(out_stream& rParamsFile)
{
    // No parameters to include

    // Call method on direct parent class
    AbstractForce<DIM>::OutputForceParameters(rParamsFile);
}

// Explicit instantiation
template class GeneralChemotacticForce<1>;
template class GeneralChemotacticForce<2>;
template class GeneralChemotacticForce<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(GeneralChemotacticForce)

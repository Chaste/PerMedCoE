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

#include <boost/shared_ptr.hpp>

#include "AbstractCellPopulation.hpp"
#include "ConstantVelocityForce.hpp"

template <unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void ConstantVelocityForce<ELEMENT_DIM, SPACE_DIM>::AddForceContribution(AbstractCellPopulation<ELEMENT_DIM, SPACE_DIM>& rCellPopulation)

{
    for (auto pCell = rCellPopulation.Begin(); pCell != rCellPopulation.End(); ++pCell)
    {
        // Retrieve the force's x, y, z components from cell data
        c_vector<double, SPACE_DIM> force;
        boost::shared_ptr<CellData> cellData = pCell->GetCellData();

        if (SPACE_DIM >= 1)
        {
            force[0] = cellData->HasItem("x_velocity") ? cellData->GetItem("x_velocity") : 0.0;
        }
        if (SPACE_DIM >= 2)
        {
            force[1] = cellData->HasItem("y_velocity") ? cellData->GetItem("y_velocity") : 0.0;
        }
        if (SPACE_DIM == 3)
        {
            force[2] = cellData->HasItem("z_velocity") ? cellData->GetItem("z_velocity") : 0.0;
        }

        // Apply the force
        unsigned int i = rCellPopulation.GetLocationIndexUsingCell(*pCell);
        rCellPopulation.GetNode(i)->AddAppliedForceContribution(force);
    }
}

// Explicit instantiation
template class ConstantVelocityForce<1>;
template class ConstantVelocityForce<2>;
template class ConstantVelocityForce<3>;

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_ALL_DIMS(ConstantVelocityForce)

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

#include "TissueWidthWriter.hpp"
#include "AbstractCellPopulation.hpp"
#include "MeshBasedCellPopulation.hpp"
#include "CaBasedCellPopulation.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "PottsBasedCellPopulation.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "ImmersedBoundaryCellPopulation.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
TissueWidthWriter<ELEMENT_DIM, SPACE_DIM>::TissueWidthWriter()
    : AbstractCellPopulationWriter<ELEMENT_DIM, SPACE_DIM>("tissuewidth.dat")
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void TissueWidthWriter<ELEMENT_DIM, SPACE_DIM>::VisitAnyPopulation(AbstractCellPopulation<SPACE_DIM, SPACE_DIM>* pCellPopulation)
{
    double max_width =-2000000;
    double min_width = 2000000;
    for (typename AbstractMesh<SPACE_DIM, SPACE_DIM>::NodeIterator node_iter = pCellPopulation->rGetMesh().GetNodeIteratorBegin();
         node_iter != pCellPopulation->rGetMesh().GetNodeIteratorEnd();
         ++node_iter)
    {
        if (!node_iter->IsDeleted())
        {
            const c_vector<double,SPACE_DIM>& position = node_iter->rGetLocation();
            if(position[0] > max_width){
                max_width = position[0];
            }
            if(position[0] < min_width){
                min_width =position[0];
            }
        }
    }
    double tissue_width = (max_width - min_width)*10 + 5; // outputs tissue diameter in um and the number of cells
    unsigned int num_cells = pCellPopulation->GetNumRealCells();
    *this->mpOutStream << min_width << "," << max_width << "," << tissue_width << "," << num_cells << "\n";
    std::cout << "Time: " << SimulationTime::Instance()->GetTime() << "    Tissue diameter: " << tissue_width << "    Number of Cells: " << num_cells << "\n";
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void TissueWidthWriter<ELEMENT_DIM, SPACE_DIM>::Visit(MeshBasedCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
    for (typename AbstractMesh<ELEMENT_DIM, SPACE_DIM>::NodeIterator node_iter = pCellPopulation->rGetMesh().GetNodeIteratorBegin();
            node_iter != pCellPopulation->rGetMesh().GetNodeIteratorEnd();
            ++node_iter)
    {
        if (!node_iter->IsDeleted())
        {
            const c_vector<double,SPACE_DIM>& position = node_iter->rGetLocation();

            for (unsigned i=0; i<SPACE_DIM; i++)
            {
                *this->mpOutStream << position[i] << " ";
            }
        }
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void TissueWidthWriter<ELEMENT_DIM, SPACE_DIM>::Visit(CaBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    VisitAnyPopulation(pCellPopulation);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void TissueWidthWriter<ELEMENT_DIM, SPACE_DIM>::Visit(NodeBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    VisitAnyPopulation(pCellPopulation);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void TissueWidthWriter<ELEMENT_DIM, SPACE_DIM>::Visit(PottsBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    VisitAnyPopulation(pCellPopulation);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void TissueWidthWriter<ELEMENT_DIM, SPACE_DIM>::Visit(VertexBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    VisitAnyPopulation(pCellPopulation);
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void TissueWidthWriter<ELEMENT_DIM, SPACE_DIM>::Visit(ImmersedBoundaryCellPopulation<SPACE_DIM>* pCellPopulation)
{
    VisitAnyPopulation(pCellPopulation);
}

// Explicit instantiation
template class TissueWidthWriter<1,1>;
template class TissueWidthWriter<1,2>;
template class TissueWidthWriter<2,2>;
template class TissueWidthWriter<1,3>;
template class TissueWidthWriter<2,3>;
template class TissueWidthWriter<3,3>;

#include "SerializationExportWrapperForCpp.hpp"
// Declare identifier for the serializer
EXPORT_TEMPLATE_CLASS_ALL_DIMS(TissueWidthWriter)

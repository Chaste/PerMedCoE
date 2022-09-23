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

#ifndef TESTSTOCHASTICDURATIONCELLCYCLE_HPP_
#define TESTSTOCHASTICDURATIONCELLCYCLE_HPP_

#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedTestSuite.hpp"

#include "CellsGenerator.hpp"
#include "WildTypeCellMutationState.hpp"
#include "TransitCellProliferativeType.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "OffLatticeSimulation.hpp"
#include "SmartPointers.hpp"
#include "NodesOnlyMesh.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "SphereGeometryBoundaryCondition.hpp"

#include "StochasticDurationCellCycleModel.hpp"

// Chaste uses PETSc to solve linear algebra problems.
// PETSc must be started & closed at the start & end of tests.
// This code will never run in parallel so we use FakePetscSetup.hpp.
#include "FakePetscSetup.hpp"

class TestStochasticDurationCellCycle : public AbstractCellBasedTestSuite
{
public:
    void TestNodeBasedStochasticDurationCellCycle()
    {
        // We cannot currently run node based simulations in parallel.
        EXIT_IF_PARALLEL;

        TS_ASSERT_THROWS_NOTHING(StochasticDurationCellCycleModel model);

        // Create 3D mesh with a single node
        // Length is dimensionless; typical cell diameter is approx 10 um
        std::vector<Node<3>*> nodes;
        nodes.push_back(new Node<3>(0u,  false,  0.0, 0.0, 0.1));
        NodesOnlyMesh<3> mesh;
        mesh.ConstructNodesWithoutMesh(nodes, 1.5); // neighbour interaction radius = 1.5

        // Create cell collection
        std::vector<CellPtr> cells;
        MAKE_PTR(WildTypeCellMutationState, p_state); 
        MAKE_PTR(TransitCellProliferativeType, p_transit_type);
        CellsGenerator<StochasticDurationCellCycleModel, 3> cells_generator; 
        cells_generator.GenerateBasicRandom(cells, mesh.GetNumNodes(), p_transit_type);

        // Create 3D population object to connect mesh and cells
        NodeBasedCellPopulation<3> cell_population(mesh, cells);

        // Create an OffLatticeSimulation with the population
        OffLatticeSimulation<3> simulator(cell_population);

        // Set some simulation options
        simulator.SetOutputDirectory("StochasticDurationCellCycle");
        simulator.SetSamplingTimestepMultiple(10);
        simulator.SetEndTime(48.0); // 48 hours
        simulator.SetDt(0.1); // 6 mins

        // Define boundary sphere
        c_vector<double,3> centre = zero_vector<double>(3);
        double radius = 6.0; // 60 um
        MAKE_PTR_ARGS(SphereGeometryBoundaryCondition<3>, p_boundary_condition, (&cell_population, centre, radius));
        simulator.AddCellPopulationBoundaryCondition(p_boundary_condition);

        // Run the simulation
        simulator.Solve();
        
        // Memory management
        for (unsigned i=0; i<nodes.size(); i++)
        {
            delete nodes[i];
        }
    }
};

#endif /*TESTSTOCHASTICDURATIONCELLCYCLE_HPP_*/

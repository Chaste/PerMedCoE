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

#ifndef TESTFIXEDDURATIONCELLCYCLE_HPP_
#define TESTFIXEDDURATIONCELLCYCLE_HPP_

#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedTestSuite.hpp"
#include "SmartPointers.hpp"

#include "CellsGenerator.hpp"
#include "GeneralisedLinearSpringForce.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "NodesOnlyMesh.hpp"
#include "OffLatticeSimulation.hpp"
#include "SphereGeometryBoundaryCondition.hpp"
#include "TransitCellProliferativeType.hpp"

#include "CellIdWriter.hpp"
#include "CellLabelWriter.hpp"
#include "CellVolumesWriter.hpp"

// PETSc must be initialized to solve linear algebra problems in Chaste.
// For sequential code, FakePetscSetup.hpp starts PETSc on a single rank.
#include "FakePetscSetup.hpp"

#include "FixedDurationCellCycleModel.hpp"

class TestFixedDurationCellCycle : public AbstractCellBasedTestSuite
{
public:
    void TestNodeBasedFixedDurationCellCycle()
    {
        // Cannot currently run cell-based simulations in parallel.
        EXIT_IF_PARALLEL;

        TS_ASSERT_THROWS_NOTHING(FixedDurationCellCycleModel model);

        // Create 3D NodesOnlyMesh with a single node
        std::vector<Node<3>*> nodes;
        nodes.push_back(new Node<3>(0u,  false,  0.0, 0.0, 0.1));
        NodesOnlyMesh<3> mesh;

        // Length is dimensionless and based on typical cell diameter i.e. approx 10 um
        double max_interaction_radius = 1.5; // 15 um neighbour interaction distance
        mesh.ConstructNodesWithoutMesh(nodes, max_interaction_radius);

        // Create cell collection
        std::vector<CellPtr> cells;
        MAKE_PTR(TransitCellProliferativeType, p_transit_type);
        CellsGenerator<FixedDurationCellCycleModel, 3> cells_generator; 
        cells_generator.GenerateBasicRandom(cells, mesh.GetNumNodes(), p_transit_type);

        // Create 3D cell population object to connect mesh and cell
        NodeBasedCellPopulation<3> cell_population(mesh, cells);

        // Add output writers
        cell_population.AddCellWriter<CellIdWriter>();
        cell_population.AddCellWriter<CellLabelWriter>();
        cell_population.AddCellWriter<CellVolumesWriter>();

        // Create an OffLatticeSimulation with the population
        OffLatticeSimulation<3> simulator(cell_population);

        // Set some simulation options
        simulator.SetOutputDirectory("FixedDurationCellCycle");
        simulator.SetEndTime(48.0); // 48 hours
        simulator.SetDt(1.0 / 60.0); // 1 min
        simulator.SetSamplingTimestepMultiple(6); // 6 mins

        // Add force for cell movement
        MAKE_PTR(GeneralisedLinearSpringForce<3>, p_force);
        simulator.AddForce(p_force);

        // Define boundary sphere
        c_vector<double,3> centre = zero_vector<double>(3);
        double radius = 3.0; // 60 um diameter
        MAKE_PTR_ARGS(SphereGeometryBoundaryCondition<3>, p_boundary_condition, (&cell_population, centre, radius));
        simulator.AddCellPopulationBoundaryCondition(p_boundary_condition);

        // Run the simulation
        cell_population.Update(false); // Needed for CellVolumesWriter, else throws
        simulator.Solve();
        
        // Memory management
        for (unsigned i=0; i<nodes.size(); i++)
        {
            delete nodes[i];
        }
    }
};

#endif /*TESTFIXEDDURATIONCELLCYCLE_HPP_*/

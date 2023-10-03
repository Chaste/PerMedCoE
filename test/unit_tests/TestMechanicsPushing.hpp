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

#ifndef TESTMECHANICSPUSHING_HPP_
#define TESTMECHANICSPUSHING_HPP_

#include <cxxtest/TestSuite.h>

#include "AbstractCellBasedTestSuite.hpp"
#include "Cell.hpp"
#include "CellsGenerator.hpp"
#include "DifferentiatedCellProliferativeType.hpp"
#include "FixedG1GenerationalCellCycleModel.hpp"
#include "GeneralisedLinearSpringForce.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "NodesOnlyMesh.hpp"
#include "OffLatticeSimulation.hpp"
#include "SmartPointers.hpp"

// PETSc must be initialized to solve linear algebra problems in Chaste.
// For sequential code, FakePetscSetup.hpp starts PETSc on a single rank.
#include "FakePetscSetup.hpp"

class TestMechanicsPushing : public AbstractCellBasedTestSuite
{
public:
    void TestNodeBasedMechanicsPushingTwoCells()
    {
        // Cannot currently run cell-based simulations in parallel.
        EXIT_IF_PARALLEL;

        // Create two 1D nodes
        std::vector<Node<1>*> nodes;
        std::vector<double> coords{ 0.0 };
        nodes.push_back(new Node<1>(0u, coords, false));
        coords[0] = 10.0;
        nodes.push_back(new Node<1>(1u, coords, false));

        // Create a 1D NodesOnlyMesh
        NodesOnlyMesh<1> mesh;

        // Length is dimensionless and based on typical cell diameter i.e. approx 10 um
        double max_interaction_radius = 1.5; // 15 um neighbour interaction distance
        mesh.ConstructNodesWithoutMesh(nodes, max_interaction_radius);

        // Create cells
        std::vector<CellPtr> cells;
        MAKE_PTR(DifferentiatedCellProliferativeType, p_diff_type);
        CellsGenerator<FixedG1GenerationalCellCycleModel, 1> cells_generator;
        cells_generator.GenerateBasic(cells, 2, std::vector<unsigned>(), p_diff_type);

        // Create cell population to connect mesh and cells
        NodeBasedCellPopulation<1> cell_population(mesh, cells);

        // Create an OffLatticeSimulation with the population
        OffLatticeSimulation<1> simulator(cell_population);

        // Set some simulation options
        simulator.SetOutputDirectory("MechanicsPushing");
        simulator.SetEndTime(1.0 / 6.0); // 10 min
        simulator.SetDt(1.0 / 60.0); // 0.1 min
        simulator.SetSamplingTimestepMultiple(1); // 1 min

        // Add force for cell movement
        MAKE_PTR(GeneralisedLinearSpringForce<1>, p_force);
        simulator.AddForce(p_force);

        // Run the simulation
        simulator.Solve();

        // Memory management
        for (unsigned i = 0; i < nodes.size(); i++)
        {
            delete nodes[i];
        }
    }
};

#endif // TESTMECHANICSPUSHING_HPP_

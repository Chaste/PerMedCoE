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

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include "AbstractCellBasedTestSuite.hpp"
#include "CellsGenerator.hpp"
#include "ConstantVelocityForce.hpp"
#include "GeneralisedLinearSpringForce.hpp"
#include "NoCellCycleModel.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "NodeLocationWriter.hpp"
#include "NodesOnlyMesh.hpp"
#include "OffLatticeSimulation.hpp"

// PETSc must be initialized to solve linear algebra problems in Chaste.
// For sequential code, FakePetscSetup.hpp starts PETSc on a single rank.
#include "FakePetscSetup.hpp"

class TestMechanicsPushing : public AbstractCellBasedTestSuite
{
public:
    void TestNodeBasedMechanicsPushingTwoCells()
    {
        // Description: two cells moving towards each other's centres at a velocity of 10 um/min

        // Cannot currently run cell-based simulations in parallel.
        EXIT_IF_PARALLEL;

        // Create two 1D nodes
        // Length is dimensionless and based on typical cell diameter i.e. approx 10 um
        // Cell centres are initially 30 um apart
        auto p_node_0 = std::make_unique<Node<1> >(0u, std::vector<double>(1, 0.0), false);
        auto p_node_1 = std::make_unique<Node<1> >(1u, std::vector<double>(1, 30.0), false);
        std::vector<Node<1>*> nodes = {p_node_0.get(), p_node_1.get()};

        // Create a 1D nodes-only mesh with 15 um neighbour interaction distance.
        double max_interaction_radius = 1.5;
        NodesOnlyMesh<1> mesh;
        mesh.ConstructNodesWithoutMesh(nodes, max_interaction_radius);

        // Create the cells. These cells do not grow.
        std::vector<CellPtr> cells;
        CellsGenerator<NoCellCycleModel, 1> cells_generator;
        cells_generator.GenerateBasic(cells, mesh.GetNumNodes());

        // Create a cell population to connect the mesh and cells
        NodeBasedCellPopulation<1> cell_population(mesh, cells);

        // Add a node location writer to the cell population
        auto p_writer = boost::make_shared<NodeLocationWriter<1, 1> >();
        cell_population.AddPopulationWriter(p_writer);
        p_writer->SetFileName("results.viznodelocations");

        // Create an off-lattice simulation with the cell population
        OffLatticeSimulation<1> simulator(cell_population);

        // Set some simulation options
        simulator.SetOutputDirectory("TestMechanicsPushing");
        simulator.SetEndTime(1.0 / 6.0); // 10 min
        simulator.SetDt(1.0 / 600.0); // 0.1 min
        simulator.SetSamplingTimestepMultiple(1); // 0.1 min

        // Add a constant 10 um velocity force
        auto p_constant_force = boost::make_shared<ConstantVelocityForce<1> >(600.0);
        simulator.AddForce(p_constant_force);

        // Add a generalised linear spring force
        auto p_spring_force = boost::make_shared<GeneralisedLinearSpringForce<1> >();
        simulator.AddForce(p_spring_force);

        // Run the simulation
        simulator.Solve();
    }
};

#endif // TESTMECHANICSPUSHING_HPP_

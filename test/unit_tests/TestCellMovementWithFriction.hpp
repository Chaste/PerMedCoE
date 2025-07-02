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

#ifndef TESTHELLO_HPP_
#define TESTHELLO_HPP_

#include <cxxtest/TestSuite.h>
/* Most Chaste code uses PETSc to solve linear algebra problems.  This involves starting PETSc at the beginning of a test-suite
 * and closing it at the end.  (If you never run code in parallel then it is safe to replace PetscSetupAndFinalize.hpp with FakePetscSetup.hpp)
 */
#include "CheckpointArchiveTypes.hpp"
#include "SmartPointers.hpp"
#include "VoronoiDataWriter.hpp"
#include "NodeLocationWriter.hpp"
#include "AbstractCellBasedTestSuite.hpp"
#include "PetscSetupAndFinalize.hpp"
#include "CellsGenerator.hpp"
#include "NoCellCycleModel.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "OffLatticeSimulation.hpp"
#include "TransitCellProliferativeType.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "RepulsionForce.hpp"
#include "FakePetscSetup.hpp"

#include "InstantaneousVelocityForce.hpp"



class TestCellMechanics : public AbstractCellBasedTestSuite
{
public:
/*
 * 1 unit in Chaste is ~ 1 cell width/10um
 * The cell should move with an initial velocity of 10um/minute, i.e. 1 cell width/minute
 * Timestep is 0.1 minutes, i.e. 10 steps/cell width
 * Total time = 10mins, i.e. 10 cell widths of movement without friction
*/
    void TestCellMechanicsCellMovement() {

      // Generate a mesh
      HoneycombMeshGenerator generator(1, 1);
      MutableMesh<2,2>* p_generating_mesh = generator.GetMesh().get();
      NodesOnlyMesh<2> mesh;
      mesh.ConstructNodesWithoutMesh(*p_generating_mesh, 1.5);

      // Add the cell 
      std::vector<CellPtr> cells;
      CellsGenerator<NoCellCycleModel, 2> cells_generator;
      cells_generator.GenerateBasicRandom(cells, mesh.GetNumNodes());

      // Set initial conditions
      // Expressed in units of simulation units/hour
      // Assuming 1 unit = 10um
      c_vector<double, 2> initialVelocity;
      initialVelocity[0] = 600.0;
      initialVelocity[1] = 0.0;

      MAKE_PTR(InstantaneousVelocityForce<2>, force);
      force->SetVelocity(initialVelocity);

      // Setup cell population
      NodeBasedCellPopulation<2> cell_population(mesh, cells);
      cell_population.Update();

      // Add node location writer
      using LocationWriter = NodeLocationWriter<2, 2>;
      MAKE_PTR(LocationWriter, writer);
      writer->SetFileName("node_locations.dat");
      cell_population.AddPopulationWriter(writer);

      // Set up simulator
      OffLatticeSimulation<2> simulator(cell_population);
      simulator.SetOutputDirectory("PerMedCoE");
      simulator.SetEndTime(1.0 / 6.0); // 10 mins expressed in hours
      simulator.SetDt(1.0 / (60.0 * 10.0)); // 0.1 mins expressed in hours
      simulator.AddForce(force);

      // Perform simulation
      simulator.Solve();

      // Write the results
      std::string output_directory = "PerMedCoE";
      OutputFileHandler output_file_handler(output_directory, false);
      cell_population.OpenWritersFiles(output_file_handler);
      cell_population.WriteResultsToFiles(output_directory);
    }
};

#endif 

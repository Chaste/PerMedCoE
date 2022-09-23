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
#include "VertexBasedCellPopulation.hpp"
#include "OffLatticeSimulation.hpp"
#include "TransitCellProliferativeType.hpp"
#include "HoneycombVertexMeshGenerator.hpp"
#include "RepulsionForce.hpp"
#include "FakePetscSetup.hpp"

#include "DragForce.hpp"
#include "ExperiencesDrag.hpp"

/**
 * @file
 *
 * This is an example of a CxxTest test suite, used to test the source
 * code, and also used to run simulations (as it provides a handy
 * shortcut to compile and link against the correct libraries using scons).
 *
 * You can #include any of the files in the project 'src' folder.
 * For example here we #include "Hello.hpp"
 *
 * You can utilise any of the code in the main the Chaste trunk
 * in exactly the same way.
 * NOTE: you will have to alter the project SConscript file lines 41-44
 * to enable #including of code from the 'heart', 'cell_based' or 'crypt'
 * components of Chaste.
 */

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
      HoneycombVertexMeshGenerator generator(1, 1);
      MutableVertexMesh<2,2>* p_mesh = generator.GetMesh();

      // Add the cell 
      std::vector<CellPtr> cells;
      MAKE_PTR(TransitCellProliferativeType, p_transit_type);
      MAKE_PTR(WildTypeCellMutationState, p_state); 
      MAKE_PTR(ExperiencesDrag, p_drag);
      p_drag->coefficient = 0.001;

      for (unsigned int i = 0; i < p_mesh->GetNumElements(); i++) {
        auto cell_cycle_model = new NoCellCycleModel();
        CellPropertyCollection collection;
        collection.AddProperty(p_drag);
        CellPtr cell(new Cell(p_state, cell_cycle_model, nullptr, false, collection));
        cell->SetCellProliferativeType(p_transit_type);
        cell->SetBirthTime(0);
        cells.push_back(cell);
      }

      // Set initial conditions
      c_vector<double, 2> initialVelocity;
      initialVelocity[0] = 1.0;
      initialVelocity[1] = 0.0;

      MAKE_PTR(DragForce<2>, force);
      force->SetVelocity(initialVelocity);

      // Setup cell population
      VertexBasedCellPopulation<2> cell_population(*p_mesh, cells);

      // Set up simulator
      OffLatticeSimulation<2> simulator(cell_population);
      simulator.SetOutputDirectory("CellBasedDemo1");
      simulator.SetEndTime(1.0 / 6.0); // 10 mins
      simulator.SetDt(1.0 / (60.0 * 10.0)); // 0.1 mins
      simulator.AddForce(force);

      // Perform simulation
      simulator.Solve();

    }
};

#endif /*TESTHELLO_HPP_*/

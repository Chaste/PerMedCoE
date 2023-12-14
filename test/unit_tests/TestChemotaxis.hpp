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

#ifndef _TEST_PETMEDCOE_CHEMOTAXIS_HPP_
#define _TEST_PETMEDCOE_CHEMOTAXIS_HPP_

#include <cxxtest/TestSuite.h>

#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "ApoptoticCellProperty.hpp"
#include "AveragedSourceParabolicPde.hpp"
#include "CellData.hpp"
#include "CellLabel.hpp"
#include "CellsGenerator.hpp"
#include "ConstBoundaryCondition.hpp"
#include "DifferentiatedCellProliferativeType.hpp"
#include "GeneralChemotacticForce.hpp"
#include "HoneycombMeshGenerator.hpp"
#include "OffLatticeSimulation.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "NodesOnlyMesh.hpp"
#include "ParabolicBoxDomainPdeModifier.hpp"
#include "UniformCellCycleModel.hpp"

#include "DiffusionEquationWithSinkTerms.hpp"

/* The following header must be included in every test that uses PETSc. Note that it
 * cannot be included in the source code. */
#include "PetscSetupAndFinalize.hpp"

/**
 *
 */
class TestChemotaxis : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestOneCellWithChemotaxis()
    {
        // Create mesh
        std::vector<Node<3>*> nodes;
        nodes.push_back(new Node<3>(0u,  false, 0.0, 0.0, 0.0));

        NodesOnlyMesh<3> mesh;
        mesh.ConstructNodesWithoutMesh(nodes, 1.5);

        // Create cells
        std::vector<CellPtr> cells;
        MAKE_PTR(DifferentiatedCellProliferativeType, p_diff_type);
        CellsGenerator<UniformCellCycleModel, 2> cells_generator;
        cells_generator.GenerateBasicRandom(cells, mesh.GetNumNodes(), p_diff_type);

        // Create cell population
        NodeBasedCellPopulation<3> cell_population(mesh, cells);

        boost::shared_ptr<AbstractCellProperty> p_label(CellPropertyRegistry::Instance()->Get<CellLabel>());
        for (AbstractCellPopulation<3>::Iterator cell_iter = cell_population.Begin();
             cell_iter != cell_population.End();
             ++cell_iter)
        {
            cell_iter->GetCellData()->SetItem("variable",0.0);
            cell_iter->GetCellData()->SetItem("variable_grad_x",0.0);
            cell_iter->GetCellData()->SetItem("variable_grad_y",0.0);
            cell_iter->GetCellData()->SetItem("variable_grad_z",0.0);
        }

        // Create crypt simulation from cell population
        OffLatticeSimulation<3> simulator(cell_population);
        simulator.SetOutputDirectory("TestChemotaxisPde");
        simulator.SetSamplingTimestepMultiple(10);
        simulator.SetEndTime(0.02);
        simulator.SetDt(0.01);

        // Create PDE and boundary condition objects
        MAKE_PTR(DiffusionEquationWithSinkTerms<3>, p_pde);
        p_pde->AddSink(Create_c_vector(1.2, 2.3, 3.4), -1.0, 0.05);
        MAKE_PTR_ARGS(ConstBoundaryCondition<3>, p_bc, (1.0));

        // Create a ChasteCuboid on which to base the finite element mesh used to solve the PDE
        ChastePoint<3> lower(-5.0, -5.0, -5.0);
        ChastePoint<3> upper(15.0, 15.0, 15.0);
        MAKE_PTR_ARGS(ChasteCuboid<3>, p_cuboid, (lower, upper));

        // Create a PDE modifier and set the name of the dependent variable in the PDE
        MAKE_PTR_ARGS(ParabolicBoxDomainPdeModifier<3>, p_pde_modifier, (p_pde, p_bc, false, p_cuboid));
        p_pde_modifier->SetOutputGradient(true);
        p_pde_modifier->SetDependentVariableName("variable");

        simulator.AddSimulationModifier(p_pde_modifier);

        MAKE_PTR(GeneralChemotacticForce<3>, p_chemotactic_force);
        p_chemotactic_force->SetVariableName("variable");
        p_chemotactic_force->SetMoveLabeledCells(false);
        p_chemotactic_force->SetStrengthParameter(1.0);
        simulator.AddForce(p_chemotactic_force);

        simulator.Solve();
    }
};

#endif /*_TEST_PETMEDCOE_CHEMOTAXIS_HPP_*/

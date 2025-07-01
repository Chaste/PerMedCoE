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

#ifndef _TESTDIFFUSIONEQUATIONWITHSINKTERM_HPP_
#define _TESTDIFFUSIONEQUATIONWITHSINKTERM_HPP_

#include <cxxtest/TestSuite.h>

#include "DiffusionEquationWithSinkTerm.hpp"

class TestDiffusionEquationWithSinkTerm : public CxxTest::TestSuite
{
public:
    void TestGettersAndSetters()
    {
        DiffusionEquationWithSinkTerm<3> pde;

        const c_vector<double, 3> sink_loc = Create_c_vector(1.23, 2.34, 3.45);
        pde.setDiffusionCoefficient(4.56);
        pde.setSinkRadius(5.67);
        pde.setUptake(6.78);
        pde.setSinkLocation(sink_loc);

        TS_ASSERT_DELTA(pde.getDiffusionCoefficient(), 4.56, 1e-12);
        TS_ASSERT_DELTA(pde.getSinkRadius(), 5.67, 1e-12);
        TS_ASSERT_DELTA(pde.getUptake(), 6.78, 1e-12);
        TS_ASSERT_DELTA(pde.getSinkLocation()[0], 1.23, 1e-12);
        TS_ASSERT_DELTA(pde.getSinkLocation()[1], 2.34, 1e-12);
        TS_ASSERT_DELTA(pde.getSinkLocation()[2], 3.45, 1e-12);
    }

    void TestComputeSourceTerm()
    {
        DiffusionEquationWithSinkTerm<3> pde;

        const c_vector<double, 3> sink_loc = Create_c_vector(10.0, 10.0, 10.0);
        const double uptake = 1.23;

        pde.setSinkRadius(10.0);
        pde.setUptake(uptake);
        pde.setSinkLocation(sink_loc);

        // Everything in the cuboid [0, 20]^3 should return -uptake; else, 0.0

        // Inside or boundary
        {
            std::vector<ChastePoint<3> > inside_or_boundary;
            inside_or_boundary.emplace_back(10.0, 10.0, 10.0);
            inside_or_boundary.emplace_back(0.0, 0.0, 0.0);
            inside_or_boundary.emplace_back(20.0, 0.0, 0.0);
            inside_or_boundary.emplace_back(0.0, 20.0, 0.0);
            inside_or_boundary.emplace_back(0.0, 0.0, 20.0);
            inside_or_boundary.emplace_back(20.0, 20.0, 0.0);
            inside_or_boundary.emplace_back(20.0, 0.0, 20.0);
            inside_or_boundary.emplace_back(0.0, 20.0, 20.0);
            inside_or_boundary.emplace_back(20.0, 20.0, 20.0);

            for (const auto& point : inside_or_boundary)
            {
                const double source = pde.ComputeSourceTerm(point, DOUBLE_UNSET, nullptr);
                TS_ASSERT_DELTA(source, -uptake, 1e-12);
            }
        }

        // Outside
        {
            std::vector<ChastePoint<3> > outside;
            outside.emplace_back(0.0, 0.0, -0.01);
            outside.emplace_back(20.01, 0.0, 0.0);
            outside.emplace_back(0.0, 20.01, 0.0);
            outside.emplace_back(0.0, 0.0, 20.01);
            outside.emplace_back(20.01, 20.01, 0.0);
            outside.emplace_back(20.01, 0.0, 20.01);
            outside.emplace_back(0.0, 20.01, 20.01);
            outside.emplace_back(20.01, 20.01, 20.01);

            for (const auto& point : outside)
            {
                const double source = pde.ComputeSourceTerm(point, DOUBLE_UNSET, nullptr);
                TS_ASSERT_DELTA(source, 0.0, 1e-12);
            }
        }
    }
};

#endif //_TESTDIFFUSIONEQUATIONWITHSINKTERM_HPP_

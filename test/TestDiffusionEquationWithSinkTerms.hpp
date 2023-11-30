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

#ifndef _TESTDIFFUSIONEQUATIONWITHSINKTERMS_HPP_
#define _TESTDIFFUSIONEQUATIONWITHSINKTERMS_HPP_

#include <cxxtest/TestSuite.h>

#include "DiffusionEquationWithSinkTerms.hpp"

class TestDiffusionEquationWithSinkTerm : public CxxTest::TestSuite
{
public:
    void TestGettersAndSetters()
    {
        DiffusionEquationWithSinkTerms<3> pde;
        pde.AddSink(Create_c_vector(0.12, 1.23, 2.34), 3.45, 4.56);
        pde.AddSink(Create_c_vector(5.67, 6.78, 7.89), 8.90, 9.01);

        const auto& locations = pde.rGetSinkLocations();
        const auto& strengths = pde.rGetSinkStrengths();
        const auto& radii = pde.rGetSinkRadii();

        TS_ASSERT_EQUALS(locations.size(), 2ul);
        TS_ASSERT_EQUALS(strengths.size(), 2ul);
        TS_ASSERT_EQUALS(radii.size(), 2ul);

        TS_ASSERT_DELTA(locations[0][0], 0.12, 1e-12);
        TS_ASSERT_DELTA(locations[0][1], 1.23, 1e-12);
        TS_ASSERT_DELTA(locations[0][2], 2.34, 1e-12);

        TS_ASSERT_DELTA(locations[1][0], 5.67, 1e-12);
        TS_ASSERT_DELTA(locations[1][1], 6.78, 1e-12);
        TS_ASSERT_DELTA(locations[1][2], 7.89, 1e-12);

        TS_ASSERT_DELTA(strengths[0], 3.45, 1e-12);
        TS_ASSERT_DELTA(strengths[1], 8.90, 1e-12);

        TS_ASSERT_DELTA(radii[0], 4.56, 1e-12);
        TS_ASSERT_DELTA(radii[1], 9.01, 1e-12);
    }

    void TestComputeSourceTerm()
    {
        DiffusionEquationWithSinkTerms<3> pde;
        pde.AddSink(Create_c_vector(1.0, 0.0, 0.0), 1.2, 0.1);
        pde.AddSink(Create_c_vector(0.0, 1.0, 0.0), 2.3, 0.2);
        pde.AddSink(Create_c_vector(0.0, 0.0, 1.0), 3.4, 0.3);

        // Far away, should all be zero
        {
            std::vector<ChastePoint<3> > far_away;
            far_away.emplace_back(0.0, 0.0, 0.0);
            far_away.emplace_back(0.89, 0.0, 0.0);
            far_away.emplace_back(1.11, 0.0, 0.0);
            far_away.emplace_back(0.0, 0.79, 0.0);
            far_away.emplace_back(0.0, 1.21, 0.0);
            far_away.emplace_back(0.0, 0.0, 0.69);
            far_away.emplace_back(0.0, 0.0, 1.31);

            for (const auto& point : far_away)
            {
                const double source = pde.ComputeSourceTerm(point, DOUBLE_UNSET, nullptr);
                TS_ASSERT_DELTA(source, 0.0, 1e-12);
            }
        }

        // Near first source
        {
            std::vector<ChastePoint<3> > nearby;
            nearby.emplace_back(1.0, 0.0, 0.0);
            nearby.emplace_back(0.91, 0.0, 0.0);
            nearby.emplace_back(1.09, 0.0, 0.0);
            nearby.emplace_back(1.09, 0.09, 0.09);
            nearby.emplace_back(1.0, -0.09, 0.0);
            nearby.emplace_back(1.0, 0.09, 0.0);
            nearby.emplace_back(1.0, 0.0, -0.09);
            nearby.emplace_back(1.0, 0.0, 0.09);

            for (const auto& point : nearby)
            {
                const double source = pde.ComputeSourceTerm(point, DOUBLE_UNSET, nullptr);
                TS_ASSERT_DELTA(source, -1.2, 1e-12);
            }
        }

        // Near second source
        {
            std::vector<ChastePoint<3> > nearby;
            nearby.emplace_back(0.0, 1.0, 0.0);
            nearby.emplace_back(0.0, 0.81, 0.0);
            nearby.emplace_back(0.0, 1.19, 0.0);
            nearby.emplace_back(0.19, 1.19, 0.19);
            nearby.emplace_back(-0.19, 1.0, 0.0);
            nearby.emplace_back(0.19, 1.0, 0.0);
            nearby.emplace_back(0.0, 1.0, -0.19);
            nearby.emplace_back(0.0, 1.0, 0.19);

            for (const auto& point : nearby)
            {
                const double source = pde.ComputeSourceTerm(point, DOUBLE_UNSET, nullptr);
                TS_ASSERT_DELTA(source, -2.3, 1e-12);
            }
        }

        // Near third source
        {
            std::vector<ChastePoint<3> > nearby;
            nearby.emplace_back(0.0, 0.0, 1.0);
            nearby.emplace_back(0.0, 0.0, 0.71);
            nearby.emplace_back(0.0, 0.0, 1.29);
            nearby.emplace_back(0.29, 0.29, 1.29);
            nearby.emplace_back(-0.29, 0.0, 1.0);
            nearby.emplace_back(0.29, 0.0, 1.0);
            nearby.emplace_back(0.0, -0.29, 1.0);
            nearby.emplace_back(0.0, 0.29, 1.0);

            for (const auto& point : nearby)
            {
                const double source = pde.ComputeSourceTerm(point, DOUBLE_UNSET, nullptr);
                TS_ASSERT_DELTA(source, -3.4, 1e-12);
            }
        }
    }

};

#endif //_TESTDIFFUSIONEQUATIONWITHSINKTERMS_HPP_

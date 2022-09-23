#pragma once
#include "AbstractCellProperty.hpp"

// Should also include a max force for dynamic friction - currently unbounded.

class ExperiencesDrag : public AbstractCellProperty {
    private:
        
    public:
        ExperiencesDrag() = default;
        double coefficient = 0.01;
        double tolerance = 0.02;

};
#include "AbstractForce.hpp"
#include "SimulationTime.hpp"

template <unsigned int DIM>
class InstantaneousVelocityForce : public AbstractForce<DIM> {
    private:
        c_vector<double, DIM> velocity;

    public:
        // Default constructor
        InstantaneousVelocityForce() {
            for (unsigned int i = 0; i < DIM; i++) {
                velocity[i] = 0.0;
            }
        }
        
        // Constructor with velocity
        InstantaneousVelocityForce(const c_vector<double, DIM> velocity) : velocity(velocity) {};
    
        void SetVelocity(const c_vector<double, DIM> velocity) {
            for (unsigned int i = 0; i < DIM; i++) {
                this->velocity[i] = velocity[i];
            }
        }

        void AddForceContribution(AbstractCellPopulation<DIM>& cellPopulation) override {

            // Only apply the force if this is the first timestep
            const unsigned int stepsElapsed = SimulationTime::Instance()->GetTimeStepsElapsed();

            if (stepsElapsed == 0) {
                // Apply the force
                for (unsigned int i = 0; i < cellPopulation.GetNumNodes(); i++) {
                    cellPopulation.GetNode(i)->AddAppliedForceContribution(velocity);
                }
            }
        }
   
        void OutputForceParameters(out_stream& paramsFile) override {

        }
};
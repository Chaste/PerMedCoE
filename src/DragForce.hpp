#include "AbstractForce.hpp"
#include "ExperiencesDrag.hpp"

// The velocity should really be computed as cell_position - last_cell_position,
// but for this simple case where there are no other forces this works fine
template <unsigned int DIM>
class DragForce : public AbstractForce<DIM> {
    private:
        c_vector<double, DIM> velocity;

    public:
        // Sets initial velocity
        DragForce() {
            for (unsigned int i = 0; i < DIM; i++) {
                velocity[i] = 0.0;
            }
        }
        DragForce(const c_vector<double, DIM> velocity) : velocity(velocity) {};
    
        void SetVelocity(const c_vector<double, DIM> velocity) {
            for (unsigned int i = 0; i < DIM; i++) {
                this->velocity[i] = velocity[i];
            }
        }

        // This method actually adds a velocity rather than a force - as such we can cheat
        // and track the velocity on a CellProperty, modify it using the drag and then
        // return it + update the velocity on the CellProperty
        void AddForceContribution(AbstractCellPopulation<DIM>& cellPopulation) override {

            for (unsigned int i = 0; i < cellPopulation.GetNumNodes(); i++) {
                // Query the first cell
                typename AbstractCellPopulation<DIM>::Iterator cell_iter = cellPopulation.Begin();
                
                // Only apply this force to cells which experience friction 
                if(cell_iter->template HasCellProperty<ExperiencesDrag>()) {
                    // Grab the friction component
                    auto props = cell_iter->rGetCellPropertyCollection();
                    auto registry = props.GetCellPropertyRegistry();
                    boost::shared_ptr<ExperiencesDrag> frictionProp = boost::dynamic_pointer_cast<ExperiencesDrag>(registry->template Get<ExperiencesDrag>());

                    // Apply the drag force 
                    velocity[0] *= (1.0 - frictionProp->coefficient);
                    velocity[1] *= (1.0 - frictionProp->coefficient);

                    cellPopulation.GetNode(i)->AddAppliedForceContribution(velocity);
                } 
            }
        }
   
        void OutputForceParameters(out_stream& paramsFile) override {

        }
};
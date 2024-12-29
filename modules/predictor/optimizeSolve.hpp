#include <Log/log.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <cppad/cppad.hpp>
#include <eigen3/Eigen/Core>

namespace StateFitting
{
    using namespace aimlog;
    class OptimizeSolve
    {
    public:
        typedef CPPAD_TESTVECTOR( CppAD::AD<double> ) ADvector;
        typedef CPPAD_TESTVECTOR(double) Dvector;
        OptimizeSolve()
        {
            //关闭任何优化器的输出
            options += "Integer print_level  0\n";
            options += "String  sb           yes\n";
            //设置最大迭代次数
            options += "Integer max_iter     100\n";
            // 设置收敛条件
            // see Mathematical Programming, Volume 106, Number 1,
            // Pages 25-57, Equation (6)
            options += "Numeric tol          1e-10\n";
        }
        void setBoundInit(Eigen::Matrix3Xd &bound)// 0: lower bound, 1: upper bound, 2: initial value
        {
            if(state_dim != 0 && state_dim != bound.cols())
                ERROR("Recieve: {}, but expect: {}", bound.cols(), state_dim);
            state_dim = bound.cols();
            state_lower_bound.resize(state_dim);
            state_upper_bound.resize(state_dim);
            state.resize(state_dim);
            for (int i = 0; i < state_dim; i++)
            {
                state_lower_bound[i] = bound(0, i);
                state_upper_bound[i] = bound(1, i);
                state[i] = bound(2, i);
            }
        }
        void setStateCoef(Eigen::VectorXd &state_coef)
        {
            if(state_dim != 0 && state_dim != state_coef.size())
                ERROR("Recieve: {}, but expect: {}", state_coef.size(), state_dim);
            state_dim = state_coef.size();
            state_coef.resize(state_dim);
            for (int i = 0; i < state_dim; i++)
            {
                state_coef[i] = state_coef(i);
            }
        }
        void setMeasureCoef(Eigen::VectorXd &measure_coef)
        {
            if(measure_dim != 0 && measure_dim != measure_coef.size())
                ERROR("Recieve: {}, but expect: {}", measure_coef.size(), measure_dim);
            measure_dim = measure_coef.size();
            measure_coef.resize(measure_dim);
            for (int i = 0; i < measure_dim; i++)
            {
                measure_coef[i] = measure_coef(i);
            }
        }
        void setHistorySizeAndLoss(double max_loss, double min_loss, int max_size, int min_size)
        {
            loss_max = max_loss;
            loss_min = min_loss;
            max_history_size = max_size;
            min_history_size = min_size;
        }
        virtual ADvector ideal_stateUpdate(const ADvector& state, const double dt) = 0;
        virtual ADvector ideal_measure(const ADvector& state, const int measure_id) = 0;
        Dvector ideal_stateUpdate_noAD(const Dvector& state, const double dt)
        {
            ADvector state_AD(state.size());
            for (int i = 0; i < state.size(); i++)
            {
                state_AD[i] = state[i];
            }
            ADvector state_update = ideal_stateUpdate(state_AD, dt);
            Dvector state_update_noAD(state_update.size());
            for (int i = 0; i < state_update.size(); i++)
            {
                state_update_noAD[i] = CppAD::Value(state_update[i]);
            }
            return state_update_noAD;
        }
        Dvector ideal_measure_noAD(const Dvector& state, const int measure_id)
        {
            ADvector state_AD(state.size());
            for (int i = 0; i < state.size(); i++)
            {
                state_AD[i] = state[i];
            }
            ADvector measure = ideal_measure(state_AD, measure_id);
            Dvector measure_noAD(measure.size());
            for (int i = 0; i < measure.size(); i++)
            {
                measure_noAD[i] = CppAD::Value(measure[i]);
            }
            return measure_noAD;
        }
        bool Update(const Dvector &measure, const Time::TimeStamp &timestamp, const int measure_id);
        double timestamp2dt(const Time::TimeStamp &timestamp)
        {
            if (history_timepoint.size() == 0)
            {
                WARN("No history timepoint, return 0");
                return 0;
            }
            return (timestamp - history_timepoint[history_timepoint.size() - 1]).toSeconds();
        }
        void getCurrentState(Dvector &state_){state_ = state;}
        void getPredictMeasure(Dvector &measure, const double dt, const int measure_id)
        {
            Dvector state_update = ideal_stateUpdate_noAD(state, dt);
            measure = ideal_measure_noAD(state_update, measure_id);
        }
        CppAD::AD<double> loss(const ADvector &state_);
        void operator()(ADvector& constraints, const ADvector& state)
        {
            constraints[0] = loss(state);
        }
        
    private:
        //FIX PARAMETER
        std::string options;
        int state_dim = 0;
        Dvector state_lower_bound;
        Dvector state_upper_bound;
        Dvector state_coef;
        int measure_dim = 0;
        Dvector measure_coef;
        int max_history_size,min_history_size;
        double loss_max,loss_min;//if loss > loss_max, then current_history_size change to min_history_size

        //RUNTIME VARIABLE
        int current_history_size;
        double decay_rate;
        std::vector<Dvector> history_measure;
        std::vector<Time::TimeStamp> history_timepoint;
        std::vector<int> history_measure_id;
        Dvector state;
    };
    using Dvector = OptimizeSolve::Dvector;
    using ADvector = OptimizeSolve::ADvector;
}
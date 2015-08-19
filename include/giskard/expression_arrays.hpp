#ifndef GISKARD_EXPRESSION_ARRAYS_HPP
#define GISKARD_EXPRESSION_ARRAYS_HPP

#include <kdl/expressiontree.hpp>

namespace KDL
{
  template<typename ResultType>
  class ExpressionArray
  {
    public:
      typedef typename KDL::Expression<ResultType>::Ptr ExpressionTypePtr;
      typedef typename KDL::AutoDiffTrait<ResultType>::DerivType DerivType;
      typedef typename KDL::Expression<DerivType>::Ptr DerivExpressionTypePtr;

      size_t num_expressions() const
      {
        return expressions_.size();
      }

      size_t num_inputs() const
      {
        int result = 0;
        for(size_t i=0; i<get_expressions().size(); ++i)
          result = std::max(result, get_expressions()[i]->number_of_derivatives());
        return result;
      }

      const std::vector< ExpressionTypePtr >& get_expressions() const
      {
        return expressions_;
      }

      const ExpressionTypePtr& get_expression(size_t index) const
      {
        return expressions_[index];
      }

      size_t find_expression(const ExpressionTypePtr& expression) const
      {
        for(size_t i=0; i<num_expressions(); ++i)
          if(get_expression(i) == expression) return i;

        return num_expressions();
      }
        
      bool has_expression(const ExpressionTypePtr& expression) const
      {
        return find_expression(expression) != num_expressions();
      }

      void set_expression(const ExpressionTypePtr& expression, size_t index)
      {
        expressions_[index] = expression;
        prepare_internals();
      }

      void set_expressions(const std::vector< ExpressionTypePtr >& expressions)
      {
        expressions_ = expressions;
        prepare_internals(); 
      }

      void push_expression(const ExpressionTypePtr& expression)
      {
        expressions_.push_back(expression);
        prepare_internals();
      }

      ExpressionTypePtr pop_expression()
      {
        ExpressionTypePtr popped_expression = expressions_.back();
        expressions_.pop_back();
        prepare_internals();
        return popped_expression;
      } 

      void update(const std::vector< double >& inputs)
      {
        optimizer_.setInputValues(inputs);
        copy_results();
      }
        
      void update(const Eigen::VectorXd& inputs)
      {
        optimizer_.setInputValues(inputs);
        copy_results();
      }

      const Eigen::Matrix<ResultType, Eigen::Dynamic, 1>& get_values() const
      {
        return values_;
      }

      const Eigen::Matrix<DerivType, Eigen::Dynamic, Eigen::Dynamic> get_derivatives() const
      {
        return derivatives_;
      }

      std::vector<DerivExpressionTypePtr> get_derivative_expressions(size_t expression_index) const
      {
        std::vector<DerivExpressionTypePtr> result;
        for(size_t i=0; i<expressions_[expression_index]->number_of_derivatives(); ++i)
          result.push_back(expressions_[expression_index]->derivativeExpression(i));
        return result;
      }

    private:
      Eigen::Matrix<ResultType, Eigen::Dynamic, 1> values_;
      Eigen::Matrix<DerivType, Eigen::Dynamic, Eigen::Dynamic> derivatives_;
      std::vector< ExpressionTypePtr > expressions_;
      KDL::ExpressionOptimizer optimizer_;

      void prepare_internals()
      {
        prepare_optimizer();
        prepare_eigensizes();
      }

      void prepare_optimizer()
      {
       optimizer_.prepare(calculate_inputs());

       for(size_t i=0; i<expressions_.size(); ++i)
         expressions_[i]->addToOptimizer(optimizer_);
      }

      std::vector<int> calculate_inputs() const
      {
        std::vector<int> input_vars;
        for(size_t i=0; i<num_inputs(); ++i)
          input_vars.push_back(i);
        return input_vars;
      }

      void prepare_eigensizes()
      {
        values_.resize(num_expressions(), 1);
        derivatives_.resize(num_expressions(), num_inputs());        
      }

      void copy_results()
      {
        derivatives_.setZero();
        for(size_t i=0; i<expressions_.size(); ++i)
        {
          values_(i, 0) = expressions_[i]->value();
          for(size_t j=0; j<expressions_[i]->number_of_derivatives(); ++j)
            derivatives_(i,j) = expressions_[i]->derivative(j);
        }
      }
  };

  typedef ExpressionArray<double> DoubleExpressionArray;

}

#endif // GISKARD_EXPRESSION_ARRAYS_HPP

////////////////////////////////////////////////////////////////////////////
//	Module 		: problem_solver.h
//	Created 	: 24.02.2004
//  Modified 	: 10.03.2004
//	Author		: Dmitriy Iassenev
//	Description : Problem solver
////////////////////////////////////////////////////////////////////////////

#pragma once

#include "xrCore/Containers/AssociativeVector.hpp"
#include "Common/object_broker.h"

#include "xrAICore/Navigation/graph_engine.h"
#include "xrAICore/Navigation/graph_engine_space.h"

template <
    typename _operator_condition,
    typename _condition_state,
    typename _operator,
    typename _condition_evaluator,
    typename _operator_id_type,
    bool _reverse_search = false,
    typename _operator_ptr = _operator*,
    typename _condition_evaluator_ptr = _condition_evaluator*>
class CProblemSolver {
public:
    struct SOperator;

    static const bool reverse_search = _reverse_search;

    using COperator = _operator;
    using CState = _condition_state;
    using CConditionEvaluator = _condition_evaluator;
    using operator_ptr = _operator_ptr;
    using condition_type = typename _operator_condition::condition_type;
    using value_type = typename _operator_condition::value_type;
    using edge_value_type = typename _operator::edge_value_type;
    using _index_type = CState;
    using edge_type = _operator_id_type;
    using OPERATOR_VECTOR = xr_vector<SOperator>;
    using const_iterator = typename OPERATOR_VECTOR::const_iterator;
    using EVALUATORS = AssociativeVector<condition_type, _condition_evaluator_ptr>;
    using self_type = CProblemSolver<
        _operator_condition,
        _condition_state,
        _operator,
        _condition_evaluator,
        _operator_id_type,
        reverse_search,
        _operator_ptr,
        _condition_evaluator_ptr>;

    struct SOperator {
        _operator_id_type m_operator_id;
        _operator_ptr m_operator;

        SOperator(const _operator_id_type& operator_id, _operator_ptr _op) : m_operator_id(operator_id), m_operator(_op) {
        
        }

        bool operator<(const _operator_id_type& operator_id) const {
            return m_operator_id < operator_id;
        }
        
        _operator_ptr get_operator() const {
            return m_operator; 
        }
    };

protected:
    OPERATOR_VECTOR m_operators;
    EVALUATORS m_evaluators;
    xr_vector<_operator_id_type> m_solution;
    CState m_target_state;
    mutable CState m_current_state;
    mutable CState m_temp;
    mutable bool m_applied;
    bool m_actuality;
    bool m_solution_changed;
    bool m_failed;

private:
    template <bool a>
    bool is_goal_reached_impl(std::enable_if_t<!a, const _index_type&> vertex_index) const
    {
        return is_goal_reached_impl(vertex_index);
    }
    template <bool a>
    bool is_goal_reached_impl(std::enable_if_t<a, const _index_type&> vertex_index) const
    {
        return is_goal_reached_impl(vertex_index, true);
    }

    bool is_goal_reached_impl(const _index_type& vertex_index) const {
        static_assert(!reverse_search, "This function cannot be used in the REVERSE search.");
        
        auto vertex_cond_begin = vertex_index.conditions().begin();
        auto vertex_cond_end = vertex_index.conditions().end();

        auto target_cond_begin = target_state().conditions().begin();
        auto target_cond_end = target_state().conditions().end();

        auto state_cond_begin = current_state().conditions().begin();
        auto state_cond_end = current_state().conditions().end();
        
        while (target_cond_begin != target_cond_end && vertex_cond_begin != vertex_cond_end) {
            const auto& target_condition = target_cond_begin->condition();

            if (vertex_cond_begin->condition() < target_condition) {
                ++vertex_cond_begin;
            }
            else if (vertex_cond_begin->condition() > target_condition)
            {
                while (state_cond_begin != state_cond_end && state_cond_begin->condition() < target_condition) {
                    ++state_cond_begin;
                }
                    
                if (state_cond_begin == state_cond_end || state_cond_begin->condition() > target_condition) {
                    evaluate_condition(state_cond_begin, state_cond_end, target_condition);
                }

                if (state_cond_begin->value() != target_cond_begin->value()) {
                    return false;
                }
                    
                ++state_cond_begin;
            }
            else
            {
                if (vertex_cond_begin->value() != target_cond_begin->value()) {
                    return false;
                }
                   
                ++vertex_cond_begin;
            }

            ++target_cond_begin;
        }

        if (vertex_cond_begin == vertex_cond_end) {
            vertex_cond_begin = state_cond_begin;
            vertex_cond_end = state_cond_end;
        }
        else {
            return true;
        }   

        while (target_cond_begin != target_cond_end) {
            if (vertex_cond_begin == vertex_cond_end || vertex_cond_begin->condition() > target_cond_begin->condition()) {
                evaluate_condition(vertex_cond_begin, vertex_cond_end, target_cond_begin->condition());
            }
                
            if (vertex_cond_begin->condition() >= target_cond_begin->condition()) {
                VERIFY(vertex_cond_begin->condition() == target_cond_begin->condition());
                
                if (vertex_cond_begin->value() != target_cond_begin->value()) {
                    return false;
                }
                    
                ++target_cond_begin;
            }

             ++vertex_cond_begin;
        }

        return true;
    }
    bool is_goal_reached_impl(const _index_type& vertex_index, bool) const {
        static_assert(reverse_search, "This function cannot be used in the STRAIGHT search.");

        auto current_cond_begin = m_current_state.conditions().begin();
        auto current_cond_end = m_current_state.conditions().end();
        auto vertex_cond_begin = vertex_index.conditions().begin();
        auto vertex_cond_end = vertex_index.conditions().end();

        while (vertex_cond_begin != vertex_cond_end) {
            if (current_cond_begin == current_cond_end || current_cond_begin->condition() > vertex_cond_begin->condition()) {
                evaluate_condition(current_cond_begin, current_cond_end, vertex_cond_begin->condition());
            }

            if (current_cond_begin->condition() >= vertex_cond_begin->condition()) {
                if (current_cond_begin->value() != vertex_cond_begin->value()) {
                    return false;
                }

                ++vertex_cond_begin;
            }

            ++current_cond_begin;
        }
        return true;
    }

    edge_value_type estimate_edge_weight_impl(const _index_type& vertex_index) const {
        static_assert(!reverse_search, "This function cannot be used in the REVERSE search.");
        
        edge_value_type result{};
        auto target_cond_begin = target_state().conditions().begin();
        auto target_cond_end = target_state().conditions().end();
        auto vertex_cond_begin = vertex_index.conditions().begin();
        auto vertex_cond_end = vertex_index.conditions().end();

        while(target_cond_begin != target_cond_end && vertex_cond_begin != vertex_cond_end) {
            if (target_cond_begin->condition() < vertex_cond_begin->condition()) {
                ++result;
                ++target_cond_begin;
            }
            else if (target_cond_begin->condition() > vertex_cond_begin->condition()) {
                ++vertex_cond_begin;
            }
            else {
                if (target_cond_begin->value() != vertex_cond_begin->value()) {
                    ++result;
                }
                ++target_cond_begin;
                ++vertex_cond_begin;
            }
        }

        return result + static_cast<edge_value_type>(target_cond_end - target_cond_begin);
    }

    edge_value_type estimate_edge_weight_impl(const _index_type& vertex_index, bool) const {
        static_assert(reverse_search, "This function cannot be used in the STRAIGHT search.");
        
        edge_value_type result{};
        auto current_cond_begin = current_state().conditions().begin();
        auto current_cond_end = current_state().conditions().end();
        auto vertex_cond_begin = vertex_index.conditions().begin();
        auto vertex_cond_end = vertex_index.conditions().end();
        while (vertex_cond_begin != vertex_cond_end) {
            if (current_cond_begin == current_cond_end || current_cond_begin->condition() > vertex_cond_begin->condition()) {
                evaluate_condition(current_cond_begin, current_cond_end, vertex_cond_begin->condition());
            }

            if (current_cond_begin->condition() < vertex_cond_begin->condition()) {
                ++current_cond_begin;
            }
            else {
                VERIFY(current_cond_begin->condition() == vertex_cond_begin->condition());
                if (current_cond_begin->value() != vertex_cond_begin->value()) {
                    ++result;
                }

                ++current_cond_begin;
                ++vertex_cond_begin;
            }
        }

        return result;
    }

private:
    struct helper
    {
        template <bool a>
        static edge_value_type estimate_edge_weight_impl(std::enable_if_t<!a, self_type const&> self, const _index_type& vertex_index)
        {
            return self.estimate_edge_weight_impl(vertex_index);
        }

        template <bool a>
        static edge_value_type estimate_edge_weight_impl(
            std::enable_if_t<a, self_type const&> self, const _index_type& vertex_index)
        {
            return self.estimate_edge_weight_impl(vertex_index, true);
        }
    }; // struct helper

public:
    CProblemSolver() { 
        init();
    }

    virtual ~CProblemSolver() { 
        clear();
    }
    void init() {

    }

    virtual void setup() {
        m_target_state.clear();
        m_current_state.clear();
        m_temp.clear();
        m_solution.clear();
        m_applied = false;
        m_solution_changed = false;
        m_actuality = true;
        m_failed = false;
    }

    bool actual() const {
        if (!m_actuality) {
            return false;
        }

        auto i = evaluators().begin();
        auto e = evaluators().end();

        for (auto&& condition : current_state().conditions()) {
            if (i->first < condition.condition()) {
                i = std::lower_bound(i, e, condition.condition(), evaluators().value_comp());
            }

            if (i->second->evaluate() != condition.value()) {
                return false;
            }
        }

        return true;
    }

    // graph interface
    edge_value_type get_edge_weight(const _index_type& first_vertex, const _index_type& second_vertex, const const_iterator& iter) const {
        auto current = iter->m_operator->weight(second_vertex, first_vertex);
        auto min = iter->m_operator->min_weight();
        
        THROW(current >= min);
        return current;
    }

    bool is_accessible(const _index_type& vertex_index) const { 
        return m_applied;
    }

    const _index_type& value(const _index_type& vertex_index, const_iterator& iter, bool reverse_search) const {
        if (reverse_search) {
            if (iter->m_operator->applicable_reverse(iter->m_operator->effects(), iter->m_operator->conditions(), vertex_index)) {
                m_applied = iter->m_operator->apply_reverse(vertex_index, iter->m_operator->effects(), m_temp, iter->m_operator->conditions());
            }
            else {
                m_applied = false;
            }
        }
        else {
            if (iter->m_operator->applicable(vertex_index, current_state(), iter->m_operator->conditions(), *this)) {
                iter->m_operator->apply(vertex_index, iter->m_operator->effects(), m_temp, m_current_state, *this);
                m_applied = true;
            }
            else {
                m_applied = false;
            }
        }

        return m_temp;
    }

    void begin(const _index_type& vertex_index, const_iterator& b, const_iterator& e) const {
        b = m_operators.begin();
        e = m_operators.end();
    }

    bool is_goal_reached(const _index_type& vertex_index) const {
        return is_goal_reached_impl<reverse_search>(vertex_index);
    }

    edge_value_type estimate_edge_weight(const _index_type& vertex_index) const {
        return helper::template estimate_edge_weight_impl<reverse_search>(*this, vertex_index);
    }

    // operator interface
    virtual void add_operator(const _operator_id_type& operator_id, _operator_ptr op) {
        auto insert_place = std::lower_bound(m_operators.begin(), m_operators.end(), operator_id);
        THROW(insert_place == m_operators.end() || insert_place->m_operator_id != operator_id);

        m_actuality = false;
        m_operators.insert(insert_place, SOperator(operator_id, op));
    }

    virtual void remove_operator(const _operator_id_type& operator_id) {
        auto erase_place = std::lower_bound(m_operators.begin(), m_operators.end(), operator_id);
        THROW(m_operators.end() != erase_place);
        
        try {
            delete_data(erase_place->m_operator);
        }
        catch (...) {
            erase_place->m_operator = 0;
        }

        m_actuality = false;
        m_operators.erase(erase_place);
    }

    _operator_ptr get_operator(const _operator_id_type& operator_id) {
        auto iter = std::lower_bound(m_operators.begin(), m_operators.end(), operator_id);
        THROW(m_operators.end() != iter);

        return (iter->get_operator());
    }
    
    const OPERATOR_VECTOR& operators() const { 
        return m_operators;
    }

    // state interface
    void set_target_state(const CState& state) {
        m_actuality = m_actuality && (m_target_state == state);
        m_target_state = state;
    }
    const CState& current_state() const { 
        return m_current_state;
    }

    const CState& target_state() const { 
        return m_target_state;
    }

    // evaluator interface
    virtual void add_evaluator(const condition_type& condition_id, _condition_evaluator_ptr evaluator) {
        THROW(evaluators().end() == evaluators().find(condition_id));
        m_evaluators.insert(std::make_pair(condition_id, evaluator));
    }

    virtual void remove_evaluator(const condition_type& condition_id) {
        auto erase_place = m_evaluators.find(condition_id);
        THROW(erase_place != m_evaluators.end());
        
        try {
            delete_data(erase_place->second);
        }
        catch (...) {
            erase_place->second = 0;
        }

        m_evaluators.erase(erase_place);
        m_actuality = false;
    }

    _condition_evaluator_ptr evaluator(const condition_type& condition_id) const {
        auto item = evaluators().find(condition_id);
        THROW(evaluators().end() != item);
        
        return item->second;
    }

    const EVALUATORS& evaluators() const { 
        return m_evaluators;
    }

    template <typename Iterator>
    void evaluate_condition(Iterator& iter_begin, Iterator& iter_end, const condition_type& condition_id) const {
        size_t index = iter_begin - m_current_state.conditions().begin();
        
        m_current_state.add_condition(iter_begin, _operator_condition(condition_id, evaluator(condition_id)->evaluate()));
        
        iter_begin = m_current_state.conditions().begin() + index;
        iter_end = m_current_state.conditions().end();
    }

    // solver interface
    void solve();

    const xr_vector<_operator_id_type>& solution() const {
        return m_solution;
    }

    virtual void clear() {
        while (!m_operators.empty()) {
            remove_operator(m_operators.back().m_operator_id);
        }
            
        while (!m_evaluators.empty()) {
            remove_evaluator((*(m_evaluators.end() - 1)).first);
        }
    }
};



#define TEMPLATE_SPECIALIZATION                                                                                  \
    template <typename _operator_condition, typename _operator, typename _condition_state,                       \
        typename _condition_evaluator, typename _operator_id_type, bool reverse_search, typename _operator_ptr, \
        typename _condition_evaluator_ptr>

#define CProblemSolverAbstract                                                                                \
    CProblemSolver<_operator_condition, _operator, _condition_state, _condition_evaluator, _operator_id_type, \
        reverse_search, _operator_ptr, _condition_evaluator_ptr>

TEMPLATE_SPECIALIZATION
void CProblemSolverAbstract::solve()
{
#ifndef AI_COMPILER
    m_solution_changed = false;

    if (actual())
        return;

    m_actuality = true;
    m_solution_changed = true;
    m_current_state.clear();

    m_failed = !ai().graph_engine().search(*this, reverse_search ? target_state() : current_state(),
        reverse_search ? current_state() : target_state(), &m_solution,
        GraphEngineSpace::CSolverBaseParameters(
            GraphEngineSpace::_solver_dist_type(-1), GraphEngineSpace::_solver_condition_type(-1), 8000));
#endif
}

#undef TEMPLATE_SPECIALIZATION
#undef CProblemSolverAbstract

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

template <typename _operator_condition, typename _condition_state, typename _operator, typename _condition_evaluator,
    typename _operator_id_type, bool _reverse_search = false, typename _operator_ptr = _operator*,
    typename _condition_evaluator_ptr = _condition_evaluator*>
class CProblemSolver
{
public:
    static const bool reverse_search = _reverse_search;

private:
    typedef CProblemSolver<_operator_condition, _condition_state, _operator, _condition_evaluator, _operator_id_type,
        reverse_search, _operator_ptr, _condition_evaluator_ptr>
        self_type;

public:
    typedef _operator COperator;
    typedef _condition_state CState;
    typedef _condition_evaluator CConditionEvaluator;
    typedef _operator_ptr operator_ptr;
    typedef typename _operator_condition::condition_type condition_type;
    typedef typename _operator_condition::value_type value_type;
    typedef typename _operator::edge_value_type edge_value_type;
    typedef CState _index_type;
    typedef _operator_id_type edge_type;

    struct SOperator
    {
        _operator_id_type m_operator_id;
        _operator_ptr m_operator;

        SOperator(const _operator_id_type& operator_id, _operator_ptr _op)
            : m_operator_id(operator_id), m_operator(_op)
        {
        }

        bool operator<(const _operator_id_type& operator_id) const { return (m_operator_id < operator_id); }
        _operator_ptr get_operator() const { return (m_operator); }
    };
    typedef xr_vector<SOperator> OPERATOR_VECTOR;
    typedef typename OPERATOR_VECTOR::const_iterator const_iterator;
    typedef AssociativeVector<condition_type, _condition_evaluator_ptr> EVALUATORS;

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
                evaluate_condition(vertex_cond_begin, vertex_cond_end, (*target_cond_begin).condition());
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
    bool is_goal_reached_impl(const _index_type& vertex_index, bool) const;

    edge_value_type estimate_edge_weight_impl(const _index_type& vertex_index) const;
    edge_value_type estimate_edge_weight_impl(const _index_type& vertex_index, bool) const;

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

    edge_value_type estimate_edge_weight(const _index_type& vertex_index) const;

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

    _operator_ptr get_operator(const _operator_id_type& operator_id);
    
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

    const xr_vector<_operator_id_type>& solution() const;

    virtual void clear() {
        while (!m_operators.empty()) {
            remove_operator(m_operators.back().m_operator_id);
        }
            
        while (!m_evaluators.empty()) {
            remove_evaluator((*(m_evaluators.end() - 1)).first);
        }
    }
};

#include "xrAICore/Navigation/graph_engine.h"
#include "xrAICore/Navigation/graph_engine_space.h"

#define TEMPLATE_SPECIALIZATION                                                                                  \
    template <typename _operator_condition, typename _operator, typename _condition_state,                       \
        typename _condition_evaluator, typename _operator_id_type, bool reverse_search, typename _operator_ptr, \
        typename _condition_evaluator_ptr>

#define CProblemSolverAbstract                                                                                \
    CProblemSolver<_operator_condition, _operator, _condition_state, _condition_evaluator, _operator_id_type, \
        reverse_search, _operator_ptr, _condition_evaluator_ptr>

TEMPLATE_SPECIALIZATION
bool CProblemSolverAbstract::is_goal_reached_impl(const _index_type& vertex_index, bool) const
{
    static_assert(reverse_search, "This function cannot be used in the STRAIGHT search.");
    typename xr_vector<_operator_condition>::const_iterator I = m_current_state.conditions().begin();
    typename xr_vector<_operator_condition>::const_iterator E = m_current_state.conditions().end();
    typename xr_vector<_operator_condition>::const_iterator i = vertex_index.conditions().begin();
    typename xr_vector<_operator_condition>::const_iterator e = vertex_index.conditions().end();
    for (; i != e;)
    {
        if ((I == E) || ((*I).condition() > (*i).condition()))
            evaluate_condition(I, E, (*i).condition());

        if ((*I).condition() < (*i).condition())
            ++I;
        else
        {
            if ((*I).value() != (*i).value())
                return (false);
            ++I;
            ++i;
        }
    }
    return (true);
}

TEMPLATE_SPECIALIZATION
const xr_vector<_operator_id_type>& CProblemSolverAbstract::solution() const { return (m_solution); }

TEMPLATE_SPECIALIZATION
_operator_ptr CProblemSolverAbstract::get_operator(const _operator_id_type& operator_id)
{
    typename OPERATOR_VECTOR::iterator I = std::lower_bound(m_operators.begin(), m_operators.end(), operator_id);
    THROW(m_operators.end() != I);
    return ((*I).get_operator());
}

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

TEMPLATE_SPECIALIZATION
typename CProblemSolverAbstract::edge_value_type CProblemSolverAbstract::estimate_edge_weight(
    const _index_type& condition) const
{
    return (helper::template estimate_edge_weight_impl<reverse_search>(*this, condition));
}

TEMPLATE_SPECIALIZATION
typename CProblemSolverAbstract::edge_value_type CProblemSolverAbstract::estimate_edge_weight_impl(
    const _index_type& condition) const
{
    static_assert(!reverse_search, "This function cannot be used in the REVERSE search.");
    edge_value_type result = 0;
    typename xr_vector<_operator_condition>::const_iterator I = target_state().conditions().begin();
    typename xr_vector<_operator_condition>::const_iterator E = target_state().conditions().end();
    typename xr_vector<_operator_condition>::const_iterator i = condition.conditions().begin();
    typename xr_vector<_operator_condition>::const_iterator e = condition.conditions().end();
    for (; (I != E) && (i != e);)
        if ((*I).condition() < (*i).condition())
        {
            ++result;
            ++I;
        }
        else if ((*I).condition() > (*i).condition())
            ++i;
        else
        {
            if ((*I).value() != (*i).value())
                ++result;
            ++I;
            ++i;
        }
    return (result + edge_value_type(E - I));
}

TEMPLATE_SPECIALIZATION
typename CProblemSolverAbstract::edge_value_type CProblemSolverAbstract::estimate_edge_weight_impl(
    const _index_type& condition, bool) const
{
    static_assert(reverse_search, "This function cannot be used in the STRAIGHT search.");
    edge_value_type result = 0;
    typename xr_vector<_operator_condition>::const_iterator I = current_state().conditions().begin();
    typename xr_vector<_operator_condition>::const_iterator E = current_state().conditions().end();
    typename xr_vector<_operator_condition>::const_iterator i = condition.conditions().begin();
    typename xr_vector<_operator_condition>::const_iterator e = condition.conditions().end();
    for (; (i != e);)
    {
        if ((I == E) || ((*I).condition() > (*i).condition()))
            evaluate_condition(I, E, (*i).condition());

        if ((*I).condition() < (*i).condition())
            ++I;
        else
        {
            VERIFY((*I).condition() == (*i).condition());
            if ((*I).value() != (*i).value())
                ++result;
            ++I;
            ++i;
        }
    }
    return (result);
}

#undef TEMPLATE_SPECIALIZATION
#undef CProblemSolverAbstract

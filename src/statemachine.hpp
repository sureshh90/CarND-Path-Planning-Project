#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <vector>

enum class LongitudinalState
{
    INIT = -1,
    ACCELERATE = 0,
    DECELERATE = 1,
    MAINTAIN = 2,
    HARD_DECELERATE = 3
};

enum class LateralState
{
    INIT = -1,
    KEEP_LANE = 0,
    PREPARE_CHANGE_LANE_LEFT = 1,
    PREPARE_CHANGE_LANE_RIGHT = 2,
    CHANGE_LANE_LEFT = 3,
    CHANGE_LANE_RIGHT = 4
};

class State
{
public:
    State(){};
    State(LongitudinalState longitudinal_state, LateralState lateral_state, int current_lane, int future_lane)
    {
        this->s_state = longitudinal_state;
        this->d_state = lateral_state;
        this->current_lane = current_lane;
        this->future_lane = future_lane;
    }

    ~State(){};

    LongitudinalState s_state;
    LateralState d_state;

    int current_lane;
    int future_lane;
};

class StateMachine
{

    State m_current_state;

public:
    StateMachine()
    {
        m_current_state = State(LongitudinalState::INIT, LateralState::INIT, -1, -1);
    };

    std::vector<State> get_all_successor_states()
    {
        std::vector<State> successor_states;
        switch (m_current_state.d_state)
        {
        case LateralState::INIT:
            std::cout << "State not possible. Please update initial state." << std::endl;
            break;
        case LateralState::KEEP_LANE:
            successor_states.push_back(State(LongitudinalState::ACCELERATE,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));
            successor_states.push_back(State(LongitudinalState::MAINTAIN,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));
            successor_states.push_back(State(LongitudinalState::DECELERATE,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));
            successor_states.push_back(State(LongitudinalState::HARD_DECELERATE,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));

            successor_states.push_back(State(LongitudinalState::MAINTAIN,
                                             LateralState::PREPARE_CHANGE_LANE_LEFT,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane - 1));
            successor_states.push_back(State(LongitudinalState::ACCELERATE,
                                             LateralState::PREPARE_CHANGE_LANE_LEFT,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane - 1));
            successor_states.push_back(State(LongitudinalState::DECELERATE,
                                             LateralState::PREPARE_CHANGE_LANE_LEFT,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane - 1));
            successor_states.push_back(State(LongitudinalState::HARD_DECELERATE,
                                             LateralState::PREPARE_CHANGE_LANE_LEFT,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane - 1));

            successor_states.push_back(State(LongitudinalState::MAINTAIN,
                                             LateralState::PREPARE_CHANGE_LANE_RIGHT,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane + 1));
            successor_states.push_back(State(LongitudinalState::ACCELERATE,
                                             LateralState::PREPARE_CHANGE_LANE_RIGHT,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane + 1));
            successor_states.push_back(State(LongitudinalState::DECELERATE,
                                             LateralState::PREPARE_CHANGE_LANE_RIGHT,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane + 1));
            successor_states.push_back(State(LongitudinalState::HARD_DECELERATE,
                                             LateralState::PREPARE_CHANGE_LANE_RIGHT,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane + 1));
            break;

        case LateralState::PREPARE_CHANGE_LANE_LEFT:
            successor_states.push_back(State(LongitudinalState::MAINTAIN,
                                             LateralState::CHANGE_LANE_LEFT,
                                             m_current_state.future_lane,
                                             m_current_state.future_lane));
            successor_states.push_back(State(LongitudinalState::MAINTAIN,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));
            successor_states.push_back(State(LongitudinalState::DECELERATE,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));
            successor_states.push_back(State(LongitudinalState::HARD_DECELERATE,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));
            break;

        case LateralState::PREPARE_CHANGE_LANE_RIGHT:
            successor_states.push_back(State(LongitudinalState::MAINTAIN,
                                             LateralState::CHANGE_LANE_RIGHT,
                                             m_current_state.future_lane,
                                             m_current_state.future_lane));
            successor_states.push_back(State(LongitudinalState::MAINTAIN,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));
            successor_states.push_back(State(LongitudinalState::DECELERATE,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));
            successor_states.push_back(State(LongitudinalState::HARD_DECELERATE,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));
            break;

        default:
            successor_states.push_back(State(LongitudinalState::MAINTAIN,
                                             LateralState::KEEP_LANE,
                                             m_current_state.current_lane,
                                             m_current_state.current_lane));
        }
        return successor_states;
    }

    std::vector<State> get_next_reachable_states()
    {
        std::vector<State> reachable_states;
        std::vector<State> all_successor_states = StateMachine::get_all_successor_states();

        std::for_each(all_successor_states.begin(), all_successor_states.end(),
                      [&reachable_states](State const &successor_state)
                      {
                          if (is_lane_valid(successor_state.current_lane) && is_lane_valid(successor_state.future_lane))
                          {
                              reachable_states.push_back(successor_state);
                          }
                      });

        return reachable_states;
    }

    const State get_current_fsm_state() const
    {
        return m_current_state;
    }

    void update_current_fsm_state(State new_state)
    {
        m_current_state = new_state;
    }
};

#endif
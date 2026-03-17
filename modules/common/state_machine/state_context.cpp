/**
 * @file              state_context.cpp
 * @author       jiangchengjie
 * @brief
 * @version     1.0.0
 * @date           2021-10-29 02:16:31
 * @copyright Copyright (c) 2021
 * @license      GNU General Public License (GPL)
 */
#include "state_context.hpp"

#include <sched.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include <cassert>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include "state.hpp"

namespace legionclaw {
namespace common {
namespace state_machine {
bool StateContext::SetCallback(
    const CallbackType& _type, const std::string& _state_name,
    const std::function<void(const std::string&)>& _f) {
  bool ret = false;
  int32_t _state_id = GetStateIDbyName(_state_name);
  if (_state_id != -1 && GetStatePtr(static_cast<uint64_t>(_state_id))) {
    switch (_type) {
    case CallbackType::UPDATE:
      GetStatePtr(static_cast<uint64_t>(_state_id))->SetCallbackUpdate(_f);
      ret = true;
      break;
    case CallbackType::ENTRY:
      GetStatePtr(static_cast<uint64_t>(_state_id))->SetCallbackEntry(_f);
      ret = true;
      break;
    case CallbackType::EXIT:
      GetStatePtr(static_cast<uint64_t>(_state_id))->SetCallbackExit(_f);
      ret = true;
      break;
    default:
      break;
    }
  }
  return ret;
}
/*****************************/
/*  Callback                 */
/*****************************/

void StateContext::OnUpdate(void) { root_state_->OnUpdate(); }

bool StateContext::IsCurrentState(const std::string& state_name) {
  bool ret = false;
  std::shared_ptr<State> state = root_state_;
  do {
    if (state->GetStateName() == state_name) {
      ret = true;
    }
    state = state->GetChild();
  } while (state != nullptr);
  return ret;
}

void StateContext::NextState(const std::string& transition_key) {
  std::shared_ptr<State> state = root_state_;
  std::string _target_state_name;
  std::vector<std::string> key_list;

  while (state) {
    if (state->GetTransitionMap().count(transition_key) != 0) {
      const uint64_t transition_state_id =
          state->GetTransitionMap().at(transition_key);
      _target_state_name = state_map_.at(transition_state_id)->GetStateName();

      if (IsCurrentState(_target_state_name)) {
        return;
      }

      if (state_map_.at(transition_state_id)->GetParent()) {
        // ADEBUG << "[Child]:TransitionState: " << state->GetStateID() << "->"
        //        << transition_state_id;

        std::shared_ptr<State> in_state = root_state_;

        do {
          if (in_state == state_map_.at(transition_state_id)->GetParent()) {
            if (in_state->GetChild()) {
              key_list.push_back(in_state->GetChild()->GetEnteredKey());
              in_state->GetChild()->OnExit();
            }
            in_state->SetChild(state_map_.at(transition_state_id));
            break;
          }
          in_state = in_state->GetChild();
        } while (in_state);

        if (debug_ == true) {
          CreateDOTGraph(dot_output_name_);
        }

        state_map_.at(transition_state_id)->SetEnteredKey(transition_key);
        state_map_.at(transition_state_id)->OnEntry();
      } else {
        // ADEBUG << "[Child]:TransitionState: " << state->GetStateID() << "->"
        //        << transition_state_id;
        state->OnExit();

        root_state_ = state_map_.at(transition_state_id);
        root_state_->SetChild(nullptr);
        root_state_->SetParent(nullptr);
        root_state_->SetEnteredKey(transition_key);

        if (debug_ == true) {
          CreateDOTGraph(dot_output_name_);
        }

        root_state_->OnEntry();
      }
      break;
    }
    state = state->GetChild();
  }

  if (IsCurrentState(_target_state_name)) {
    ShowStateName();
  }
}

/*****************************/
/* Getter/Setter             */
/*****************************/

void StateContext::CreateGraphTransitionList(
    std::ofstream& outputfile, int idx,
    std::map<uint64_t, std::vector<uint64_t>>& sublist) {
  if (!sublist[idx].empty() || state_map_.at(idx)->GetParent() == NULL) {
    outputfile << "subgraph cluster_" << idx << "{\n"
               << "label = \"" << state_map_.at(idx)->GetStateName() << "\";\n";
    if (!state_map_.at(idx)->GetParent()) {
      outputfile << "group = 1;\n";
    }

    for (auto& state : sublist[idx]) {
      CreateGraphTransitionList(outputfile, state, sublist);
    }
  }

  for (auto& map : state_map_.at(idx)->GetTransitionMap()) {
    if ((state_map_.at(map.second)->GetParent() ==
             state_map_.at(idx)->GetParent() ||
         state_map_.at(map.second)->GetParent() == state_map_.at(idx)) &&
        state_map_.at(map.second)->GetParent() != NULL) {
      outputfile << idx << "->" << map.second << " [label=\"" << map.first
                 << "\"];\n";
    }
  }
  if (!sublist[idx].empty() || state_map_.at(idx)->GetParent() == NULL) {
    outputfile << "}\n";
  }
  for (auto& map : state_map_.at(idx)->GetTransitionMap()) {
    if ((state_map_.at(map.second)->GetParent() !=
             state_map_.at(idx)->GetParent() &&
         state_map_.at(map.second)->GetParent() != state_map_.at(idx)) ||
        state_map_.at(map.second)->GetParent() == NULL) {
      outputfile << idx << "->" << map.second << " [label=\"" << map.first
                 << "\"];\n";
    }
  }
}

void StateContext::CreateDOTGraph(std::string _file_name) {
  std::ofstream outputfile(_file_name);
  outputfile << "digraph state_machine_graph {\n"
             << "dpi = \"192\";\n node [style=filled];\n";
  std::vector<uint64_t> rootlist;
  std::map<uint64_t, std::vector<uint64_t>> sublist;
  std::map<uint64_t, int> layer_map;

  // create child list
  for (auto& state : state_map_) {
    outputfile << state.second->GetStateID() << "[label=\""
               << state.second->GetStateName() << "\"";

    {
      std::shared_ptr<State> temp = root_state_;
      while (temp) {
        if (temp->GetStateID() == state.second->GetStateID()) {
          outputfile << ",color = \"crimson\"";
        }
        temp = temp->GetChild();
      }
    }
    if (state.second->GetParent()) {
      sublist[state.second->GetParent()->GetStateID()].push_back(
          state.second->GetStateID());
    } else {
      outputfile << ", group = 1";
      rootlist.push_back(state.second->GetStateID());
    }
    outputfile << "];\n";
  }

  for (auto& root_idx : rootlist) {
    int idx = root_idx;
    CreateGraphTransitionList(outputfile, idx, sublist);
  }

  outputfile << "}";
}

std::shared_ptr<State> StateContext::GetStartState() {
  for (auto& state : state_map_) {
    if (state.second->GetStateName() == "Start") {
      // std::cout<<"state.second;:"<<state.second<<"\n";
      return state.second;
    }
  }
  return nullptr;
}

int32_t StateContext::GetStateIDbyName(std::string _name) {
  for (const auto& i : state_map_) {
    if (i.second->GetStateName() == _name) {
      return static_cast<int32_t>(i.second->GetStateID());
    }
  }
  return -1;
}

std::string StateContext::GetAvailableTransition(void) {
  std::string text;

  std::shared_ptr<State> state = root_state_;
  do {
    for (const auto& keyval : state->GetTransitionMap()) {
      text += keyval.first + ":" +
              state_map_.at(keyval.second)->GetStateName() + ",";
    }
    state = state->GetChild();
  } while (state != nullptr);

  return text;
}

std::string StateContext::GetStateText(void) {
  std::string text;

  std::shared_ptr<State> state = root_state_;
  do {
    text += state->GetStateName() + "\n";
    state = state->GetChild();
  } while (state != nullptr);

  return text;
}

std::string StateContext::GetCurrentStateName() {
  std::string text;
  std::shared_ptr<State> state_temp = nullptr;
  std::shared_ptr<State> state = root_state_;
  do {
    state_temp = state->GetChild();
    if (state_temp == nullptr) {
      break;
    }
    state = state_temp;
  } while (state != nullptr);
  text = state->GetStateName();
  return text;
}

void StateContext::SetTransitionMap(const json& node,
                                    const std::shared_ptr<State>& _state) {
  for (unsigned int j = 0; j < node.size(); j++) {
    int32_t state_id = GetStateIDbyName(node[j]["Target"]);
    if (state_id == -1)
      continue;

    _state->AddTransition(node[j]["Key"], static_cast<uint64_t>(state_id));
  }
}

void StateContext::ShowStateName() {
  std::shared_ptr<State> state = root_state_;
  do {
    state = state->GetChild();
  } while (state != nullptr);
}

std::shared_ptr<State> StateContext::GetStatePtr(const json& node) {
  return GetStatePtr((std::string)node["StateName"]);
}

std::shared_ptr<State>
StateContext::GetStatePtr(const std::string& _state_name) {
  int32_t state_id = GetStateIDbyName(_state_name);
  if (_state_name == "null" || state_id == -1)
    return nullptr;
  else
    return GetStatePtr(static_cast<uint64_t>(state_id));
}

std::shared_ptr<State> StateContext::GetStatePtr(const uint64_t& _state_id) {
  return state_map_.at(_state_id);
}

void StateContext::CeateStateMap(std::string _state_file_name,
                                 std::string _msg_name) {
  json state_json;
  std::ifstream in(_state_file_name);
  in >> state_json;

  if (state_json.is_null()) {
    std::cout << "state_json is null" << "\n";
    return;
  }

  // create state
  for (unsigned int i = 0; i < state_json[_msg_name].size(); i++) {
    state_map_[i] = std::shared_ptr<State>(
        new State(state_json[_msg_name][i]["StateName"], i));
  }

  // set Parent
  // set transition
  for (unsigned int i = 0; i < state_json[_msg_name].size(); i++) {
    GetStatePtr((std::string)state_json[_msg_name][i]["Parent"]);

    GetStatePtr(state_json[_msg_name][i])
        ->SetParent(
            GetStatePtr((std::string)state_json[_msg_name][i]["Parent"]));

    SetTransitionMap(state_json[_msg_name][i]["Transition"],
                     GetStatePtr(state_json[_msg_name][i]));
  }
}

void StateContext::Reset() { root_state_ = GetStartState(); }
} // namespace state_machine
} // namespace common
} // namespace legionclaw
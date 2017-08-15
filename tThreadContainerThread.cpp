//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    plugins/scheduling/tThreadContainerThread.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 */
//----------------------------------------------------------------------
#include "plugins/scheduling/tThreadContainerThread.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tRuntimeEnvironment.h"
#include "core/port/tAggregatedEdge.h"
#include <set>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/scheduling/tExecutionControl.h"
#include "plugins/scheduling/tPeriodicFrameworkElementTask.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace scheduling
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

/*! Flags used for storing information in tPeriodicFrameworkElementTask::task_classification */
enum tTaskClassificationFlag
{
  eSENSE_TASK = 1,
  eSENSE_DEPENDENCY = 2,
  eSENSE_DEPENDENT = 4,
  eCONTROL_TASK = 8,
  eCONTROL_DEPENDENCY = 16,
  eCONTROL_DEPENDENT = 32
};

typedef core::tFrameworkElement::tFlag tFlag;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
#define FINROC_PORT_BASED_SCHEDULING

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

tThreadContainerThread* tThreadContainerThread::single_thread_container = nullptr;

// Abort predicates for tThreadContainerThread::ForEachConnectedTask()
static bool IsSensorInterface(core::tEdgeAggregator& ea)
{
  return ea.GetFlag(core::tFrameworkElement::tFlag::SENSOR_DATA);
}
static bool IsControllerInterface(core::tEdgeAggregator& ea)
{
  return ea.GetFlag(core::tFrameworkElement::tFlag::CONTROLLER_DATA);
}
static bool IsSensorOrControllerInterface(core::tEdgeAggregator& ea)
{
  return IsSensorInterface(ea) || IsControllerInterface(ea);
}
static bool AlwaysFalse(core::tEdgeAggregator& ea)
{
  return false;
}

/*!
 * \param fe Framework element
 * \return Is framework element an interface?
 */
static inline bool IsInterface(core::tFrameworkElement& fe)
{
  return fe.GetFlag(tFlag::EDGE_AGGREGATOR) || fe.GetFlag(tFlag::INTERFACE);
}


tThreadContainerThread::tThreadContainerThread(core::tFrameworkElement& thread_container, rrlib::time::tDuration default_cycle_time,
    bool warn_on_cycle_time_exceed, data_ports::tOutputPort<rrlib::time::tDuration> execution_duration,
    data_ports::tOutputPort<std::vector<tTaskProfile>> execution_details) :
  tLoopThread(default_cycle_time, true, warn_on_cycle_time_exceed),
  tWatchDogTask(true),
  thread_container(thread_container),
  reschedule(true),
  schedule(),
  task_set_first_index { 0, 0, 0, 0 },
                     execution_duration(execution_duration),
                     execution_details(execution_details),
                     total_execution_duration(0),
                     max_execution_duration(0),
                     execution_count(0),
                     current_task(NULL),
                     current_cycle_start_application_time(rrlib::time::cNO_TIME)
{
  this->SetName("ThreadContainer " + thread_container.GetName());
  this->thread_container.GetRuntime().AddListener(*this);
#ifdef RRLIB_SINGLE_THREADED
  assert(single_thread_container == nullptr);
  single_thread_container = this;
#endif
}

tThreadContainerThread::~tThreadContainerThread()
{
  this->thread_container.GetRuntime().RemoveListener(*this);
  single_thread_container = nullptr;
}

std::string tThreadContainerThread::CreateLoopDebugOutput(const std::vector<tPeriodicFrameworkElementTask*>& task_list)
{
  std::ostringstream stream;
  for (auto it = task_list.rbegin(); it != task_list.rend(); ++it)
  {
    stream << (it != task_list.rbegin() ? "-> " : "   ");
    stream << (*it)->GetLogDescription() << std::endl;
    for (auto next = (*it)->next_tasks.begin(); next != (*it)->next_tasks.end(); ++next)
    {
      if (*next == *task_list.rbegin())
      {
        stream << "-> " << (*next)->GetLogDescription();
        return stream.str();
      }
    }
  }
  return "ERROR";
}

template <typename T1, typename T2>
void Increment(bool trace_reverse, T1& it_incoming, T2& it_outgoing)
{
  if (trace_reverse)
  {
    it_incoming++;
  }
  else
  {
    it_outgoing++;
  }
}

template <bool (ABORT_PREDICATE)(core::tEdgeAggregator&), class TFunction>
void tThreadContainerThread::ForEachConnectedTask(core::tEdgeAggregator& origin, std::vector<core::tEdgeAggregator*>& trace, TFunction& function, bool trace_reverse)
{
  // Add to trace stack
  trace.push_back(&origin);

#ifdef FINROC_PORT_BASED_SCHEDULING
  for (auto it = origin.ChildPortsBegin(); it != origin.ChildPortsEnd(); ++it)
  {
    ForEachConnectedTask<ABORT_PREDICATE, TFunction>(*it, trace, function, trace_reverse);
  }

  // remove from trace stack
  assert(trace[trace.size() - 1] == &origin);
  trace.pop_back();
}

template <bool (ABORT_PREDICATE)(core::tEdgeAggregator&), class TFunction>
void tThreadContainerThread::ForEachConnectedTask(core::tAbstractPort& origin, std::vector<core::tEdgeAggregator*>& trace, TFunction& function, bool trace_reverse)
{
#endif

  auto it_incoming = origin.IncomingConnectionsBegin();
  auto end_incoming = origin.IncomingConnectionsEnd();
  auto it_outgoing = origin.OutgoingConnectionsBegin();
  auto end_outgoing = origin.OutgoingConnectionsEnd();
  for (; trace_reverse ? it_incoming != end_incoming : it_outgoing != end_outgoing; Increment(trace_reverse, it_incoming, it_outgoing))
  {
#ifdef FINROC_PORT_BASED_SCHEDULING
    core::tAbstractPort& dest_port = trace_reverse ? it_incoming->Source() : it_outgoing->Destination();
    core::tEdgeAggregator* dest_aggregator = core::tEdgeAggregator::GetAggregator(dest_port);
#else
    core::tAggregatedEdge& aggregated_edge = trace_reverse ? **it_incoming : **it_outgoing;
    core::tEdgeAggregator* dest_aggregator = trace_reverse ? &aggregated_edge.source : &aggregated_edge.destination;
#endif
    tExecutionControl* execution_control = dest_aggregator ? tExecutionControl::Find(*dest_aggregator) : nullptr;
    if ((!execution_control) || execution_control->GetAnnotated<core::tFrameworkElement>() != &thread_container)
    {
      continue;
    }
    core::tEdgeAggregator& dest = *dest_aggregator;
    if (ABORT_PREDICATE(dest))
    {
      continue;
    }

    if (std::find(trace.begin(), trace.end(), &dest) == trace.end())
    {
      // Have we reached another task?
      tPeriodicFrameworkElementTask* connected_task = dest.GetAnnotation<tPeriodicFrameworkElementTask>();
      if (connected_task == NULL && IsInterface(dest))
      {
        connected_task = dest.GetParent()->GetAnnotation<tPeriodicFrameworkElementTask>();
      }
      if (connected_task == NULL && trace_reverse && IsInterface(dest))
      {
        for (auto child = dest.GetParent()->ChildrenBegin(); child != dest.GetParent()->ChildrenEnd(); ++child)
        {
          if (IsInterface(*child))
          {
            tPeriodicFrameworkElementTask* task_to_test = child->template GetAnnotation<tPeriodicFrameworkElementTask>();
            if (task_to_test && std::find(task_to_test->outgoing.begin(), task_to_test->outgoing.end(), &dest) != task_to_test->outgoing.end())
            {
              connected_task = task_to_test;
              break;
            }
          }
        }
      }
      if (connected_task)
      {
        function(*connected_task);
        continue;
      }

      // continue from this edge aggregator
      auto it_next = trace_reverse ? dest.IncomingConnectionsBegin() : dest.OutgoingConnectionsBegin();
      auto end_next = trace_reverse ? dest.IncomingConnectionsEnd() : dest.OutgoingConnectionsEnd();
      if (it_next != end_next) // not empty?
      {
#ifdef FINROC_PORT_BASED_SCHEDULING
        trace.push_back(&dest);
        ForEachConnectedTask<ABORT_PREDICATE, TFunction>(dest_port, trace, function, trace_reverse);
        trace.pop_back();
#else
        ForEachConnectedTask<ABORT_PREDICATE, TFunction>(dest, trace, function, trace_reverse);
#endif
      }
      else if (IsModuleInputInterface(dest)) // in case we have a module with event-triggered execution (and, hence, no periodic task)
      {
        core::tFrameworkElement* parent = dest.GetParent();
        if (parent->GetFlag(tFlag::EDGE_AGGREGATOR))
        {
          core::tEdgeAggregator* ea = static_cast<core::tEdgeAggregator*>(parent);
          if (std::find(trace.begin(), trace.end(), ea) == trace.end())
          {
            ForEachConnectedTask<ABORT_PREDICATE, TFunction>(*ea, trace, function, trace_reverse);
          }
        }
        // if we have e.g. an sensor input interface, only continue with sensor output
        uint required_flags = dest.GetAllFlags().Raw() & (tFlag::SENSOR_DATA | tFlag::CONTROLLER_DATA).Raw();
        required_flags |= (tFlag::READY | tFlag::EDGE_AGGREGATOR | tFlag::INTERFACE).Raw();
        for (auto it = parent->ChildrenBegin(); it != parent->ChildrenEnd(); ++it)
        {
          if ((it->GetAllFlags().Raw() & required_flags) == required_flags)
          {
            core::tEdgeAggregator& ea = static_cast<core::tEdgeAggregator&>(*it);
            if (std::find(trace.begin(), trace.end(), &ea) == trace.end())
            {
              ForEachConnectedTask<ABORT_PREDICATE, TFunction>(ea, trace, function, trace_reverse);
            }
          }
        }
      }
    }
  }

#ifndef FINROC_PORT_BASED_SCHEDULING
  // remove from trace stack
  assert(trace[trace.size() - 1] == &origin);
  trace.pop_back();
#endif
}

void tThreadContainerThread::HandleWatchdogAlert()
{
  tPeriodicFrameworkElementTask* task = current_task;
  if (!task)
  {
    FINROC_LOG_PRINT(ERROR, "Got stuck without executing any task!? This should not happen.");
  }
  else
  {
    core::tFrameworkElement* stuck_element = task->incoming.size() > 0 ? task->incoming[0] : task->GetAnnotated<core::tFrameworkElement>();
    FINROC_LOG_PRINT(ERROR, "Got stuck executing task associated with '", *stuck_element, "'. Please check your code for infinite loops etc.!");
  }
  tWatchDogTask::Deactivate();
}

bool tThreadContainerThread::IsModuleInputInterface(core::tFrameworkElement& fe)
{
  if (IsInterface(fe))
  {
    uint port_count = 0;
    uint pure_input_port_count = 0;
    for (auto it = fe.ChildPortsBegin(); it != fe.ChildPortsEnd(); ++it)
    {
      if (data_ports::IsDataFlowType(it->GetDataType()))
      {
        port_count++;
        if (it->GetFlag(tFlag::ACCEPTS_DATA) && (!it->GetFlag(tFlag::EMITS_DATA)))
        {
          pure_input_port_count++;
        }
      }
    }
    return (2 * pure_input_port_count) >= port_count; // heuristic: min. 50% of ports are pure input ports
  }
  return false;
}

void tThreadContainerThread::MainLoopCallback()
{
  if (reschedule)
  {
    // TODO: this rescheduling implementation leads to unpredictable delays (scheduling could be performed by another thread)
    reschedule = false;
    {
      tLock lock(this->thread_container.GetStructureMutex());
      schedule.clear();
      rrlib::time::tTimestamp start_time = rrlib::time::Now();

      /*! Sets of tasks that need to be scheduled */
      std::set<tPeriodicFrameworkElementTask*> sense_tasks, control_tasks, initial_tasks, other_tasks;

      /*! Sense and control interfaces */
      std::set<core::tEdgeAggregator*> sense_interfaces, control_interfaces;

      // find tasks and classified interfaces
      for (auto it = thread_container.SubElementsBegin(true); it != thread_container.SubElementsEnd(); ++it)
      {
        if ((!it->IsReady()) || tExecutionControl::Find(*it)->GetAnnotated<core::tFrameworkElement>() != &thread_container)    // don't handle elements in nested thread containers
        {
          continue;
        }
        tPeriodicFrameworkElementTask* task = it->GetAnnotation<tPeriodicFrameworkElementTask>();
        if (task)
        {
          task->previous_tasks.clear();
          task->next_tasks.clear();
          task->task_classification = 0;
          if (task->IsSenseTask())
          {
            task->task_classification = eSENSE_TASK;
            sense_tasks.insert(task);
            sense_interfaces.insert(task->incoming.begin(), task->incoming.end());
            sense_interfaces.insert(task->outgoing.begin(), task->outgoing.end());
          }
          else if (task->IsControlTask())
          {
            task->task_classification = eCONTROL_TASK;
            control_tasks.insert(task);
            control_interfaces.insert(task->incoming.begin(), task->incoming.end());
            control_interfaces.insert(task->outgoing.begin(), task->outgoing.end());
          }
          else
          {
            other_tasks.insert(task);
          }
        }

        if (it->GetFlag(tFlag::INTERFACE))
        {
          if (it->GetFlag(tFlag::SENSOR_DATA))
          {
            sense_interfaces.insert(static_cast<core::tEdgeAggregator*>(&(*it)));
          }
          if (it->GetFlag(tFlag::CONTROLLER_DATA))
          {
            control_interfaces.insert(static_cast<core::tEdgeAggregator*>(&(*it)));
          }
        }
      }

      // classify tasks by flooding
      std::vector<core::tEdgeAggregator*> trace; // trace we're currently following
      {
        int flag_to_check = 0;
        std::function<void (tPeriodicFrameworkElementTask&)> function = [&](tPeriodicFrameworkElementTask & connected_task)
        {
          if ((connected_task.task_classification & (flag_to_check | eSENSE_TASK | eCONTROL_TASK)) == 0)
          {
            connected_task.task_classification |= flag_to_check;
            bool reverse = (flag_to_check == eSENSE_DEPENDENCY || flag_to_check == eCONTROL_DEPENDENCY);
            for (core::tEdgeAggregator * next : (reverse ? connected_task.incoming : connected_task.outgoing))
            {
              ForEachConnectedTask<IsSensorOrControllerInterface>(*next, trace, function, reverse);
            }
          }
        };

        for (core::tEdgeAggregator * interface : sense_interfaces)
        {
          flag_to_check = eSENSE_DEPENDENT;
          ForEachConnectedTask<IsSensorOrControllerInterface>(*interface, trace, function, false);
          flag_to_check = eSENSE_DEPENDENCY;
          ForEachConnectedTask<IsSensorOrControllerInterface>(*interface, trace, function, true);
        }
        for (core::tEdgeAggregator * interface : control_interfaces)
        {
          flag_to_check = eCONTROL_DEPENDENT;
          ForEachConnectedTask<IsSensorOrControllerInterface>(*interface, trace, function, false);
          flag_to_check = eCONTROL_DEPENDENCY;
          ForEachConnectedTask<IsSensorOrControllerInterface>(*interface, trace, function, true);
        }
      }

      std::set<tPeriodicFrameworkElementTask*> other_tasks_copy = other_tasks;
      for (tPeriodicFrameworkElementTask * other_task : other_tasks_copy)
      {
        bool sense_task = (other_task->task_classification & (eSENSE_DEPENDENCY | eSENSE_DEPENDENT)) == (eSENSE_DEPENDENCY | eSENSE_DEPENDENT);
        bool control_task = (other_task->task_classification & (eCONTROL_DEPENDENCY | eCONTROL_DEPENDENT)) == (eCONTROL_DEPENDENCY | eCONTROL_DEPENDENT);
        if (!(sense_task || control_task))
        {
          // max. two flags are possible - check all combinations
          if ((other_task->task_classification & (eSENSE_DEPENDENCY | eCONTROL_DEPENDENCY)) == (eSENSE_DEPENDENCY | eCONTROL_DEPENDENCY))
          {
            initial_tasks.insert(other_task);
            other_tasks.erase(other_task);
            continue;
          }
          if ((other_task->task_classification & (eSENSE_DEPENDENT | eCONTROL_DEPENDENT)) == (eSENSE_DEPENDENT | eCONTROL_DEPENDENT))
          {
            continue;
          }
          if ((other_task->task_classification & (eSENSE_DEPENDENCY | eCONTROL_DEPENDENT)) == (eSENSE_DEPENDENCY | eCONTROL_DEPENDENT))
          {
            sense_task = true;
          }
          if ((other_task->task_classification & (eSENSE_DEPENDENT | eCONTROL_DEPENDENCY)) == (eSENSE_DEPENDENT | eCONTROL_DEPENDENCY))
          {
            control_task = true;
          }
        }
        if (!(sense_task || control_task))
        {
          // max. one flag is possible
          sense_task = other_task->task_classification & (eSENSE_DEPENDENCY | eSENSE_DEPENDENT);
          control_task = other_task->task_classification & (eCONTROL_DEPENDENCY | eCONTROL_DEPENDENT);
        }

        if (sense_task || control_task)
        {
          other_tasks.erase(other_task);
          if (sense_task)
          {
            sense_tasks.insert(other_task);
          }
          if (control_task)
          {
            control_tasks.insert(other_task);
          }
        }
      }

      /*! temporary variable for trace backs */
      std::vector<tPeriodicFrameworkElementTask*> trace_back;

      // create task graphs for the four relevant sets of tasks and schedule them
      std::set<tPeriodicFrameworkElementTask*>* task_sets[4] = { &initial_tasks, &sense_tasks, &control_tasks, &other_tasks };
      for (size_t i = 0; i < 4; i++)
      {
        trace.clear();
        std::set<tPeriodicFrameworkElementTask*>& task_set = *task_sets[i];
        auto task = task_set.begin();
        std::function<void (tPeriodicFrameworkElementTask&)> function = [&](tPeriodicFrameworkElementTask & connected_task)
        {
          if (task_set.find(&connected_task) != task_set.end() &&
              std::find((*task)->next_tasks.begin(), (*task)->next_tasks.end(), &connected_task) == (*task)->next_tasks.end())
          {
            (*task)->next_tasks.push_back(&connected_task);
            connected_task.previous_tasks.push_back(*task);
          }
        };

        // create task graph
        for (; task != task_set.end(); ++task)
        {
          // trace outgoing connections to other elements in task set
          for (auto it = (*task)->outgoing.begin(); it < (*task)->outgoing.end(); ++it)
          {
            if (i == 1)
            {
              ForEachConnectedTask<IsControllerInterface>(**it, trace, function, false);
            }
            else if (i == 2)
            {
              ForEachConnectedTask<IsSensorInterface>(**it, trace, function, false);
            }
            else
            {
              ForEachConnectedTask<AlwaysFalse>(**it, trace, function, false);
            }
          }
        }

        task_set_first_index[i] = schedule.size();

        // now create schedule
        while (task_set.size() > 0)
        {
          // do we have a task without previous tasks?
          bool found = false;
          for (auto it = task_set.begin(); it != task_set.end(); ++it)
          {
            tPeriodicFrameworkElementTask* task = *it;
            if (task->previous_tasks.size() == 0)
            {
              schedule.push_back(task);
              task_set.erase(task);
              found = true;

              // delete from next tasks' previous task list
              for (auto next = task->next_tasks.begin(); next != task->next_tasks.end(); ++next)
              {
                (*next)->previous_tasks.erase(std::remove((*next)->previous_tasks.begin(), (*next)->previous_tasks.end(), task), (*next)->previous_tasks.end());
              }
              break;
            }
          }
          if (found)
          {
            continue;
          }

          // ok, we didn't find task to continue with... (loop)
          trace_back.clear();
          tPeriodicFrameworkElementTask* current = *task_set.begin();
          trace_back.push_back(current);
          while (true)
          {
            bool end = true;
            for (size_t i = 0u; i < current->previous_tasks.size(); i++)
            {
              tPeriodicFrameworkElementTask* prev = current->previous_tasks[i];
              if (std::find(trace_back.begin(), trace_back.end(), prev) == trace_back.end())
              {
                end = false;
                current = prev;
                trace_back.push_back(current);
                break;
              }
            }
            if (end)
            {
              FINROC_LOG_PRINT(WARNING, "Detected loop:\n", CreateLoopDebugOutput(trace_back), "\nBreaking it up at '", current->previous_tasks[0]->GetLogDescription(), "' -> '", current->GetLogDescription(), "' (The latter will be executed before the former)");
              schedule.push_back(current);
              task_set.erase(current);

              // delete from next tasks' previous task list
              for (auto next = current->next_tasks.begin(); next != current->next_tasks.end(); ++next)
              {
                (*next)->previous_tasks.erase(std::remove((*next)->previous_tasks.begin(), (*next)->previous_tasks.end(), current), (*next)->previous_tasks.end());
              }
              break;
            }
          }
        }
      }

      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "Created schedule in ", rrlib::time::ToIsoString(rrlib::time::Now() - start_time));
      for (size_t i = 0; i < schedule.size(); ++i)
      {
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "  ", i, ": ", schedule[i]->GetLogDescription());
      }
    }
  }

  // execute tasks
  SetDeadLine(rrlib::time::Now() + GetCycleTime() * 4 + std::chrono::seconds(4));

  if (execution_details.GetWrapped() == nullptr || execution_count == 0) // we skip profiling the first/initial execution
  {
    current_cycle_start_application_time = IsUsingApplicationTime() && this->IsAlive() ? tLoopThread::GetCurrentCycleStartTime() : rrlib::time::Now();

    execution_duration.Publish(GetLastCycleTime());
    for (size_t i = 0u; i < schedule.size(); i++)
    {
      current_task = schedule[i];
      //FINROC_LOG_PRINT(DEBUG_WARNING, "Executing ", current_task->GetLogDescription());
      current_task->task.ExecuteTask();
    }
    execution_count++;
  }
  else
  {
    data_ports::tPortDataPointer<std::vector<tTaskProfile>> details = execution_details.GetUnusedBuffer();
    details->resize(schedule.size() + 1);
    rrlib::time::tTimestamp start = rrlib::time::Now(true);
    current_cycle_start_application_time = start;

    for (size_t i = 0u; i < schedule.size(); i++)
    {
      current_task = schedule[i];
      rrlib::time::tTimestamp task_start = rrlib::time::Now(true);
      current_task->task.ExecuteTask();
      rrlib::time::tDuration task_duration = rrlib::time::Now(true) - task_start;

      // Update internal task statistics
      current_task->total_execution_duration += task_duration;
      current_task->execution_count++;
      current_task->max_execution_duration = std::max(task_duration, current_task->max_execution_duration);

      // Fill task profile to publish
      tTaskProfile& task_profile = (*details)[i + 1];
      task_profile.handle = current_task->GetAnnotated<core::tFrameworkElement>()->GetHandle();
      task_profile.last_execution_duration = task_duration;
      task_profile.max_execution_duration = current_task->max_execution_duration;
      task_profile.average_execution_duration = rrlib::time::tDuration(current_task->total_execution_duration.count() / current_task->execution_count);
      task_profile.total_execution_duration = current_task->total_execution_duration;
      task_profile.task_classification = tTaskClassification::OTHER;
    }

    // Set classification
    for (size_t i = task_set_first_index[1]; i < task_set_first_index[2]; i++)
    {
      (*details)[i + 1].task_classification = tTaskClassification::SENSE;  // +1, because first task is at index 1
    }
    for (size_t i = task_set_first_index[2]; i < task_set_first_index[3]; i++)
    {
      (*details)[i + 1].task_classification = tTaskClassification::CONTROL;
    }


    // Update thread statistics
    rrlib::time::tDuration duration = rrlib::time::Now(true) - start;
    this->total_execution_duration += duration;
    this->execution_count++;
    this->max_execution_duration = std::max(duration, this->max_execution_duration);

    // Fill thread profile to publish
    tTaskProfile& profile = (*details)[0];
    profile.handle = thread_container.GetHandle();
    profile.last_execution_duration = duration;
    profile.max_execution_duration = this->max_execution_duration;
    assert(execution_count > 1);
    profile.average_execution_duration = rrlib::time::tDuration(this->total_execution_duration.count() / (this->execution_count - 1)); // we did not include initial execution for profile statistics
    profile.total_execution_duration = this->total_execution_duration;

    // Publish profiling information
    for (size_t i = 0u; i < schedule.size(); i++)
    {
      if (schedule[i]->execution_duration.GetWrapped())
      {
        schedule[i]->execution_duration.Publish((*details)[i + 1].last_execution_duration);
      }
    }
    execution_duration.Publish(duration);
    execution_details.Publish(details);
  }

  tWatchDogTask::Deactivate();
}

void tThreadContainerThread::OnConnectorChange(core::tRuntimeListener::tEvent change_type, core::tConnector& connector)
{
  if (connector.Source().IsChildOf(this->thread_container) && connector.Destination().IsChildOf(this->thread_container))
  {
    reschedule = true;
  }
}

void tThreadContainerThread::OnFrameworkElementChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element)
{
  if (element.GetAnnotation<tPeriodicFrameworkElementTask>() && element.IsChildOf(this->thread_container, true))
  {
    reschedule = true;
  }
}

void tThreadContainerThread::OnUriConnectorChange(core::tRuntimeListener::tEvent change_type, core::tUriConnector& connector)
{
}

void tThreadContainerThread::Run()
{
  tLoopThread::Run();
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

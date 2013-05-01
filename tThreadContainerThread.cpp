//
// You received this file as part of Finroc
// A Framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
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

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

tThreadContainerThread::tThreadContainerThread(core::tFrameworkElement& thread_container, rrlib::time::tDuration default_cycle_time, bool warn_on_cycle_time_exceed, data_ports::tOutputPort<rrlib::time::tDuration> last_cycle_execution_time) :
  tLoopThread(default_cycle_time, true, warn_on_cycle_time_exceed),
  tWatchDogTask(true),
  thread_container(thread_container),
  reschedule(true),
  schedule(),
  tasks(),
  non_sensor_tasks(),
  trace(),
  trace_back(),
  last_cycle_execution_time(last_cycle_execution_time),
  current_task(NULL)
{
  this->SetName("ThreadContainer " + thread_container.GetName());
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
    FINROC_LOG_PRINT(ERROR, "Got stuck executing task associated with '", task->incoming[0]->GetQualifiedName(), "'. Please check your code for infinite loops etc.!");
  }
  tWatchDogTask::Deactivate();
}

void tThreadContainerThread::MainLoopCallback()
{
  if (reschedule)
  {
    reschedule = false;
    {
      tLock lock(this->thread_container.GetStructureMutex());

      // find tasks
      tasks.clear();
      non_sensor_tasks.clear();
      schedule.clear();

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
          if (task->IsSenseTask())
          {
            tasks.push_back(task);
          }
          else
          {
            non_sensor_tasks.push_back(task);
          }
        }
      }

      tasks.insert(tasks.end(), non_sensor_tasks.begin(), non_sensor_tasks.end());

      // create task graph
      for (auto task = tasks.begin(); task != tasks.end(); ++task)
      {
        // trace outgoing connections
        for (auto it = (*task)->outgoing.begin(); it < (*task)->outgoing.end(); ++it)
        {
          TraceOutgoing(**task, **it);
        }
      }

      // now create schedule
      while (tasks.size() > 0)
      {
        // do we have task without previous tasks?
        bool found = false;
        for (auto it = tasks.begin(); it != tasks.end(); ++it)
        {
          tPeriodicFrameworkElementTask* task = *it;
          if (task->previous_tasks.size() == 0)
          {
            schedule.push_back(task);
            tasks.erase(std::remove(tasks.begin(), tasks.end(), task), tasks.end());
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

        // ok, we didn't find module to continue with... (loop)
        FINROC_LOG_PRINT(WARNING, "Detected loop: doing traceback");
        trace_back.clear();
        tPeriodicFrameworkElementTask* current = tasks[0];
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
            FINROC_LOG_PRINT(WARNING, "Choosing ", current->incoming[0]->GetQualifiedName(), " as next element");
            schedule.push_back(current);
            tasks.erase(std::remove(tasks.begin(), tasks.end(), current), tasks.end());

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
  }

  // execute tasks
  last_cycle_execution_time.Publish(GetLastCycleTime());

  SetDeadLine(rrlib::time::Now() + GetCycleTime() * 4 + std::chrono::seconds(1));

  for (size_t i = 0u; i < schedule.size(); i++)
  {
    current_task = schedule[i];
    current_task->task.ExecuteTask();
  }

  tWatchDogTask::Deactivate();
}

void tThreadContainerThread::OnEdgeChange(core::tRuntimeListener::tEvent change_type, core::tAbstractPort& source, core::tAbstractPort& target)
{
  if (source.IsChildOf(this->thread_container) && target.IsChildOf(this->thread_container))
  {
    reschedule = true;
  }
}

void tThreadContainerThread::OnFrameworkElementChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element)
{
  if (element.IsChildOf(this->thread_container, true))
  {
    reschedule = true;
  }
}

void tThreadContainerThread::Run()
{
  this->thread_container.GetRuntime().AddListener(*this);
  tLoopThread::Run();
}

void tThreadContainerThread::StopThread()
{
  tLock lock(this->thread_container.GetStructureMutex());
  this->thread_container.GetRuntime().RemoveListener(*this);
  tLoopThread::StopThread();
}

void tThreadContainerThread::TraceOutgoing(tPeriodicFrameworkElementTask& task, core::tEdgeAggregator& outgoing)
{
  // add to trace stack
  trace.push_back(&outgoing);

  for (auto it = outgoing.OutgoingConnectionsBegin(); it != outgoing.OutgoingConnectionsEnd(); ++it)
  {
    core::tAggregatedEdge& aggregated_edge = **it;
    core::tEdgeAggregator& dest = aggregated_edge.destination;

    if (std::find(trace.begin(), trace.end(), &dest) == trace.end())
    {
      // ok, have we reached another task?
      tPeriodicFrameworkElementTask* task2 = dest.GetAnnotation<tPeriodicFrameworkElementTask>();
      if (task2 == NULL && IsInterface(dest))
      {
        task2 = dest.GetParent()->GetAnnotation<tPeriodicFrameworkElementTask>();
      }
      if (task2)
      {
        if (std::find(task.next_tasks.begin(), task.next_tasks.end(), task2) == task.next_tasks.end())
        {
          task.next_tasks.push_back(task2);
          task2->previous_tasks.push_back(&task);
        }
        continue;
      }

      // continue from this edge aggregator
      if (dest.OutgoingConnectionsBegin() != dest.OutgoingConnectionsEnd()) // not empty?
      {
        TraceOutgoing(task, dest);
      }
      else if (IsInterface(dest)) // TODO: check whether this breaks sense-control-groups
      {
        core::tFrameworkElement* parent = dest.GetParent();
        if (parent->GetFlag(tFlag::EDGE_AGGREGATOR))
        {
          core::tEdgeAggregator* ea = static_cast<core::tEdgeAggregator*>(parent);
          if (std::find(trace.begin(), trace.end(), ea) == trace.end())
          {
            TraceOutgoing(task, *ea);
          }
        }
        for (auto it = parent->ChildrenBegin(); it != parent->ChildrenEnd(); ++it)
        {
          if (it->GetFlag(tFlag::READY) && it->GetFlag(tFlag::EDGE_AGGREGATOR) && it->GetFlag(tFlag::INTERFACE))
          {
            core::tEdgeAggregator& ea = static_cast<core::tEdgeAggregator&>(*it);
            if (std::find(trace.begin(), trace.end(), &ea) == trace.end())
            {
              TraceOutgoing(task, ea);
            }
          }
        }
      }
    }
  }

  // remove from trace stack
  assert(trace[trace.size() - 1] == &outgoing);
  trace.pop_back();
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

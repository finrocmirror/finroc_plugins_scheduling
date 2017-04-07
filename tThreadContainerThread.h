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
/*!\file    plugins/scheduling/tThreadContainerThread.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 * \brief   Contains tThreadContainerThread
 *
 * \b tThreadContainerThread
 *
 * Thread that executes tasks inside thread container.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__scheduling__tThreadContainerThread_h__
#define __plugins__scheduling__tThreadContainerThread_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/thread/tLoopThread.h"
#include "rrlib/watchdog/tWatchDogTask.h"
#include "core/tRuntimeListener.h"
#include "plugins/data_ports/tOutputPort.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/scheduling/tTaskProfile.h"

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
struct tPeriodicFrameworkElementTask;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Thread container thread
/*!
 * Thread that executes tasks inside thread container.
 */
class tThreadContainerThread : public rrlib::thread::tLoopThread, public core::tRuntimeListener, public rrlib::watchdog::tWatchDogTask
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tThreadContainerThread(core::tFrameworkElement& thread_container, rrlib::time::tDuration default_cycle_time,
                         bool warn_on_cycle_time_exceed, data_ports::tOutputPort<rrlib::time::tDuration> execution_duration,
                         data_ports::tOutputPort<std::vector<tTaskProfile>> execution_details);

  virtual ~tThreadContainerThread();

  /*!
   * \return Returns pointer to current thread if it is a tThreadContainerThread - NULL otherwise
   */
  static tThreadContainerThread* CurrentThread()
  {
#ifndef RRLIB_SINGLE_THREADED
    rrlib::thread::tThread& thread = rrlib::thread::tThread::CurrentThread();
    return (typeid(thread) == typeid(tThreadContainerThread)) ? static_cast<tThreadContainerThread*>(&thread) : NULL;
#else
    return single_thread_container;
#endif
  }

  /*!
   * (TODO: Add method to structure::tModuleBase to access this more conveniently?)
   *
   * \return Start time of current cycle (unlike the base class, always returns time in 'application time')
   */
  inline rrlib::time::tTimestamp GetCurrentCycleStartTime()
  {
    return current_cycle_start_application_time;
  }

  /*!
   * \return Shared Pointer to thread container thread
   */
  std::shared_ptr<tThreadContainerThread> GetSharedPtr()
  {
    return std::static_pointer_cast<tThreadContainerThread>(tThread::GetSharedPtr());
  }

  virtual void MainLoopCallback() override;

  virtual void Run() override;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Thread container that thread belongs to */
  core::tFrameworkElement& thread_container;

  /*! true, when thread needs to make a new schedule before next run */
  std::atomic<bool> reschedule;

  /*!
   * simple schedule: Tasks will be executed in specified order
   * There are four sets of tasks: [initial tasks, sense tasks, control tasks, other tasks]
   */
  std::vector<tPeriodicFrameworkElementTask*> schedule;

  /*! Indices where the different sets of tasks start in the schedule */
  size_t task_set_first_index[4];

  /*! Port to publish time spent in last call to MainLoopCallback() */
  data_ports::tOutputPort<rrlib::time::tDuration> execution_duration;

  /*!
   * Port to publish details on execution (port is only created if profiling is enabled)
   * The first element contains the profile the whole thread container.
   * The other elements contain the profile the executed tasks - in the order of their execution
   */
  data_ports::tOutputPort<std::vector<tTaskProfile>> execution_details;

  /*! Total execution duration of thread */
  rrlib::time::tDuration total_execution_duration;

  /*! Maximum execution duration of schedule */
  rrlib::time::tDuration max_execution_duration;

  /*! Number of times that schedule was executed */
  int64_t execution_count;

  /*!
   * Thread sets this to the task it is currently executing (for error message, should it get stuck)
   * NULL if not executing any task
   */
  tPeriodicFrameworkElementTask* current_task;

  /*! Start time of current control cycle in application time */
  rrlib::time::tTimestamp current_cycle_start_application_time;

  /*! Contains pointer to the only thread container in single threaded mode */
  static tThreadContainerThread* single_thread_container;

  /*!
   * Helper function for debug output.
   *
   * \param task_list List of tasks
   * \return String with fully-qualified names of each attached framework element of list elements in a new line
   */
  std::string CreateLoopDebugOutput(const std::vector<tPeriodicFrameworkElementTask*>& task_list);

  /*!
   * Applies function to each task connected with specified edge aggregator.
   * Traces and follows all connections as long as elements are managed by this thread container (depth-first search).
   * The ABORT_PREDICATE (binary predicate) is called on each edge aggregator encountered on the way.
   * If it is false, the path beyond is not followed.
   *
   * Note that the function may be called multiple times with the same edge aggregator.
   *
   * (implementation note: can be implemented in .cpp file, since it is only called from there)
   *
   * \param origin Element to start with (tEdgeAggregator or tAbstractPort)
   * \param trace Path in data flow graph we're currently checking. Maintained, to avoid that follow cycles again and again.
   *              Is not cleared by this function. Elements are merely pushed and popped.
   * \param function Function to call for each on each connected task. It needs one parameter:
   *                 'tPeriodicFrameworkElementTask& connected_task'
   * \param trace_reverse Traces outgoing connections if false - or in reverse direction of data flow graph if true.
   */
  template <bool (ABORT_PREDICATE)(core::tEdgeAggregator&), typename TFunction>
  void ForEachConnectedTask(core::tEdgeAggregator& origin, std::vector<core::tEdgeAggregator*>& trace, TFunction& function, bool trace_reverse);
  template <bool (ABORT_PREDICATE)(core::tEdgeAggregator&), typename TFunction>
  void ForEachConnectedTask(core::tAbstractPort& origin, std::vector<core::tEdgeAggregator*>& trace, TFunction& function, bool trace_reverse);

  /*!
   * \param fe Framework element
   * \return Is framework element an input interface of a module?
   */
  static bool IsModuleInputInterface(core::tFrameworkElement& fe);

  virtual void HandleWatchdogAlert() override;

  virtual void OnConnectorChange(tEvent change_type, core::tConnector& connector) override;
  virtual void OnFrameworkElementChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element) override;
  virtual void OnUriConnectorChange(tEvent change_type, core::tUriConnector& connector);
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif

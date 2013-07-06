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
  typedef core::tFrameworkElement::tFlag tFlag;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tThreadContainerThread(core::tFrameworkElement& thread_container, rrlib::time::tDuration default_cycle_time,
                         bool warn_on_cycle_time_exceed, data_ports::tOutputPort<rrlib::time::tDuration> last_cycle_execution_time);

  /*!
   * \return Returns pointer to current thread if it is a tThreadContainerThread - NULL otherwise
   */
  static tThreadContainerThread* CurrentThread()
  {
    rrlib::thread::tThread& thread = rrlib::thread::tThread::CurrentThread();
    return (typeid(thread) == typeid(tThreadContainerThread)) ? static_cast<tThreadContainerThread*>(&thread) : NULL;
  }

  /*!
   * \return Shared Pointer to thread container thread
   */
  std::shared_ptr<tThreadContainerThread> GetSharedPtr()
  {
    return std::static_pointer_cast<tThreadContainerThread>(tThread::GetSharedPtr());
  }

  /*!
   * \param fe Framework element
   * \return Is framework element an interface?
   */
  inline bool IsInterface(core::tFrameworkElement& fe)
  {
    return fe.GetFlag(tFlag::EDGE_AGGREGATOR) || fe.GetFlag(tFlag::INTERFACE);
  }

  virtual void MainLoopCallback(); // TODO: mark override with gcc 4.7

  virtual void OnEdgeChange(core::tRuntimeListener::tEvent change_type, core::tAbstractPort& source, core::tAbstractPort& target); // TODO: mark override with gcc 4.7

  virtual void OnFrameworkElementChange(core::tRuntimeListener::tEvent change_type, core::tFrameworkElement& element); // TODO: mark override with gcc 4.7

  virtual void Run(); // TODO: mark override with gcc 4.7

  virtual void StopThread(); // TODO: mark override with gcc 4.7

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Thread container that thread belongs to */
  core::tFrameworkElement& thread_container;

  /*! true, when thread needs to make a new schedule before next run */
  std::atomic<bool> reschedule;

  /*! simple schedule: Tasks will be executed in specified order */
  std::vector<tPeriodicFrameworkElementTask*> schedule;

  /*! temporary list of tasks that need to be scheduled */
  std::vector<tPeriodicFrameworkElementTask*> tasks;

  /*! temporary list of tasks that need to be scheduled - which are not sensor tasks */
  std::vector<tPeriodicFrameworkElementTask*> non_sensor_tasks;

  /*! temporary variable for scheduling algorithm: trace we're currently following */
  std::vector<core::tEdgeAggregator*> trace;

  /*! temporary variable: trace back */
  std::vector<tPeriodicFrameworkElementTask*> trace_back;

  /*! Port to publish time spent in last call to MainLoopCallback() */
  data_ports::tOutputPort<rrlib::time::tDuration> last_cycle_execution_time;

  /*!
   * Thread sets this to the task it is currently executing (for error message, should it get stuck)
   * NULL if not executing any task
   */
  tPeriodicFrameworkElementTask* current_task;


  virtual void HandleWatchdogAlert(); // TODO: mark override with gcc 4.7

  /*!
   * Trace outgoing connection
   *
   * \param task Task we're tracing from
   * \param outgoing edge aggregator with outgoing connections to follow
   */
  void TraceOutgoing(tPeriodicFrameworkElementTask& task, core::tEdgeAggregator& outgoing);

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif

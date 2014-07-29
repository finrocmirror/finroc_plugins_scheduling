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
/*!\file    plugins/scheduling/tThreadContainerElement.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 * \brief   Contains tThreadContainerElement
 *
 * \b tThreadContainerElement
 *
 * Contains thread that executes ordered periodic tasks of all children.
 * Execution is performed in the order of the graph.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__scheduling__tThreadContainerElement_h__
#define __plugins__scheduling__tThreadContainerElement_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/parameters/tStaticParameter.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/scheduling/tStartAndPausable.h"
#include "plugins/scheduling/tThreadContainerThread.h"

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
// Class declaration
//----------------------------------------------------------------------
//! Framework element containing thread for execution
/*!
 * Contains thread that executes ordered periodic tasks of all children.
 * Execution is performed in the order of the graph.
 */
template <typename BASE>
class tThreadContainerElement : public BASE, public tStartAndPausable
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*! Should this container contain a real-time thread? */
  parameters::tStaticParameter<bool> rt_thread;

  /*! Warn on cycle time exceed */
  parameters::tStaticParameter<bool> warn_on_cycle_time_exceed;

  /*! Port to publish time spent in last call to MainLoopCallback() */
  data_ports::tOutputPort<rrlib::time::tDuration> execution_duration;

  /*!
   * Port to publish details on execution (port is only created if profiling is enabled)
   * The first element contains the profile the whole thread container.
   * The other elements contain the profile the executed tasks - in the order of their execution
   */
  data_ports::tOutputPort<std::vector<tTaskProfile>> execution_details;


  /*!
   * All constructor parameters are forwarded to class BASE (usually parent, name, flags)
   */
  template <typename ... ARGS>
  tThreadContainerElement(ARGS && ... args);

  virtual ~tThreadContainerElement();

  /*!
   * Execute one cycle manually.
   * This can be handy for test programs (e.g. for accelerating them)
   * and is not meant for "normal" applications.
   *
   * StartExecution() must not be called when using this method.
   */
  void ExecuteCycle();

  /*!
   * \return Cycle time in milliseconds
   */
  inline rrlib::time::tDuration GetCycleTime()
  {
    return cycle_time.Get();
  }

  virtual bool IsExecuting() override
  {
    return thread.get();
  }

  /*!
   * Block until thread has stopped
   */
  void JoinThread();

  virtual void PauseExecution() override
  {
    StopThread();
    JoinThread();
  }

  /*!
   * \param period Cycle time
   */
  inline void SetCycleTime(const rrlib::time::tDuration& period)
  {
    cycle_time.Set(period);
  }

  /*!
   * \param period Cycle time in milliseconds
   */
  inline void SetCycleTime(int64_t period)
  {
    SetCycleTime(std::chrono::milliseconds(period));
  }

  virtual void StartExecution() override;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Thread cycle time */
  parameters::tStaticParameter<rrlib::time::tDuration> cycle_time;

  /*! Thread - while program is running - in pause mode null */
  std::shared_ptr<tThreadContainerThread> thread;

  /*! Mutex for operations on thread container */
  rrlib::thread::tOrderedMutex mutex;

  /*!
   * Stop thread in thread container (does not block - call join thread to block until thread has terminated)
   */
  void StopThread();
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "plugins/scheduling/tThreadContainerElement.hpp"

#endif

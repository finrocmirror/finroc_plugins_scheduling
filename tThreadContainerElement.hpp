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
/*!\file    plugins/scheduling/tThreadContainerElement.hpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tLockOrderLevel.h"
#include "core/port/tPortGroup.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/scheduling/scheduling.h"
#include "plugins/scheduling/tExecutionControl.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

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

template <typename BASE>
template <typename ... ARGS>
tThreadContainerElement<BASE>::tThreadContainerElement(ARGS && ... args) :
  BASE(args...),
  rt_thread("Realtime Thread", this, false),
  warn_on_cycle_time_exceed("Warn on cycle time exceed", this, true),
  execution_duration("Execution Duration", new core::tPortGroup(this, "Profiling", core::tFrameworkElementFlag::INTERFACE, core::tFrameworkElement::tFlag::EMITS_DATA | core::tFrameworkElement::tFlag::OUTPUT_PORT)),
  execution_details("Details", execution_duration.GetParent(), IsProfilingEnabled() ? BASE::tFlag::PORT : BASE::tFlag::DELETED),
  cycle_time("Cycle Time", this, std::chrono::milliseconds(40), data_ports::tBounds<rrlib::time::tDuration>(rrlib::time::tDuration::zero(), std::chrono::seconds(60))),
  thread(),
  mutex("tThreadContainerElement", static_cast<int>(core::tLockOrderLevel::RUNTIME_REGISTER) - 1)
{
  this->AddAnnotation(*new tExecutionControl(*this));
}

template <typename BASE>
tThreadContainerElement<BASE>::~tThreadContainerElement()
{
  if (thread.get())
  {
    StopThread();
    JoinThread();
  }
}

template <typename BASE>
void tThreadContainerElement<BASE>::ExecuteCycle()
{
  if (!thread.get())
  {
    rrlib::thread::tLock l(mutex);
    tThreadContainerThread* thread_tmp = new tThreadContainerThread(*this, cycle_time.Get(), warn_on_cycle_time_exceed.Get(), execution_duration, execution_details);
    thread_tmp->SetAutoDelete();
    thread = std::static_pointer_cast<tThreadContainerThread>(thread_tmp->GetSharedPtr());
    thread->StopThread();
    thread->Start();
    thread->Join();
  }
  else
  {
    assert(!thread->IsAlive());
  }
  thread->MainLoopCallback();
}

template <typename BASE>
void tThreadContainerElement<BASE>::JoinThread()
{
  rrlib::thread::tLock l(mutex);
  if (thread.get() != NULL)
  {
    thread->Join();
    thread.reset();
  }
}

template <typename BASE>
void tThreadContainerElement<BASE>::StartExecution()
{
  rrlib::thread::tLock l(mutex);
  if (thread)
  {
    FINROC_LOG_PRINT(WARNING, "Thread is already executing.");
    return;
  }
  tThreadContainerThread* thread_tmp = new tThreadContainerThread(*this, cycle_time.Get(), warn_on_cycle_time_exceed.Get(), execution_duration, execution_details);
  thread_tmp->SetAutoDelete();
  thread = std::static_pointer_cast<tThreadContainerThread>(thread_tmp->GetSharedPtr());
  if (rt_thread.Get())
  {
    thread_tmp->SetRealtime();
  }
  l.Unlock();
  thread->Start();
}

template <typename BASE>
void tThreadContainerElement<BASE>::StopThread()
{
  rrlib::thread::tLock l(mutex);
  if (thread)
  {
    thread->StopThread();
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

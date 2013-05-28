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

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
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
  cycle_time("Cycle Time", this, std::chrono::milliseconds(40), data_ports::tBounds<rrlib::time::tDuration>(rrlib::time::tDuration::zero(), std::chrono::seconds(60))),
  warn_on_cycle_time_exceed("Warn on cycle time exceed", this, true),
  thread(),
  last_cycle_execution_time("Last Cycle execution time", this)
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
  tThreadContainerThread* thread_tmp = new tThreadContainerThread(*this, cycle_time.Get(), warn_on_cycle_time_exceed.Get(), last_cycle_execution_time);
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

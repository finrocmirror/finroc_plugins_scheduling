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
/*!\file    plugins/scheduling/tPeriodicFrameworkElementTask.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 */
//----------------------------------------------------------------------
#include "plugins/scheduling/tPeriodicFrameworkElementTask.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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

tPeriodicFrameworkElementTask::tPeriodicFrameworkElementTask(core::tEdgeAggregator* incoming_ports, core::tEdgeAggregator* outgoing_ports, rrlib::thread::tTask& task) :
  task(task),
  incoming(),
  outgoing(),
  previous_tasks(),
  next_tasks()
{
  if (incoming_ports)
  {
    incoming.push_back(incoming_ports);
  }
  if (outgoing_ports)
  {
    outgoing.push_back(outgoing_ports);
  }
}

tPeriodicFrameworkElementTask::tPeriodicFrameworkElementTask(const std::vector<core::tEdgeAggregator*>& incoming_ports, const std::vector<core::tEdgeAggregator*>& outgoing_ports, rrlib::thread::tTask& task):
  task(task),
  incoming(incoming_ports),
  outgoing(outgoing_ports),
  previous_tasks(),
  next_tasks()
{
}

bool tPeriodicFrameworkElementTask::IsSenseTask()
{
  for (auto it = outgoing.begin(); it < outgoing.end(); it++)
  {
    if ((*it)->GetFlag(core::tFrameworkElement::tFlag::SENSOR_DATA))
    {
      return true;
    }
  }
  for (auto it = incoming.begin(); it < incoming.end(); it++)
  {
    if ((*it)->GetFlag(core::tFrameworkElement::tFlag::SENSOR_DATA))
    {
      return true;
    }
  }
  return false;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

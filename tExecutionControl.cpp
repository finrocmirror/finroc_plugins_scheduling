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
/*!\file    plugins/scheduling/tExecutionControl.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 */
//----------------------------------------------------------------------
#include "plugins/scheduling/tExecutionControl.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/rtti/rtti.h"

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

tExecutionControl::tExecutionControl(tStartAndPausable& implementation) :
  implementation(implementation)
{
}

tExecutionControl* tExecutionControl::Find(core::tFrameworkElement& fe)
{
  return FindParentWithAnnotation<tExecutionControl>(fe);
}

void tExecutionControl::FindAll(std::vector<tExecutionControl*>& result, core::tFrameworkElement& fe)
{
  if (fe.IsReady())
  {
    for (auto it = fe.SubElementsBegin(true); it != fe.SubElementsEnd(); ++it)
    {
      tExecutionControl* ec = it->GetAnnotation<tExecutionControl>();
      if (ec)
      {
        result.push_back(ec);
      }
    }
  }
}

void tExecutionControl::PauseAll(core::tFrameworkElement& fe)
{
  std::vector<tExecutionControl*> ecs;
  FindAll(ecs, fe);
  for (auto it = ecs.begin(); it < ecs.end(); it++)
  {
    if ((*it)->IsRunning())
    {
      (*it)->Pause();
    }
  }
}

void tExecutionControl::StartAll(core::tFrameworkElement& fe)
{
  std::vector<tExecutionControl*> ecs;
  FindAll(ecs, fe);
  for (auto it = ecs.begin(); it < ecs.end(); it++)
  {
    if (!(*it)->IsRunning())
    {
      (*it)->Start();
    }
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

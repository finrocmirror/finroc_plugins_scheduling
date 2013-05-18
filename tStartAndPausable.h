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
/*!\file    plugins/scheduling/tStartAndPausable.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 * \brief   Contains tStartAndPausable
 *
 * \b tStartAndPausable
 *
 * Interface for objects whose execution can be started and stopped
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__scheduling__tStartAndPausable_h__
#define __plugins__scheduling__tStartAndPausable_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/tNoncopyable.h"

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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Object whose execution can be started and paused
/*!
 * Interface for objects whose execution can be started and paused
 */
class tStartAndPausable : private rrlib::util::tNoncopyable
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * \return Whether element currently executing
   */
  virtual bool IsExecuting() = 0;

  /*!
   * Stops execution
   */
  virtual void PauseExecution() = 0;

  /*!
   * Starts execution
   */
  virtual void StartExecution() = 0;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif

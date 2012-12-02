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
/*!\file    plugins/scheduling/tExecutionControl.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 * \brief   Contains tExecutionControl
 *
 * \b tExecutionControl
 *
 * Annotation for framework elements that can be started and paused (e.g. via finstruct)
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__scheduling__tExecutionControl_h__
#define __plugins__scheduling__tExecutionControl_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tFrameworkElement.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/scheduling/tStartAndPausable.h"

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
//! Execution control interface
/*!
 * Annotation for framework elements that can be started and paused (e.g. via finstruct)
 */
class tExecutionControl : public core::tAnnotation
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tExecutionControl(tStartAndPausable& implementation);


  /*!
   * Find StartAndPausable that is responsible for executing specified object
   *
   * \param fe Element
   * \return StartAndPausable
   */
  static tExecutionControl* Find(core::tFrameworkElement& fe);

  /*!
   * Returns all execution controls below including specified element
   *
   * \param result Result buffer for list of execution controls (controls are added to list)
   * \param elementHandle Framework element that is root of subtree to search for execution controls
   */
  static void FindAll(std::vector<tExecutionControl*>& result, core::tFrameworkElement& fe);

  /*!
   * \return Is currently executing?
   */
  inline bool IsRunning()
  {
    return implementation.IsExecuting();
  }

  /*!
   * Stop/Pause execution
   */
  inline void Pause()
  {
    implementation.PauseExecution();
  }

  /*!
   * Pauses all execution controls below and possibly attached to specified element
   *
   * \param fe Framework element that is root of subtree to search for execution controls
   */
  static void PauseAll(core::tFrameworkElement& fe);

  /*!
   * Start/Resume execution
   */
  inline void Start()
  {
    implementation.StartExecution();
  }

  /*!
   * Starts all execution controls below and possibly attached to specified element
   *
   * \param fe Framework element that is root of subtree to search for execution controls
   */
  static void StartAll(core::tFrameworkElement& fe);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Wrapped StartAndPausable */
  tStartAndPausable& implementation;
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif

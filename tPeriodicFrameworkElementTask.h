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
/*!\file    plugins/scheduling/tPeriodicFrameworkElementTask.h
 *
 * \author  Max Reichardt
 *
 * \date    2012-12-02
 *
 * \brief   Contains tPeriodicFrameworkElementTask
 *
 * \b tPeriodicFrameworkElementTask
 *
 * This represents a periodic task on the annotated Framework element
 * Such tasks are executed by a ThreadContainer - in the order of the
 * data flow graph.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__scheduling__tPeriodicFrameworkElementTask_h__
#define __plugins__scheduling__tPeriodicFrameworkElementTask_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/thread/tTask.h"
#include "core/port/tEdgeAggregator.h"

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
//! Periodic task attached to framework element
/*!
 * This represents a periodic task on the annotated Framework element
 * Such tasks are executed by a ThreadContainer - in the order of the
 * data flow graph.
 */
struct tPeriodicFrameworkElementTask : public core::tAnnotation
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * \param incoming_ports Element containing incoming ports (relevant for execution order)
   * \param outgoing_ports Element containing outgoing ports (relevant for execution order)
   * \param task Task to execute
   */
  tPeriodicFrameworkElementTask(core::tEdgeAggregator& incoming_ports, core::tEdgeAggregator& outgoing_ports, rrlib::thread::tTask& task);

  /*!
   * \param incoming_ports Elements containing incoming ports (relevant for execution order)
   * \param outgoing_ports Elements containing outgoing ports (relevant for execution order)
   * \param task Task to execute
   */
  tPeriodicFrameworkElementTask(const std::vector<core::tEdgeAggregator*>& incoming_ports, const std::vector<core::tEdgeAggregator*>& outgoing_ports, rrlib::thread::tTask& task);

  /*!
   * \return Is this a sensor task?
   */
  bool IsSenseTask();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  friend class tThreadContainerThread;

  /*! Task to execute */
  rrlib::thread::tTask& task;

  /*! Element containing incoming ports (relevant for execution order) */
  std::vector<core::tEdgeAggregator*> incoming;

  /*! Element containing outgoing ports (relevant for execution order) */
  std::vector<core::tEdgeAggregator*> outgoing;

  /*! Tasks to execute before this one (updated during scheduling) */
  std::vector<tPeriodicFrameworkElementTask*> previous_tasks;

  /*! Tasks to execute after this one (updated during scheduling) */
  std::vector<tPeriodicFrameworkElementTask*> next_tasks;
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
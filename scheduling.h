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
/*!\file    plugins/scheduling/scheduling.h
 *
 * \author  Max Reichardt
 *
 * \date    2014-04-17
 *
 * Definitions and configuration functions for scheduling plugin
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__scheduling__scheduling_h__
#define __plugins__scheduling__scheduling_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
// Function declarations
//----------------------------------------------------------------------

/*!
 * \return True if profiling is enabled (false by default)
 */
bool IsProfilingEnabled();

/*!
 * Sets whether profiling should be enabled.
 * This creates additional ports containing information about execution
 * of tasks.
 * Profiling is disabled by default.
 * This must be set, before tasks (and framework elements) are created.
 *
 * \param Whether to enable profiling
 */
void SetProfilingEnabled(bool enabled);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif

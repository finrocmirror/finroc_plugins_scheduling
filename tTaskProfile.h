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
/*!\file    plugins/scheduling/tTaskProfile.h
 *
 * \author  Max Reichardt
 *
 * \date    2014-04-17
 *
 * \brief   Contains tTaskProfile
 *
 * \b tTaskProfile
 *
 * Profile of one (periodic) task.
 * A thread container creates such profiles for the executed tasks if profiling is enabled.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__scheduling__tTaskProfile_h__
#define __plugins__scheduling__tTaskProfile_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/serialization/serialization.h"
#include "core/tFrameworkElement.h"

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

/*! Enum to specify which kind of task a task profile is associated to */
enum class tTaskClassification
{
  SENSE,
  CONTROL,
  OTHER
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Task profile
/*!
 * Profile of one (periodic) task.
 * A thread container creates such profiles for the executed tasks if profiling is enabled.
 */
struct tTaskProfile
{
  /*! Last execution duration */
  rrlib::time::tDuration last_execution_duration;

  /*! Maximum execution duration (excluding first/initial execution) */
  rrlib::time::tDuration max_execution_duration;

  /*! Average execution duration */
  rrlib::time::tDuration average_execution_duration;

  /*! Total execution duration */
  rrlib::time::tDuration total_execution_duration;

  /*! Handle of framework element associated with task */
  core::tFrameworkElement::tHandle handle;

  /*! Specifies which kind of task a task profile is associated to (used e.g. as hint for finstruct) */
  tTaskClassification task_classification;

  tTaskProfile() :
    last_execution_duration(0),
    max_execution_duration(0),
    average_execution_duration(0),
    total_execution_duration(0),
    handle(0),
    task_classification(tTaskClassification::OTHER)
  {}

};

inline rrlib::serialization::tOutputStream &operator << (rrlib::serialization::tOutputStream &stream, const tTaskProfile &profile)
{
  stream << profile.last_execution_duration << profile.max_execution_duration << profile.average_execution_duration
         << profile.total_execution_duration << profile.handle << profile.task_classification;
  return stream;
}

inline rrlib::serialization::tInputStream &operator >> (rrlib::serialization::tInputStream &stream, tTaskProfile &profile)
{
  stream >> profile.last_execution_duration >> profile.max_execution_duration >> profile.average_execution_duration
         >> profile.total_execution_duration >> profile.handle >> profile.task_classification;
  return stream;
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif

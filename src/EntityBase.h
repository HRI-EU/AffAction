/******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH
  Carl-Legien Str. 30
  63073 Offenbach/Main
  Germany

  UNPUBLISHED PROPRIETARY MATERIAL.
  ALL RIGHTS RESERVED

******************************************************************************/

#ifndef RCS_ENTITYBASE_H
#define RCS_ENTITYBASE_H

#include <EventSystem.h>
#include <Rcs_graph.h>

#include <pthread.h>


namespace aff
{
/*! \brief Class wrapping event-loop specific functionality, Maintains some
 *         members related to timings, such as the time step dt, a flag
 *         indicating if the loop is paused, some statistice and a mutex
 *         that ensures no issues when accessed concurrently.
 *
 *         This class deserves some severe rethinking. When designing an
 *         Entity Component System, several entities should coexist. Therefore,
 *         this class, even though convenient, should better be renamed or
 *         restructured.
 *
 *         The class subscribes to the following events:
 *         - TogglePause: Toggles the pause member variable. If it is set to
 *                        true, the process() call will wait for the enter
 *                        key before continuing.
 *         - ToggleTimeFreeze: Toggles the timeFreeze member variable. This has
 *                             no effect inside this class (\todo: should be
 *                             fixed)
 *         - Print: Prints some class information to the console.
 */
class EntityBase : public ES::EventSystem
{
public:

  /*! \brief Sets all members to defaults. The default time step is 0.05.
   */
  EntityBase();

  /*! \brief Releases the mutex. \todo: Should this be a std::mutex?
   */
  virtual ~EntityBase();

  /*! \brief Returns the time step between two calls to process()
   */
  double getDt() const;

  /*! \brief Sets the time step between two calls to process(). No checking is
   *         done.
   */
  void setDt(double dt_);

  /*! \brief Returns the accumulated time. It is incremented by dt in each call
   *         to stepTime()
   */
  double getTime() const;

  /*! \brief Sets the classes internal time to the given value.
   */
  void setTime(double time_);

  /*! \brief Increments the internal time by the internal dt.
   */
  void stepTime();

  /*! \brief Processes the event look and waits for enter key if pause mode
   *         is activated.
   */
  void process();

  /*! \brief Returns the size of the event queue at the time of the call. If
   *         it is queries right after the process() call, it will be empty.
   */
  size_t queueSize() const;

  /*! \brief Returns the all-time maximum queue size since construction of this
   *         class.
   */
  size_t getMaxQueueSize() const;

  /*! \brief Returns true if the timeFrozen member is true, false otherwise.
   *         The timeFrozen member has no effect inside this class.
   */
  bool getTimeFrozen() const;

  /*! \brief Returns true if the emergency stop callback has been called, false
   *         otherwise.
   */
  bool getEmergencyStopFlag() const;

  bool initialize(RcsGraph* graph);

private:

  void onTogglePause();
  void onToggleTimeFrozen();
  void onPrint();
  void onEmergencyStop();
  void onEmergencyRecover();

  double dt;
  double time;
  bool pause;
  bool timeFrozen;
  bool eStop;
  size_t maxQueueSize;
  mutable pthread_mutex_t mutex;
};

}

#endif   // RCS_ENTITYBASE_H

/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. All advertising materials mentioning features or use of this software
     must display the following acknowledgement: This product includes
     software developed by the Honda Research Institute Europe GmbH.

  4. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef RCS_COMPONENTBASE_H
#define RCS_COMPONENTBASE_H

#include "EntityBase.h"



namespace aff
{

/*! \brief Base class for all components. It stores a reference to the
 *         event system so that it can conveniently subscribe and publish.
 *         All subscriptions are stored in an internal vector of
 *         subscriptions. These are destroyed upon cestruction of this class,
 *         so that no dangling callbacks are stored in the event loop once
 *         the class is deleted.
 */
class ComponentBase
{
public:

  /*! \brief Constructor just stores a reference to the event system.
   */
  ComponentBase(EntityBase* parent) : entity(parent)
  {
  }

  /*! \brief Returns the pointer to the event system.
   */
  EntityBase* getEntity()
  {
    return this->entity;
  }

  /*! \brief Returns the pointer to the event system.
   */
  const EntityBase* getEntity() const
  {
    return this->entity;
  }

  /*! \brief Polymorphic destructor. Empties the subscriptions vector.
   */
  virtual ~ComponentBase()
  {
    unsubscribe();
  }

  /*! \brief Derieved classes can overwrte this to set flags through the base class.
   */
  virtual bool setParameter(const std::string& parameterName, bool flag)
  {
    return false;
  }

  /*! \brief Derieved classes can overwrte this to set pointer through the base class.
   */
  virtual bool setParameter(const std::string& parameterName, void* ptr)
  {
    return false;
  }

protected:

  /*! \brief Simplified member function subsciption. It also keeps track of all
   *         subscribed functions internally, which simplifies the common use
   *         case. If the subclass defines a subscribe() method, these methods
   *         will be hidden. To workaround, add
   *         "using ComponentBase::subscribe;" to the class body.
   */
  template<typename ...Args, typename T>
  ES::SubscriptionHandle subscribe(std::string name, void(T::*fp)(Args...) const) //const
  {
    auto thisCast = dynamic_cast<const T*>(this);
    if (thisCast == NULL)
    {
      throw std::invalid_argument("Passed function is not a member of this instance.");
    }
    auto sh = this->entity->subscribe(name, fp, thisCast);
    subscriptions.emplace_back(sh);
    return sh;
  }

  template<typename ...Args, typename T>
  ES::SubscriptionHandle subscribe(std::string name, void(T::*fp)(Args...))
  {
    auto thisCast = dynamic_cast<T*>(this);
    if (thisCast == NULL)
    {
      throw std::invalid_argument("Passed function is not a member of this instance.");
    }
    auto sh = this->entity->subscribe(name, fp, thisCast);
    subscriptions.emplace_back(sh);
    return sh;
  }


  template<typename ...Args, typename Return, typename T>
  ES::SubscriptionHandle subscribe(std::string name, Return(T::*fp)(Args...) const, ES::ignore_result_t) //const
  {
    auto thisCast = dynamic_cast<const T*>(this);
    if (thisCast == NULL)
    {
      throw std::invalid_argument("Passed function is not a member of this instance.");
    }
    auto sh = this->entity->subscribe(name, fp, thisCast, ES::ignore_result);
    subscriptions.emplace_back(sh);
    return sh;
  }

  template<typename ...Args, typename Return, typename T>
  ES::SubscriptionHandle subscribe(std::string name, Return(T::*fp)(Args...), ES::ignore_result_t)
  {
    auto thisCast = dynamic_cast<T*>(this);
    if (thisCast == NULL)
    {
      throw std::invalid_argument("Passed function is not a member of this instance.");
    }
    auto sh = this->entity->subscribe(name, fp, thisCast, ES::ignore_result);
    subscriptions.emplace_back(sh);
    return sh;
  }

  virtual void unsubscribe()
  {
    subscriptions.clear();
  }

private:

  EntityBase* entity;

  // all subscriptions of this component
  std::vector<ES::ScopedSubscription> subscriptions;
};

}

#endif   // RCS_COMPONENTBASE_H

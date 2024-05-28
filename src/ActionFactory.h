/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
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

#ifndef AFF_ACTIONFACTORY_H
#define AFF_ACTIONFACTORY_H

#include "ActionBase.h"



/*! \brief Convenience macro to register actions in the factory. Here is an
 *         example: For the action ABC, the macro expands to
 *         static aff::ActionFactoryRegistrar<ABC> ABC_(Name)
 *         This allows to conveniently write for instance:
 *         REGISTER_ACTION(ActionPour, "pour");
 */
// #define REGISTER_ACTION(Type, Name) \
//   static aff::ActionFactoryRegistrar<Type> Type ## _(Name)

/*! \brief Convenience macro to register actions in the factory. Here is an
 *         example: For the action ABC, the macro expands to
 *         static aff::ActionFactoryRegistrar<ABC> ABC10(Name)
 *         where 10 is the (unique) line number from where the marco is being
 *         resolved in the compile unit (c or c++ file). This allows to
 *         conveniently write:
 *         REGISTER_ACTION(ActionPour, "pour version1");
 *         REGISTER_ACTION(ActionPour, "pour version2");
 *         The second argument can be queried from the class using the getName()
 *         method. It is being set in the ActionFactory after the class
 *         instantiation.
 */
#define ACTION_CONCATENATE_TWO(x, y) x ## y
#define ACTION_UNIQUE(x, y) ACTION_CONCATENATE_TWO(x, y)
#define REGISTER_ACTION(Type, Name) \
  static aff::ActionFactoryRegistrar<Type> ACTION_UNIQUE(Type, __LINE__) (Name)


namespace aff
{

/*! \brief Factory class for ActionBase classes and its derieved classes. The
 *         factory implements methods to construct classes of type ActionBase
 *         (and derieved from them). It is based on a registrar class that
 *         registers methods for constructing actions. In order to enable a
 *         class derived from ActionBase to be used with this factory class,
 *         the following needs to be provided:
 *
 *         - A constructor that constructs an instance from an xml node: e.g.
 *           MyNewAction::MyNewAction(const ActionDomain& domain,
                                      const RcsGraph* graph,
                                      const vector<string> params);
 *         - Inserting a macro to register the new set in the implementation
 *           file: REGISTER_ACTION(MyNewAction);
 */
class ActionFactory
{
  template <class T> friend class ActionFactoryRegistrar;

public:

  /*! \brief Creates a new action by name using the registered
   *         construction function. The string actionCommand
   *         will first be decomposed into portions that are
   *         separated by a "+" sign (composite actions), and
   *         then by " " characters (individual words of an
   *         action command).
   */
  static ActionBase* create(const ActionScene& domain,
                            const RcsGraph* graph,
                            std::string actionCommand,
                            ActionResult& explanation);

  /*! \brief Creates a new action by name using the registered
   *         construction function.
   */
  static ActionBase* create(const ActionScene& domain,
                            const RcsGraph* graph,
                            std::vector<std::string> params,
                            ActionResult& explanation);

  /*! \brief Prints out all registered actions to the console.
   */
  static void print();

  /*! \brief Prints out all registered actions to a string.
   */
  static std::string printToString();

private:

  /*! \brief Creates a new action by name using the registered
   *         construction function.
   */
  static ActionBase* create(const ActionScene& domain,
                            const RcsGraph* graph,
                            std::string actionName,
                            std::vector<std::string> params,
                            ActionResult& explanation);

  /*! \brief Private constructor because ActionFactory is a singleton class
   */
  ActionFactory();

  /*! \brief Signature of action creation function.
   */
  typedef ActionBase* (*ActionMaker)(const ActionScene& domain,
                                     const RcsGraph* graph,
                                     std::vector<std::string> params);

  /*! \brief Registers a new function for creating constraints. You can not
   *        call this function directly. Instead us the above macro.
   */
  static void registerAction(std::string name, ActionMaker createFunction);

  static std::map<std::string, ActionFactory::ActionMaker>& constructorMap();
};





/*! \brief Registrar class for action classes. Here is how to use
 *        it:
 *        - Implement an action derieved from ActionBase
 *        - In the implementation of this class on the global scope, add:<br>
 *          REGISTER_ACTION(MyCoolNewAction);
 *        - This registers a constraint of type MyCoolNewAction that can be
 *          instantiated : <br>
 *          auto c = ActionFactory::create(domain, graph, stringVec);
 */
template<class T>
class ActionFactoryRegistrar
{
public:

  /*! \brief Registers a new action with a given name. This line
   *         needs to be put into the cpp file:
   *         REGISTER_ACTION(MyCoolNewConstraint);
   *
   *         Then, you can create a MyCoolNewAction such as
   *         auto a = ActionFactory::create(domain, graph, stringVec);
   *
   *  \param className The name that is used for instantiating a new
   *                   action class by name
   */
  ActionFactoryRegistrar(std::string className)
  {
    // Register the function to create the constraint
    ActionFactory::registerAction(className, &ActionFactoryRegistrar::create);
  }

private:

  /*! \brief This function creates a new action instance of type T
   *         passing the given variables to the respective constructor.
   */
  static ActionBase* create(const ActionScene& domain,
                            const RcsGraph* graph,
                            std::vector<std::string> params)
  {
    return new T(domain, graph, params);
  }
};

}

#endif // AFF_ACTIONFACTORY_H

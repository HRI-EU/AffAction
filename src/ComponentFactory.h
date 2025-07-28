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

#ifndef AFF_COMPONENTFACTORY_H
#define AFF_COMPONENTFACTORY_H

#include "ComponentBase.h"
#include "ActionScene.h"


/*! \brief Convenience macro to register components in the factory. Here is an
 * example: For the component ABC, the macro expands to
 * static aff::ComponentFactoryRegistrar<ABC> ABC_(PARSE_ARG)
 */
#define REGISTER_COMPONENT(T, PARSE_ARG) static aff::ComponentFactoryRegistrar<T> T ## _(PARSE_ARG)


namespace aff
{

/*! \brief Factory class for ComponentBase classes and its
 *         derieved classes. The factory implements methods to construct
 *         classes of type ComponentBase (and derieved from them)
 *         from an xml node or file containing an xml description. It is based
 *         on a registrar class that registers methods for reading and writing
 *         xml descriptions of such components. In order to enable a
 *         class derived from ComponentBase to be used with this
 *         factory class, the following needs to be provided:
 *
 *         - A constructor that constructs an instance from an xml node: e.g.
 *           MyNewComponent::MyNewComponent(xmlNode* node);
 *         - Inserting a macro to register the new set in the implementation
 *           file: REGISTER_COMPONENT(MyNewSet);
 *
 *         Most of the classes in this library have been implemented like this.
 *         See for instance \ref AnimationComponent.
 */
class ComponentFactory
{
  template <class T> friend class ComponentFactoryRegistrar;

public:

  /*! \brief Creates a new component by name using the registered
   *         construction function.
   *
   * \param node      Xml configuration
   * \return          New ComponentBase instance or nullptr in
   *                  case of failure
   */
  static std::shared_ptr<ComponentBase> create(std::string parseArg,
                                               EntityBase* entity,
                                               const RcsGraph* graph,
                                               const ActionScene* scene,
                                               std::string extraArgs);

  /*! \brief Creates a new component by name using the registered
   *         construction function.
   *
   * \param node      Xml configuration
   * \return          New ComponentBase instance or nullptr in
   *                  case of failure
   */
  static std::shared_ptr<ComponentBase> create(std::string parseArg,
                                               EntityBase* entity,
                                               const RcsGraph* graph,
                                               std::string extraArgs);

  /*! \brief Prints out all registered components to the console
   */
  static void print();

private:

  /*! \brief Private constructor because ComponentFactory is a singleton class
   */
  ComponentFactory();

  /*! \brief Signature of component creation function.
   */
  typedef std::shared_ptr<ComponentBase> (*ComponentMaker)(EntityBase* entity,
                                                           const RcsGraph* graph,
                                                           const ActionScene* scene,
                                                           std::string extraArgs);

  /*! \brief Registers a new function for creating components. You can not
   *        call this function directly. Instead us the above macro.
   */
  static void registerComponent(std::string name,
                                ComponentMaker createFunction);

  static std::map<std::string, ComponentFactory::ComponentMaker>& constructorMap();
};





/*! \brief Registrar class for component classes. Here is how to use
 *        it:
 *        - Implement a component derieved from ComponentBase
 *        - In the implementation of this class on the global scope, add:<br>
 *          REGISTER_COMPONENT(MyCoolNewComponent);
 *        - This registers a component of type MyCoolNewComponent that can be
 *          instantiated : <br>
 *          auto c = ComponentFactory::create(node);
 */
template<class T>
class ComponentFactoryRegistrar
{
public:

  /*! \brief Registers a new component with a given name. This line
   *         needs to be put into the cpp file:
   *         REGISTER_COMPONENT(MyCoolNewComponent, "-animation");
   *
   *         Then, you can create a MyCoolNewConmponent such as
   *         auto component = ComponentFactory::create(node);
   *
   *  \param className The name that is used for instanciating a new
   *                   component class by name
   */
  ComponentFactoryRegistrar(std::string className)
  {
    // Register the function to create the component
    ComponentFactory::registerComponent(className,
                                        &ComponentFactoryRegistrar::create);
  }

private:

  /*! \brief This function creates a new component instance of type T
   *         passing the given variables to the respective constructor.
   *
   * \param node    xml configuration
   * \return        New ComponentBase of type T
   */
  static std::shared_ptr<ComponentBase> create(EntityBase* entity,
                                               const RcsGraph* graph,
                                               const ActionScene* scene,
                                               std::string extraArgs)
  {
    return std::make_shared<T>(entity, graph, scene, extraArgs);
  }
};

}

#endif // AFF_COMPONENTFACTORY_H

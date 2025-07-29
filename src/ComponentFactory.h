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


/*! \brief Convenience macro to register components in the factory. Here is an
 * example: For the component ABC, the macro expands to
 * static aff::ComponentFactoryRegistrar<ABC> ABC_(PARSE_ARG)
 */
#define REGISTER_COMPONENT(T, PARSE_ARG) static aff::ComponentFactoryRegistrar<T> T ## _(PARSE_ARG)


namespace aff
{
class ActionScene;

/*! \brief Factory class for ComponentBase and its
 *         derieved classes. The factory implements methods to construct
 *         classes of type ComponentBase (and derieved from them). It is based
 *         on a registrar class. In order to enable a
 *         class derived from ComponentBase to be used with this
 *         factory class, the following needs to be provided:
 *
 *         - A constructor that constructs an instance from the below signatures:
 *           - new MyComponent(EntityBase*, const RcsGraph*)
 *           - new MyComponent(EntityBase*, const RcsGraph*, std::string)
 *           - new MyComponentEntityBase*, const RcsGraph*, const ActionScene*, std::string);
 *         - Inserting a macro to register the new set in the implementation
 *           file: REGISTER_COMPONENT(MyComponent);
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
   */
  static ComponentBase* create(std::string parseArg,
                               EntityBase* entity,
                               const RcsGraph* graph,
                               std::string extraArgs = std::string());

  static ComponentBase* create(std::string parseArg,
                               EntityBase* entity,
                               const RcsGraph* graph,
                               const ActionScene* scene,
                               std::string extraArgs = std::string());

  /*! \brief Prints out all registered components to the console
   */
  static void print();

  /*! \brief Signature of component creation function.
  */
  struct ComponentContext
  {
    EntityBase* entity = nullptr;
    const RcsGraph* graph = nullptr;
    const ActionScene* scene = nullptr;
    std::string extraArgs;
  };

  using CreatorFcn = std::function<ComponentBase*(const ComponentContext& ctx)>;


private:

  /*! \brief Private constructor because ComponentFactory is a singleton class
   */
  ComponentFactory();

  /*! \brief Registers a new function for creating components. You can not
   *        call this function directly. Instead us the above macro.
   */
  static void registerComponent(std::string name, CreatorFcn createFunction);
};





/*! \brief Registrar class for component classes. Here is how to use it:
 *        - Implement a component derieved from ComponentBase
 *        - In the implementation of this class on the global scope, add:<br>
 *          REGISTER_COMPONENT(MyCoolNewComponent);
 *        - This registers a component of type MyCoolNewComponent that can be
 *          instantiated : <br>
 *          auto c = ComponentFactory::create(args);
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
    ComponentFactory::registerComponent(className, &ComponentFactoryRegistrar::create);
  }

private:

  // Helper overload when (EntityBase*, const RcsGraph*, const ActionScene*, std::string) constructor is available
  template <typename U = T>
  static typename std::enable_if<std::is_constructible<U, EntityBase*, const RcsGraph*, const ActionScene*, std::string>::value, ComponentBase*>::type
  tryConstruct(const ComponentFactory::ComponentContext& ctx)
  {
    if (ctx.scene)
    {
      return new U(ctx.entity, ctx.graph, ctx.scene, ctx.extraArgs);
    }
    return nullptr;
  }

  // Helper overload when (EntityBase*, const RcsGraph*, std::string) constructor is available
  template <typename U = T>
  static typename std::enable_if<std::is_constructible<U, EntityBase*, const RcsGraph*, std::string>::value, ComponentBase*>::type
  tryConstruct(const ComponentFactory::ComponentContext& ctx)
  {
    return new U(ctx.entity, ctx.graph, ctx.extraArgs);
  }

  // Helper overload when (EntityBase*, const RcsGraph*) constructor is available
  template <typename U = T>
  static typename std::enable_if<std::is_constructible<U, EntityBase*, const RcsGraph*>::value, ComponentBase*>::type
  tryConstruct(const ComponentFactory::ComponentContext& ctx)
  {
    return new U(ctx.entity, ctx.graph);
  }

  // Fallback if neither is constructible
  template <typename U = T>
  static typename std::enable_if<
  !std::is_constructible<U, EntityBase*, const RcsGraph*>::value &&
  !std::is_constructible<U, EntityBase*, const RcsGraph*, std::string>::value &&
  !std::is_constructible<U, EntityBase*, const RcsGraph*, const ActionScene*, std::string>::value,
  ComponentBase*>::type
  tryConstruct(const ComponentFactory::ComponentContext&)
  {
    throw std::runtime_error("No usable constructor for component");
  }

  /*! \brief This function creates a new component instance of type T
  *         passing the given variables to the respective constructor.
  *
  * \param ctx     Context with arguments
  * \return        New ComponentBase of type T
  */
  static ComponentBase* create(const ComponentFactory::ComponentContext& ctx)
  {
    return tryConstruct(ctx);
  }

};

}

#endif // AFF_COMPONENTFACTORY_H

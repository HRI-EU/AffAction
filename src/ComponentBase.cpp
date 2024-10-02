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

#include "ComponentBase.h"




namespace aff
{



/*! \brief Constructor just stores a reference to the event system.
 */
ComponentBase::ComponentBase(EntityBase* parent) : entity(parent)
{
}

std::string ComponentBase::getName() const
{
  int status = 0;
  const char* name = typeid(*this).name();

#ifdef __GNUC__ // GCC or Clang
  char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
  std::string result = (status == 0 ? demangled : "Unknown");
  free(demangled); // Free allocated memory for demangled name
  return result;
#else
  return std::string(name); // MSVC or other compilers
#endif
}

/*! \brief Returns the pointer to the event system.
 */
EntityBase* ComponentBase::getEntity()
{
  return this->entity;
}

/*! \brief Returns the pointer to the event system.
 */
const EntityBase* ComponentBase::getEntity() const
{
  return this->entity;
}

/*! \brief Polymorphic destructor. Empties the subscriptions vector.
 */
ComponentBase::~ComponentBase()
{
  unsubscribe();
}

/*! \brief Derieved classes can overwrte this to set flags through the base class.
 */
bool ComponentBase::setParameter(const std::string& parameterName, bool flag)
{
  return false;
}

/*! \brief Derieved classes can overwrte this to set pointer through the base class.
 */
bool ComponentBase::setParameter(const std::string& parameterName, void* ptr)
{
  return false;
}

void ComponentBase::unsubscribe()
{
  subscriptions.clear();
}

}   // namespace

/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef RCS_TEXTEDITCOMPONENT_H
#define RCS_TEXTEDITCOMPONENT_H

#include "ComponentBase.h"

#include <TextEditGui.h>


namespace aff
{
/*! \brief Text input gui class. Runs a TextEditWidget in its own thread, reads in
 *         the text, and publishes it as string.
 *
 *         The class publishes the entered string as text after return is
 *         pressed. The name of the event can be configured in the constructor.
 *
 *
 *         The class subscribes to the following events:
 *         - Start: Launches a Gui. Only one will be started.
 *         - Stop: Destroys the Gui.
 *
 */
class TextEditComponent : public ComponentBase
{
  friend class TextUpdateCallback;

public:

  /*! \brief Constructs a JointGuiComponent instance and subscribes to all
   *         events described above. The constructor does not launch a gui.
   *         This happens only through the Start event.
   *
   *  \param[in] parent          Entity class responsible for event subscriptions
   */
  TextEditComponent(EntityBase* parent, const std::string& windowTitle="TextEditComponent");

  /*! \brief Unsubscribes all subscribers and frees all memory.
   */
  virtual ~TextEditComponent();


private:

  void guiCallback(std::string text);
  void onStart();
  void onStop();
  mutable pthread_mutex_t mtx;
  Rcs::TextEditGui* gui;
  std::string windowTitle;

  TextEditComponent(const TextEditComponent&);
  TextEditComponent& operator=(const TextEditComponent&);
};

}

#endif   // RCS_TEXTEDITCOMPONENT_H

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

#include "TextEditComponent.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_guiFactory.h>



namespace aff
{

class TextUpdateCallback : public Rcs::TextEditWidget::TextChangeCallback
{
public:
  TextUpdateCallback(TextEditComponent* gui_) : gui(gui_)
  {
  }

  virtual void callback(std::string text)
  {
    gui->guiCallback(text);
  };

  TextEditComponent* gui;
};


TextEditComponent::TextEditComponent(EntityBase* parent, const std::string& title) :
  ComponentBase(parent), gui(NULL), windowTitle(title)
{
  pthread_mutex_init(&this->mtx, NULL);

  subscribe("Start", &TextEditComponent::onStart);
  subscribe("Stop", &TextEditComponent::onStop);
}

TextEditComponent::~TextEditComponent()
{
  onStop();
  unsubscribe();
  pthread_mutex_destroy(&this->mtx);
}

void TextEditComponent::onStart()
{
  if (!gui)
  {
    gui = new Rcs::TextEditGui(QString::fromStdString(windowTitle));
    gui->registerCallback(new TextUpdateCallback(this));
  }
  else
  {
    RLOG(1, "TextEditGui already running");
  }
}

void TextEditComponent::onStop()
{
  if (gui)
  {
    RLOG(1, "Deleting gui");
    delete gui;
    RLOG(1, "Done deleting gui");
    gui = NULL;
  }
  else
  {
    RLOG(1, "TextEditGui already deleted");
  }
}

void TextEditComponent::guiCallback(std::string text)
{
  getEntity()->publish("PlanDFSEE", text);
  //getEntity()->publish("ActionSequence", text);
}

}   // namespace aff

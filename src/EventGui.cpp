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

#include "EventGui.h"

#include <Rcs_macros.h>

#include <QLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalMapper>
#include <QTextStream>

#include <algorithm>

namespace aff
{

  /*******************************************************************************
   *
   ******************************************************************************/
  EventLine::EventLine(std::string eventName_,
                       ES::EventSystem* entity_,
                       ES::SubscriberCollectionBase* hc) :
    eventName(eventName_), entity(entity_), text(NULL), text2(NULL), event(hc)
{
  unsigned int pCount = event->getParametersParser()->getParameterCount();
  QHBoxLayout* gridLayout = new QHBoxLayout;
  gridLayout->setContentsMargins(0,0,0,0);
  bool unknownType = false;

  switch (pCount)
  {
    case 0:   // Event with no arguments
    {
      char str[256];
      snprintf(str, 256, "%s (%d)", eventName.c_str(), (int) event->getHandlerCount());
      QLabel* label = new QLabel(QString(str));
      QFont font("Helvetica", 10, QFont::Bold);
      label->setFont(font);
      gridLayout->addWidget(label);

      QPushButton* button = new QPushButton("Fire", this);
      connect(button, SIGNAL(clicked()), SLOT(handleButton()));

      gridLayout->addWidget(button);
      break;
    }

    case 1:
    {
      std::string str = eventName + "(" + std::to_string(event->getHandlerCount());
      switch (event->getParametersParser()->getParameterType(0))
      {
        case ES::ParameterType::INT:
          str += "int)";
          break;

        case ES::ParameterType::DOUBLE:
          str += "double)";
          break;

        case ES::ParameterType::STRING:
          str += "string)";
          break;

        case ES::ParameterType::BOOL:
          str += "bool)";
          break;

        default:
          str += "unknown)";
          unknownType = true;
          break;
      }

      if (unknownType == false)
      {
        QLabel* label = new QLabel(QString::fromStdString(str));
        QFont font("Helvetica", 10, QFont::Bold);
        label->setFont(font);
        gridLayout->addWidget(label);

        this->text = new QLineEdit(this);
        connect(text, SIGNAL(returnPressed()), this, SLOT(handleText()));
        gridLayout->addWidget(text);
      }
      break;
    }

    case 2:
    {
      std::string str = eventName + "(" + std::to_string(event->getHandlerCount());
      switch (event->getParametersParser()->getParameterType(0))
      {
        case ES::ParameterType::INT:
          str += "int)";
          break;

        case ES::ParameterType::DOUBLE:
          str += "double)";
          break;

        case ES::ParameterType::STRING:
          str += "string)";
          break;

        case ES::ParameterType::BOOL:
          str += "bool)";
          break;

        default:
          str += "unknown)";
          unknownType = true;
          break;
      }

      switch (event->getParametersParser()->getParameterType(1))
      {
        case ES::ParameterType::INT:
          str += "int)";
          break;

        case ES::ParameterType::DOUBLE:
          str += "double)";
          break;

        case ES::ParameterType::STRING:
          str += "string)";
          break;

        case ES::ParameterType::BOOL:
          str += "bool)";
          break;

        default:
          str += "unknown)";
          unknownType = true;
          break;
      }

      if (unknownType == false)
      {
        QLabel* label = new QLabel(QString::fromStdString(str));
        QFont font("Helvetica", 10, QFont::Bold);
        label->setFont(font);
        gridLayout->addWidget(label);

        this->text = new QLineEdit(this);
        this->text2 = new QLineEdit(this);
        connect(text, SIGNAL(returnPressed()), this, SLOT(handleText2()));
        connect(text2, SIGNAL(returnPressed()), this, SLOT(handleText2()));
        gridLayout->addWidget(text);
        gridLayout->addWidget(text2);
      }
      break;
    }

  }   // switch

  setLayout(gridLayout);
}


void EventLine::handleText()
{
  RLOG(0, "handleText: %s %s", eventName.c_str(), text->text().toStdString().c_str());

  switch (event->getParametersParser()->getParameterType(0))
  {
    case ES::ParameterType::INT:
      entity->publish(eventName, text->text().toInt());
      break;

    case ES::ParameterType::DOUBLE:
      entity->publish(eventName, text->text().toDouble());
      break;

    case ES::ParameterType::STRING:
      entity->publish(eventName, text->text().toStdString());
      break;

    case ES::ParameterType::BOOL:
    {
      bool value = false;
      if (text->text().compare("true", Qt::CaseInsensitive)==0)
      {
        value = true;
      }
      entity->publish(eventName, value);
    }
    break;

    default:
      break;
  }

  text->clear();
}


void EventLine::handleText2()
{
  switch (event->getParametersParser()->getParameterType(0))
  {
    case ES::ParameterType::INT:
      switch (event->getParametersParser()->getParameterType(1))
      {
        case ES::ParameterType::INT:
          entity->publish(eventName, text->text().toInt(), text2->text().toInt());
          break;

        case ES::ParameterType::DOUBLE:
          entity->publish(eventName, text->text().toInt(), text2->text().toDouble());
          break;

        case ES::ParameterType::STRING:
          entity->publish(eventName, text->text().toInt(), text2->text().toStdString());
          break;

        case ES::ParameterType::BOOL:
        {
          bool value = false;
          if (QString::compare(text2->text(), QString("true"), Qt::CaseInsensitive)==0)
          {
            value = true;
          }
          entity->publish(eventName, text->text().toInt(), value);
        }
        break;

        default:
          break;
      }
      break;

    case ES::ParameterType::DOUBLE:
      switch (event->getParametersParser()->getParameterType(1))
      {
        case ES::ParameterType::INT:
          entity->publish(eventName, text->text().toDouble(), text2->text().toInt());
          break;

        case ES::ParameterType::DOUBLE:
          entity->publish(eventName, text->text().toDouble(), text2->text().toDouble());
          break;

        case ES::ParameterType::STRING:
          entity->publish(eventName, text->text().toDouble(), text2->text().toStdString());
          break;

        case ES::ParameterType::BOOL:
        {
          bool value = false;
          if (QString::compare(text2->text(), QString("true"), Qt::CaseInsensitive)==0)
          {
            value = true;
          }
          entity->publish(eventName, text->text().toDouble(), value);
        }
        break;

        default:
          break;
      }
      break;

    case ES::ParameterType::STRING:
      switch (event->getParametersParser()->getParameterType(1))
      {
        case ES::ParameterType::INT:
          entity->publish(eventName, text->text().toStdString(), text2->text().toInt());
          break;

        case ES::ParameterType::DOUBLE:
          entity->publish(eventName, text->text().toStdString(), text2->text().toDouble());
          break;

        case ES::ParameterType::STRING:
          entity->publish(eventName, text->text().toStdString(), text2->text().toStdString());
          break;

        case ES::ParameterType::BOOL:
        {
          bool value = false;
          if (QString::compare(text2->text(), QString("true"), Qt::CaseInsensitive)==0)
          {
            value = true;
          }
          entity->publish(eventName, text->text().toStdString(), value);
        }
        break;

        default:
          break;
      }
      break;

    default:
      break;
  }

  text->clear();
  text2->clear();
}


void EventLine::handleButton()
{
  entity->publish(eventName);
}

  /*******************************************************************************
   *
   ******************************************************************************/
  class EventWidget : public QScrollArea
  {
  public:
    EventWidget(ES::EventSystem* entity);
    virtual ~EventWidget() = default;
  };

  EventWidget::EventWidget(ES::EventSystem* entity) : QScrollArea()
  {
    setWindowTitle("EventWidget");

    QVBoxLayout* gridLayout = new QVBoxLayout();

    // this is needed for the widget actually being scrollable
    QWidget* scrollWidget = new QWidget(this);
    scrollWidget->setLayout(gridLayout);
    this->setWidget(scrollWidget);

    for (auto& entry : entity->getRegisteredEvents())
    {
      std::string eventName = entry.first;
      ES::SubscriberCollectionBase* event = entry.second;
      unsigned int pCount = event->getParametersParser()->getParameterCount();

      if ((pCount == 0) ||
          ((pCount == 1) && (event->getParametersParser()->getParameterType(0) != ES::ParameterType::UNSUPPORTED)) ||
          ((pCount == 2) && (event->getParametersParser()->getParameterType(0) != ES::ParameterType::UNSUPPORTED) && (event->getParametersParser()->getParameterType(1) != ES::ParameterType::UNSUPPORTED)))
      {
        gridLayout->addWidget(new EventLine(eventName, entity, event));
      }
    }   // for (auto ...

    int height = std::min(1100, ((int)entity->getRegisteredEvents().size()) * 19);

    resize(450, height);
    this->setWidgetResizable(true);
  }



  /*******************************************************************************
   *
   ******************************************************************************/
  EventGui* EventGui::create(ES::EventSystem* e)
  {
    return new EventGui(e);
  }

  EventGui::EventGui(ES::EventSystem* e) : AsyncWidget(), entity(e)
  {
    launch();
  }

  void EventGui::construct()
  {
    setWidget(new EventWidget(entity));
  }



}   // namespace 

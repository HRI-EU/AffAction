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

#include "ActionFromXML.h"
#include "ActionFactory.h"

#include <ActivationSet.h>

#include <Rcs_stlParser.h>
#include <Rcs_parser.h>
#include <Rcs_macros.h>
#include <Rcs_utils.h>


namespace aff
{
REGISTER_ACTION(ActionFromXML, "load");

ActionFromXML::ActionFromXML(const ActionScene& domain,
                             const RcsGraph* graph,
                             std::vector<std::string> params)
{
  xmlDocPtr doc = nullptr;
  xmlNodePtr node = nullptr;

  if (params.empty())
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The load action was called without parameters.",
                          "Call it with an argument being the file name",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  xmlFile = params[0];

  if (File_exists(xmlFile.c_str()))
  {
    node = parseXMLFile(xmlFile.c_str(), "Action", &doc);
  }
  else
  {
    node = parseXMLMemory(xmlFile.c_str(), xmlFile.length() + 1, &doc);
  }

  if (!node)
  {
    throw ActionException(ActionException::ParamNotFound,
                          "The string '" + xmlFile + "' is neither a file nor could not be parsed as string.",
                          "Make sure it is a valid file or xml string",
                          std::string(__FILENAME__) + " " + std::to_string(__LINE__));
  }

  setName(Rcs::getXMLNodePropertySTLString(node, "name"));

  double duration = getDefaultDuration();
  getXMLNodePropertyDouble(node, "duration", &duration);
  setDuration(duration);


  RLOG_CPP(0, "Creating action: " << getName());
  xmlNodePtr child = node->children;

  while (child)
  {
    if (child->type == XML_ELEMENT_NODE)
    {

      if (isXMLNodeNameNoCase(child, "Task"))
      {
        auto buf = xmlBufferCreate();
        xmlNodeDump(buf, doc, child, 0, 0);
        tasks.push_back((const char*)buf->content);
        xmlBufferFree(buf);
      }
      else if (isXMLNodeNameNoCase(child, "ConstraintSet"))
      {
        auto buf = xmlBufferCreate();
        xmlNodeDump(buf, doc, child, 0, 0);
        trajectory = (const char*)buf->content;
        xmlBufferFree(buf);
      }

    }   // child->type == XML_ELEMENT_NODE

    child = child->next;
  }

  xmlFreeDoc(doc);
}

ActionFromXML::~ActionFromXML()
{
}

tropic::TCS_sptr ActionFromXML::createTrajectory(double t_start, double t_end) const
{
  auto a1 = std::make_shared<tropic::ActivationSet>();
  a1->tropic::ConstraintSet::fromXML(trajectory);
  return a1;
}

std::vector<std::string> ActionFromXML::createTasksXML() const
{
  return tasks;
}

std::vector<std::string> ActionFromXML::getManipulators() const
{
  return usedManipulators;
}

std::unique_ptr<ActionBase> ActionFromXML::clone() const
{
  return std::make_unique<ActionFromXML>(*this);
}

std::string ActionFromXML::getActionCommand() const
{
  std::string cmd = getName() + " " + xmlFile;

  if (getDuration() != getDefaultDuration())
  {
    cmd += " duration " + std::to_string(getDuration());
  }

  return cmd;
}

}   // namespace aff

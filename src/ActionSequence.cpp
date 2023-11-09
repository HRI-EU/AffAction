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

#include "ActionSequence.h"

#include <Rcs_typedef.h>
#include <Rcs_shape.h>
#include <Rcs_parser.h>
#include <Rcs_stlParser.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_utils.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_body.h>
#include <Rcs_resourcePath.h>

#include <algorithm>
#include <exception>

namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
ActionSequence::ActionSequence(const std::string& xmlFile)
{
  xmlDocPtr doc = NULL;
  xmlNodePtr node = parseXMLFile(xmlFile.c_str(), "Graph", &doc);

  if (node)
  {
    parseRecursive(node, *this);
  }
  else
  {
    RLOG(4, "Failed to read xml file \"%s\"", xmlFile.c_str());
  }

  xmlFreeDoc(doc);
}

void ActionSequence::print() const
{
  RMSG("Found %zu sequences: ", sequences.size());
  for (const auto& pair : sequences)
  {
    std::cout << pair.first << " :" << pair.second << std::endl;
  }
}

void ActionSequence::parseRecursive(const xmlNodePtr parentNode, ActionSequence& seq)
{

  for (xmlNodePtr node = parentNode; node; node = node->next)
  {
    if (node->type == XML_ELEMENT_NODE)
    {
      //RLOG(5, "Found node with name: %s", node->name);

      if (xmlStrcmp(node->name, BAD_CAST "ActionSequence") == 0)
      {
        std::string name = Rcs::getXMLNodePropertySTLString(node, "name");
        std::string text = Rcs::getXMLNodePropertySTLString(node, "text");
        seq.sequences.push_back(std::make_pair(name, text));
      }

    }

    parseRecursive(node->children, seq);
  }

}

std::vector<std::pair<std::string, std::string>> ActionSequence::filterByKeyword(const std::string& keyWord) const
{
  std::vector<std::pair<std::string, std::string>> res;

  for (const auto& pair : sequences)
  {
    if (pair.first.length() < keyWord.length())
    {
      continue;
    }

    if (STRNEQ(keyWord.c_str(), pair.first.c_str(), keyWord.length()))
    {
      res.push_back(std::make_pair(pair.first, pair.second));
    }
  }

  return res;
}

} // namespace aff
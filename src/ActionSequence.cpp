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
 * Replace the first occurence of a string in another string.
 * The replacement is aware of word boundaries, i.e. it will not replace
 * "s 1" in "s 11".
 ******************************************************************************/
static void replaceFirst(std::string& str, const std::string& search, const std::string& replace)
{
  size_t pos = 0;
  const size_t search_len = search.length();
  const size_t replace_len = replace.length();

  while ((pos = str.find(search, pos)) != std::string::npos)
  {
    // Check if the found occurrence is not part of a bigger sequence
    // E.g. for "s 1" we don't want to replace "s 11"
    if ((pos == 0 || !std::isalnum(str[pos - 1])) &&
        (pos + search_len == str.length() || !std::isalnum(str[pos + search_len])))
    {
      str.replace(pos, search_len, replace);
      pos += replace_len;
      // quit after 1st replacement
      break;
    }
    else
    {
      // Move past the found occurrence without replacing
      pos += search_len;
    }
  }
}

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

std::string ActionSequence::resolve(const std::string& cfgFile, std::string text)
{
  // Step 1: Analyse if we received an ActionSequence to be broken down. This is
  // starting with either 'sequence' or 's'.
  std::vector<std::string> words = Rcs::String_split(text, " ");

  // We first check if a sequence command has been given. This is
  // starting with either 'sequence' or 's'.
  if (words.size() != 2)
  {
    RLOG_CPP(1, "Received sequence with " << words.size() << " parameters (two are expected).");
    return text;
  }

  if (words[0] != "sequence" && words[0] != "s")
  {
    RLOG_CPP(1, "Sequence keyword is " << words[0] << " but must be 's' or 'sequence'");
    return text;
  }

  // In case we detected a sequence, we parse the graph's configuration
  // file again to load all available sequences. We do this here in order
  // to allow to modify the file at run-time.
  RLOG(1, "Detected action sequence");
  ActionSequence seq(cfgFile);
  bool sequenceFound = false;
  for (const auto& pair : seq.sequences)
  {
    if (words[1] == pair.first)
    {
      // If a valid sequence name has been found, we expand it into the text string.
      text = pair.second;
      sequenceFound = true;
      RLOG_CPP(0, "Sequence expanded to '" << text << "'");
      break;
    }
  }

  // If we end up here, a sequence with a name has been given, but the
  // sequence was not specified in the configuration file. This is
  // treated as an error.
  if (!sequenceFound)
  {
    RLOG_CPP(0, "Action sequence " << words[1] << " not found in config file: '"
             << text << "'");
    return text;
  }

  // Check for cascading sequences and expand them
  bool innerSequenceFound = true;
  while (innerSequenceFound)
  {
    // We start with the assumption that no inner sequence has been found.
    // Split the text into its individual commands.
    innerSequenceFound = false;
    std::vector<std::string> textWords = Rcs::String_split(text, ";");

    // Split individual commands into single words and check if they are
    // sequences.
    for (const auto& command : textWords)
    {
      std::vector<std::string> words = Rcs::String_split(command, " ");
      if ((!words.empty()) && (words[0] == "sequence" || words[0] == "s"))
      {
        // If a inner sequence command is found, start the expanding process.
        // Flag that a cascading sequence has been found, which enforces another
        // top level loop to check for new inner sequences.
        innerSequenceFound = true;
        RLOG_CPP(1, "Detected cascading action sequence");

        // Check if the sequence is in fact part of the configuration file.
        bool sequenceFound = false;
        for (const auto& pair : seq.sequences)
        {
          // If we find the sequence, we replace the ALL the occurences
          // with the sequence text as it's defined in the config.
          if (words[1] == pair.first)
          {
            RLOG_CPP(1, "Expanding '" << command << "' into '" << pair.second << "'");

            replaceFirst(text, command, pair.second);

            sequenceFound = true;
            RLOG_CPP(1, "Sequence expanded to '" << text << "'");
            break;
          }
        }

        // TODO (CP) - duplicated code, refactor
        // If we end up here, a sequence with a name has been given, but the
        // sequence was not specified in the configuration file. This is
        // treated as an error.
        if (!sequenceFound)
        {
          RLOG_CPP(0, "Action sequence " << words[1] << " not found in config file: '" << text << "'");
          return text;
        }
      }
    }
  }

  // From here on, any sequence has been expanded
  RLOG_CPP(1, "Sequence expanded to '" << text << "'");

  return text;
}

} // namespace aff

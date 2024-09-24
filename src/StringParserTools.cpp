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

#include "StringParserTools.hpp"



namespace aff
{


// Helper function to convert a string to lowercase
static std::string toLower(const std::string& str)
{
  std::string lowerStr = str;
  std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(),
                 [](unsigned char c)
  {
    return std::tolower(c);
  });
  return lowerStr;
}

// Overload for bool with case-insensitive comparison
int getKeyValuePair(std::vector<std::string>& params, const std::string& key, bool& value, bool erase)
{
  auto it = std::find(params.begin(), params.end(), key);

  if (it == params.end())
  {
    return -1;   // Key not found
  }

  if ((it + 1) == params.end())
  {
    return -2;   // Key not found
  }

  std::string strValue = toLower(*(it + 1));  // Convert value to lowercase

  // Handle true and false cases (case-insensitive)
  if (strValue == "true" || strValue == "1")
  {
    value = true;
  }
  else if (strValue == "false" || strValue == "0")
  {
    value = false;
  }
  else
  {
    return -3;  // Invalid value for bool
  }

  return 0;
}

bool getKey(std::vector<std::string>& params, const std::string& key, bool erase)
{
  auto it = std::find(params.begin(), params.end(), key);

  if (it == params.end())
  {
    return false;   // Key not found
  }

  if (erase)
  {
    params.erase(it);
  }

  return true;
}

bool getAndEraseKey(std::vector<std::string>& params, const std::string& key)
{
  return getKey(params, key, true);
}

}   // namespace


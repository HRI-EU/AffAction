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

#ifndef AFF_STRINGPARSERTOOLS_H
#define AFF_STRINGPARSERTOOLS_H

#include <algorithm>
#include <sstream>
#include <vector>
#include <string>



namespace aff
{

// Overload of below template function for bool with case-insensitive comparison
int getKeyValuePair(std::vector<std::string>& params, const std::string& key, bool& value, bool erase=false);
bool getKey(std::vector<std::string>& params, const std::string& key, bool erase = false);
bool getAndEraseKey(std::vector<std::string>& params, const std::string& key);

// General template.
// Return values:
//  0: Success
// -1: Key not found
// -2: Conversion error: Missing string for value
// -3: Other conversion error
template<typename T>
int getKeyValuePair(std::vector<std::string>& params, const std::string& key, T& value, bool erase=false)
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

  std::string strValue = *(it + 1);

  // Check for unsigned types (generalized for any unsigned type)
  if (std::is_unsigned<T>::value && strValue.find('-') != std::string::npos)
  {
    return -3;  // Fail if it's a negative number
  }

  // Check for decimal point in non-floating point values
  if (std::is_integral<T>::value && strValue.find('.') != std::string::npos)
  {
    return -3;  // Fail if trying to assign a floating-point string to an integer type
  }

  std::stringstream stream(strValue);
  stream >> value;

  // Check if the stream conversion succeeded and if there are no remaining characters
  if (stream.fail() || !stream.eof())
  {
    return -3;  // Conversion failed, or there are leftover characters
  }

  if (erase)
  {
    params.erase(it, it+2);
  }

  return 0;
}

template<typename T>
int getAndEraseKeyValuePair(std::vector<std::string>& params, const std::string& key, T& value)
{
  return getKeyValuePair(params, key, value, true);
}



}   // namespace



#endif // AFF_STRINGPARSERTOOLS_H

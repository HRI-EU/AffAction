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

#include "SceneTransformationDataPlayer.h"

#include <Rcs_timer.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_math.h>

#include <iostream>
#include <fstream>

namespace aff
{

SceneTransformationDataPlayer::SceneTransformationDataPlayer(EntityBase* parent) :
  ComponentBase(parent)
{
  playData = false;
  newDataAvailable = false;
  currentIndex = 0;
  subscribe("PostUpdateGraph", &SceneTransformationDataPlayer::onPostUpdateGraph);
}

SceneTransformationDataPlayer::~SceneTransformationDataPlayer()
{
  RLOG(1, "SceneTransformationDataPlayer destroyed");
}

void SceneTransformationDataPlayer::getRobotBodies(RcsGraph* graph)
{
  const RcsBody* bdy = RcsGraph_getBodyByName(graph, "Johnnie");
  robotBodies.push_back(bdy->name);
  RCSBODY_TRAVERSE_CHILD_BODIES(graph, bdy)
  {
    robotBodies.push_back(BODY->name);

  }
  for (const auto& body : robotBodies)
  {
    std::cout << "Robot body: " << body << std::endl;
  }
}


void SceneTransformationDataPlayer::loadFromFile(const std::string& fileName)
{
  std::ifstream ifs(fileName);
  if (!ifs.is_open())
  {
    RLOG_CPP(0, "Failed to open file: " << fileName);
    return;
  }

  nlohmann::json jsonData;
  ifs >> jsonData;

  parse(jsonData);
}

// Function to parse a single transformation entry
BodyTransformation SceneTransformationDataPlayer::parseTransformation(const nlohmann::json& transformation)
{
  BodyTransformation trans;

  // Check if "child" and "parent" exist
  if (transformation.contains("child") && transformation.contains("parent"))
  {
    trans.child = transformation.at("child").get<std::string>();
    trans.parent = transformation.at("parent").get<std::string>();
  }

  // Check for relative transformation fields
  if (transformation.contains("relative_transformation"))
  {
    const auto& relTrans = transformation.at("relative_transformation");

    if (relTrans.contains("position") && relTrans.at("position").is_array())
    {
      auto position = relTrans.at("position").get<std::vector<double>>();
      if (position.size() == 3)
      {
        for (size_t i = 0; i < 3; ++i)
        {
          trans.relativeTransformation.org[i] = position[i];
        }
      }
    }

    // Parse the rotation matrix
    if (relTrans.contains("rotation") && relTrans.at("rotation").is_array())
    {
      auto rotation = relTrans.at("rotation").get<std::vector<double>>();
      if (rotation.size() == 9)    // 3x3 matrix flattened into 9 elements
      {
        for (size_t i = 0; i < 3; ++i)
        {
          for (size_t j = 0; j < 3; ++j)
          {
            trans.relativeTransformation.rot[i][j] = rotation[i * 3 + j];
          }
        }
      }
    }
  }

  return trans;
}

bool isTransformationFilteredOut(const std::string& name, const std::vector<std::string>& not_wanted)
{
  return std::find(not_wanted.begin(), not_wanted.end(), name) != not_wanted.end();
}

void SceneTransformationDataPlayer::parse(const nlohmann::json& json)
{
  std::lock_guard<std::mutex> lock(dataMutex);
  recordedTransformations.clear();
  if (!json.is_array())
  {
    std::cerr << "Expected JSON array at root!" << std::endl;
    return;
  }
  for (const auto& entry : json)
  {
    TransformationRecord tRecord;

    if (!entry.is_object())
    {
      std::cerr << "Expected an object within the array!" << std::endl;
      continue;
    }
    if (entry.contains("time"))
    {
      double time = entry.at("time").get<double>();  // Use .at() for safer access
      tRecord.time = time;
      std::cout << "Time: " << time << std::endl;
    }
    else
    {
      std::cerr << "Missing 'time' field" << std::endl;
    }
    // Access the "transformations" array
    if (entry.contains("transformations") && entry.at("transformations").is_array())
    {
      const auto& transformations = entry.at("transformations");

      std::vector<BodyTransformation> bodyTransformations;

      // Iterate over each transformation
      for (const auto& transformation : transformations)
      {
        if (!transformation.is_object())
        {
          std::cerr << "Expected an object in transformations" << std::endl;
          continue;
        }

        // Parse the transformation entry
        BodyTransformation trans = parseTransformation(transformation);
        // Filter out unwanted transformations based on child or parent name
        if (isTransformationFilteredOut(trans.child, robotBodies) || isTransformationFilteredOut(trans.parent, robotBodies))
        {
          std::cout << "Skipping transformation for child: " << trans.child << " or parent: " << trans.parent << std::endl;
          continue;
        }

        bodyTransformations.push_back(trans);
        // Output the parsed transformation data
        std::cout << "Child: " << trans.child << ", Parent: " << trans.parent << std::endl;

        // Output position (translation vector)
        std::cout << "Position: [" << trans.relativeTransformation.org[0] << ", "
                  << trans.relativeTransformation.org[1] << ", "
                  << trans.relativeTransformation.org[2] << "]" << std::endl;

        // Output rotation matrix
        std::cout << "Rotation Matrix:" << std::endl;
        for (int i = 0; i < 3; ++i)
        {
          std::cout << "[" << trans.relativeTransformation.rot[i][0] << ", "
                    << trans.relativeTransformation.rot[i][1] << ", "
                    << trans.relativeTransformation.rot[i][2] << "]" << std::endl;
        }
        std::cout << std::endl;
      }
      tRecord.transformations = bodyTransformations;
    }
    else
    {
      std::cerr << "Missing or invalid 'transformations' array" << std::endl;
    }
    recordedTransformations.push_back(tRecord);
  }

  RLOG_CPP(1, "Loaded " << recordedTransformations.size() << " transformations from file");
  newDataAvailable = true;
}

void SceneTransformationDataPlayer::startPlayback()
{
  currentIndex = 0;
  playData = true;
}

void SceneTransformationDataPlayer::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  updateGraph(desired);
}

void SceneTransformationDataPlayer::updateGraph(RcsGraph* graph)
{
  if (!newDataAvailable || !playData)
  {
    return;
  }
  double desiredTimeBetweenTransformations = 0.0;
  if (currentIndex>0)
  {
    desiredTimeBetweenTransformations = recordedTransformations[currentIndex].time-recordedTransformations[currentIndex-1].time;
    RLOG(0, "Desired time between trasnformations: %f", desiredTimeBetweenTransformations);
  }
  double currentTime = Timer_getSystemTime();

  // if (desiredTimeBetweenTransformations>0.0)
  // {
  //     double currentTimeDiff = currentTime-previousTime;
  //     if (currentTimeDiff<desiredTimeBetweenTransformations)
  //     {
  //         Timer_waitDT(desiredTimeBetweenTransformations-currentTimeDiff);
  //     }
  // }

  std::lock_guard<std::mutex> lock(dataMutex);
  if (currentIndex >= recordedTransformations.size())
  {
    RLOG(0, "End of recorded transformations reached");
    playData = false;
    return;
  }

  applyTransformationToGraph(graph, recordedTransformations[currentIndex]);
  if (currentIndex>0)
  {
    RLOG(0, "Applied transformations at time: %f", recordedTransformations[currentIndex].time-recordedTransformations[currentIndex-1].time);
  }
  currentIndex++;
  previousTime = currentTime;

}


void SceneTransformationDataPlayer::applyTransformationToGraph(RcsGraph* graph, const TransformationRecord& record)
{


  for (const auto& tf : record.transformations)
  {

    // Get bodies by name
    const RcsBody* child = RcsGraph_getBodyByName(graph, tf.child.c_str());

    int jidx = RcsBody_getJointIndex(graph, child);
    if (jidx!=-1)
    {
      HTr_to6DVector(&graph->q->ele[jidx], &tf.relativeTransformation);

    }


  }
}

} // namespace aff

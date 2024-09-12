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

#include "ExampleFlowMatching.h"

#include <EventGui.h>

#include <ExampleFactory.h>
#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_graphParser.h>
#include <Rcs_macros.h>
#include <Rcs_shape.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_body.h>
#include <ForceDragger.h>

#include <CmdLineWidget.h>

#include <GraphNode.h>
#include <PhysicsNode.h>

#include <osgDB/WriteFile>

#include <fstream>
#include <iostream>
#include <thread>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif


namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
static std::string getCurrentTimeString()
{
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S_")
     << std::setfill('0') << std::setw(3) << milliseconds.count();
  return ss.str();
}

static bool createDirectory(const std::string& dirName)
{
#if defined (_WIN32)
  return _mkdir(dirName.c_str()) == 0;
#else
  return mkdir(dirName.c_str(), 0755) == 0;
#endif
}

static osg::ref_ptr<osg::Image> vectorToOsgImage(const std::vector<std::vector<std::vector<float>>>& imageData)
{
  // Get the dimensions of the input image
  int height = imageData.size();
  if (height == 0)
  {
    std::cerr << "Input data is empty." << std::endl;
    return nullptr;
  }

  int width = imageData[0].size();
  if (width == 0 || imageData[0][0].size() != 3)
  {
    std::cerr << "Input data is not correctly formatted." << std::endl;
    return nullptr;
  }

  // Allocate a buffer for the pixel data (using new to allow OSG to manage it)
  unsigned char* pixelData = new unsigned char[width * height * 3]; // 3 channels for RGB

  // Flatten the vector of vectors into a single array of unsigned char
  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      pixelData[(i * width + j) * 3 + 0] = static_cast<unsigned char>(imageData[i][j][0] * 255.0f); // Red
      pixelData[(i * width + j) * 3 + 1] = static_cast<unsigned char>(imageData[i][j][1] * 255.0f); // Green
      pixelData[(i * width + j) * 3 + 2] = static_cast<unsigned char>(imageData[i][j][2] * 255.0f); // Blue
    }
  }

  // Create an osg::Image and assign the pixel data to it
  osg::ref_ptr<osg::Image> osgImage = new osg::Image();
  osgImage->setImage(width, height, 1,  // width, height, depth
                     GL_RGB,            // Internal format (RGB)
                     GL_RGB,            // Pixel format
                     GL_UNSIGNED_BYTE,  // Data type
                     pixelData,         // Pointer to the pixel data
                     osg::Image::USE_NEW_DELETE); // Use USE_NEW_DELETE so OSG manages memory

  return osgImage;
}

class SampleRecorder : public VirtualCameraComponent
{
public:

  // Data structure for one sample
  struct Sample
  {
    double time;                  // Time value
    std::vector<double> position; // Position vector
    std::string imageFileName;    // Image file name
    osg::ref_ptr<osg::Image> image;

    // Constructor to initialize the struct easily
    Sample(double t, const std::vector<double>& pos, const std::string& imgFile, osg::ref_ptr<osg::Image> img)
      : time(t), position(pos), imageFileName(imgFile), image(img)
    {
    }


  };


  SampleRecorder(EntityBase* parent, int width_=640, int height_=480,
                 bool color=true, bool depth=false) :
    VirtualCameraComponent(parent, width_, height_, color, depth)
  {
    subscribe("ToggleRecording", &SampleRecorder::toggleRecording);
    subscribe("Save", &SampleRecorder::save);
    subscribe("TogglePixelGui", &VirtualCameraComponent::togglePixelGui);
  }

  void capture(const std::vector<double>& controls)
  {
    VirtualCameraComponent::capture();

    const std::vector<std::vector<float>>& zImage = virtualRenderer->getDepthImageRef();
    const std::vector<std::vector<std::vector<float>>>& rgbImage = virtualRenderer->getRGBImageRef();

    if (recordImages)
    {
      static size_t runningIdx = 0;
      osg::ref_ptr<osg::Image> img = vectorToOsgImage(rgbImage);

      // Create a stringstream to build the filename with padding
      std::ostringstream ss;
      ss << "rgb_" << std::setw(5) << std::setfill('0') << runningIdx++ << ".png";
      samples.emplace_back(Sample(Timer_getTime(), controls, ss.str(), img));
      RLOG_CPP(0, "Adding " << ss.str() << " - now " << samples.size() << " samples.");
    }

  }

  void startRecording()
  {
    recordImages = true;
  }

  void stopRecording()
  {
    recordImages = false;
  }

  void toggleRecording()
  {
    if (recordImages)
    {
      stopRecording();
    }
    else
    {
      startRecording();
    }
  }

  bool isRecording()
  {
    return recordImages;
  }

  static void saveThread(const std::vector<Sample>& samples, std::string saveDir)
  {
    bool success = createDirectory(saveDir);
    RCHECK_MSG(success, "Could not create directory \"%s\"", saveDir.c_str());

    RLOG_CPP(0, "Saving " << samples.size() << " samples");
    std::string dataFileName = saveDir + "/" + "data.txt";
    std::ofstream outFile(dataFileName);

    if (!outFile.is_open())
    {
      RLOG_CPP(0, "Failed to open file: " << dataFileName);
      return;
    }

    for (size_t i = 0; i < samples.size(); ++i)
    {
      // Write the time and filename
      outFile << samples[i].time << " ";
      outFile << samples[i].imageFileName << " ";

      // Write the position vector
      for (const double& pos : samples[i].position)
      {
        outFile << pos << " ";
      }
      outFile << "\n";

      // Write the image file
      std::string filename = saveDir + "/" + samples[i].imageFileName;
      bool success = osgDB::writeImageFile(*samples[i].image, filename);
      RLOG(0, "%s saving file \"%s\" (%zu of %zu)",
           success ? "SUCCESS" : "FAILURE", filename.c_str(), i+1, samples.size());
    }

    outFile.close();
  }

  void save()
  {
    stopRecording();
    auto tmp = samples;
    //saveThread(tmp);
    std::thread t(saveThread, std::move(tmp), getCurrentTimeString());
    t.detach();
    samples.clear();
  }

  void clear()
  {
    stopRecording();
    samples.clear();
  }

  //std::string saveDir;

  std::vector<Sample> samples;
};


/*******************************************************************************
 *
 ******************************************************************************/
static void onSetLogLevel(int dl)
{
  RcsLogLevel = dl;
}

/*******************************************************************************
 *
 ******************************************************************************/
class NamedBodyForceDragger : public Rcs::MouseDragger
{
public:

  NamedBodyForceDragger(Rcs::PhysicsBase* sim_, SampleRecorder* cam_) :
    Rcs::MouseDragger(), sim(sim_), cam(cam_), leftShiftPressedPrev(false)
  {
  }

  virtual void update()
  {

    double I_mouseTip[3], I_bodyAnchor[3], k_bodyAnchor[3];
    bool leftMouseButtonPressed, rightMouseButtonPressed, leftShiftPressed, leftCtrlPressed;

    const RcsBody* bdy = Rcs::MouseDragger::getDragData(I_mouseTip, I_bodyAnchor, k_bodyAnchor,
                                                        &leftMouseButtonPressed,
                                                        &rightMouseButtonPressed,
                                                        &leftShiftPressed,
                                                        &leftCtrlPressed,
                                                        false);

    if (bdy)
    {
      HTr A_BI = bdy->A_BI;
      A_BI.org[0] = I_mouseTip[0];
      A_BI.org[1] = I_mouseTip[1];
      sim->applyTransform(bdy, &A_BI);
    }

    if ((!leftShiftPressedPrev) && leftShiftPressed)   // Start recording
    {
      cam->startRecording();
    }
    else if (leftShiftPressedPrev && (!leftShiftPressed))
    {
      cam->stopRecording();
    }

    leftShiftPressedPrev = leftShiftPressed;
  }

  Rcs::PhysicsBase* sim;
  SampleRecorder* cam;
  bool leftShiftPressedPrev;
};

/*******************************************************************************
 *
 ******************************************************************************/
RCS_REGISTER_EXAMPLE(ExampleFlowMatching, "Actions", "FlowMatching");

ExampleFlowMatching::ExampleFlowMatching() : ExampleFlowMatching(0, NULL)
{
}

ExampleFlowMatching::ExampleFlowMatching(int argc, char** argv) : ExampleBase(argc, argv)
{
  initGraph = NULL;
  rndGraph = NULL;
  dt = 0.01;
  dt_max = 0.0;
  loopCount = 0;
}

ExampleFlowMatching::~ExampleFlowMatching()
{
  stop();
  RcsGraph_destroy(initGraph);
  RcsGraph_destroy(rndGraph);
  Rcs_removeResourcePath(configDirectory.c_str());
}

bool ExampleFlowMatching::initParameters()
{
  xmlFileName = "g_tbar.xml";
  configDirectory = "../src/Smile/src/AffAction/config/xml/flowmatching";

  return true;
}

bool ExampleFlowMatching::parseArgs(Rcs::CmdLineParser* parser)
{
  parser->getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");
  parser->getArgument("-dt", &dt, "Time step (default is %f)", dt);
  parser->getArgument("-f", &xmlFileName, "Configuration file name "
                      "(default is %s)", xmlFileName.c_str());
  parser->getArgument("-dir", &configDirectory, "Configuration file directory "
                      "(default is %s)", configDirectory.c_str());

  if (parser->hasArgument("-h"))
  {
    std::cout << help() << std::endl;
    return false;
  }

  return true;
}

bool ExampleFlowMatching::initAlgo()
{
  Rcs_addResourcePath(RCS_CONFIG_DIR);
  Rcs_addResourcePath(configDirectory.c_str());

  entity.subscribe("SetLogLevel", &onSetLogLevel);
  entity.subscribe("Quit", &ExampleFlowMatching::onQuit, this);
  entity.setDt(dt);

  this->initGraph = RcsGraph_create(xmlFileName.c_str());
  RCHECK(RcsGraph_check(initGraph, NULL, NULL));
  this->rndGraph = RcsGraph_clone(initGraph);

  // Graph component contains "sensed" graph
  graphC = std::make_unique<aff::GraphComponent>(&entity, initGraph);
  graphC->setEnableRender(false);
  physicsC = std::make_unique<aff::PhysicsComponent>(&entity, initGraph, "Bullet", "physics.xml");

  // Initialization sequence to initialize all graphs from the sensory state. This also triggers the
  // "Start" event, starting all component threads.
  entity.initialize(graphC->getGraph());

  RLOG_CPP(1, help());

  return true;
}

bool ExampleFlowMatching::initGraphics()
{
  // Determine camera transform
  double q_cam[6];
  VecNd_set6(q_cam, -2.726404, -2.515165, 3.447799,   -0.390124, 0.543041, 0.795294);

  // Check if there is a body named 'initial_camera_view'.
  const RcsBody* camera_body = RcsGraph_getBodyByName(graphC->getGraph(), "default_camera_view");
  if (camera_body)
  {
    RLOG(1, "Setting initial view based on body 'initial_camera_view'.");
    HTr_to6DVector(q_cam, &camera_body->A_BI);
  }

  // Create GraphicsWindow
  viewer = std::make_unique<aff::GraphicsWindow>(&entity, true, true);
  viewer->setTitle("ExampleFlowMatching");
  viewer->setCameraTransform(q_cam[0], q_cam[1], q_cam[2], q_cam[3], q_cam[4], q_cam[5]);

  // Display physics in GraphicsWindow
  osg::ref_ptr<Rcs::PhysicsNode> pn = new Rcs::PhysicsNode();
  pn->setSimulation(physicsC->getPhysics());
  pn->init(false);   // without force dragger, we use our own
  pn->getPhysicsGraphNode()->displayCollisionModel(true);   // shows the goal in wireframe

  // Add camera renderer
  vcamC = std::make_unique<SampleRecorder>(&entity);
  vcamC->setSceneData(new Rcs::GraphNode(graphC->getGraph()));
  vcamC->setCameraTransform(q_cam);

  // Mouse dragger without lines etc.
  osg::ref_ptr<NamedBodyForceDragger> dragger = new NamedBodyForceDragger(physicsC->getPhysics(), static_cast<SampleRecorder*>(vcamC.get()));
  dragger->setEnableDragLine(false);
  pn->addChild(dragger.get());
  viewer->add(pn.get());


#if defined(_MSC_VER)
  viewer->setWindowSize(12, 36, 640, 480);
#else
  viewer->setWindowSize(0, 0, 640, 480);
#endif

  viewer->setKeyCallback('e', [this](char k)
  {
    new aff::EventGui(&entity);
  }, "Launch event gui");

  viewer->setKeyCallback('p', [this](char k)
  {
    entity.call<const RcsGraph*>("InitFromState", initGraph);
  }, "Reset physics");

  viewer->setKeyCallback('n', [this](char k)
  {
    RLOG(0, "Rejecting samples");
    randomize(rndGraph);
    static_cast<SampleRecorder*>(vcamC.get())->clear();
    entity.call<const RcsGraph*>("InitFromState", rndGraph);
  }, "New configuration, reject previous episode");

  viewer->setKeyCallback('y', [this](char k)
  {
    RLOG(0, "Saving samples");
    static_cast<SampleRecorder*>(vcamC.get())->save();
    randomize(rndGraph);
    entity.call<const RcsGraph*>("InitFromState", rndGraph);
  }, "New configuration, save previous episode");

  Timer_setZero();

  return true;
}

void ExampleFlowMatching::stop()
{
  ExampleBase::stop();

  // The runLoop is ended with ExampleBase::stop(). We still need to call each
  // component's stop event.
  entity.publish("Stop");
  entity.process();
}

void ExampleFlowMatching::step()
{
  double dtProcess = Timer_getSystemTime();

  // 1kHz
  for (size_t i=0; i<10; ++i)
  {
    entity.publish("UpdateGraph", graphC->getGraph());
  }
  entity.publish("ComputeKinematics", graphC->getGraph());

  // 30Hz
  if (loopCount%3==0)
  {
    entity.publish("Render");
  }

  // 10Hz
  if (loopCount%10==0)
  {
    const RcsBody* block = RcsGraph_getBodyByName(graphC->getGraph(), "block");
    RCHECK(block);
    double q_block[6];
    HTr_to6DVector(q_block, &block->A_BI);
    std::vector<double> controls(q_block, q_block+6);
    static_cast<SampleRecorder*>(vcamC.get())->capture(controls);
  }

  entity.process();
  entity.stepTime();

  dtProcess = Timer_getSystemTime() - dtProcess;

  if (entity.getTime() > 3.0)
  {
    dt_max = std::max(dt_max, dtProcess);
  }


  char timeStr[256];
  snprintf(timeStr, 256, "Time: %.3f %.3f  dt: %.1f dt_max: %.1f msec\n"
           "queue: %zu (max: %zu)",
           entity.getTime(), Timer_getTime(), dtProcess * 1.0e3, dt_max * 1.0e3,
           entity.queueSize(), entity.getMaxQueueSize());
  entity.publish("SetTextLine", std::string(timeStr), 0);

  Timer_waitDT(entity.getDt() - dtProcess);

  loopCount++;
}

void ExampleFlowMatching::onQuit()
{
  entity.publish("Stop");
  runLoop = false;
}

void ExampleFlowMatching::randomize(RcsGraph* graph) const
{
  // Extents
  const RcsBody* table = RcsGraph_getBodyByName(graph, "table");
  RCHECK(table && table->nShapes > 0);
  const double* e = table->shapes[0].extents;
  double bounds = 0.25;

  const RcsBody* tbar = RcsGraph_getBodyByName(graph, "tbar");
  RCHECK(tbar && tbar->rigid_body_joints);
  double* q_rbj = RcsBody_getStatePtr(graph, tbar);
  q_rbj[0] = Math_getRandomNumber(-0.5 * e[0] + bounds, 0.5 * e[0] - bounds);
  q_rbj[1] = Math_getRandomNumber(-0.5 * e[1] + bounds, 0.5 * e[1] - bounds);
  q_rbj[5] = Math_getRandomNumber(-M_PI, M_PI);

  const RcsBody* pusher = RcsGraph_getBodyByName(graph, "block");
  RCHECK(pusher && pusher->rigid_body_joints);
  bounds = 0.05;

  size_t iter = 0;
  do
  {
    q_rbj = RcsBody_getStatePtr(graph, pusher);
    q_rbj[0] = Math_getRandomNumber(-0.5 * e[0] + bounds, 0.5 * e[0] - bounds);
    q_rbj[1] = Math_getRandomNumber(-0.5 * e[1] + bounds, 0.5 * e[1] - bounds);

    RcsGraph_setState(graph, NULL, NULL);
    if (iter>5)
    {
      RLOG_CPP(0, "Sampling pusher " << iter << " times");
    }
    iter++;
  }
  while (RcsBody_distance(tbar, pusher, NULL, NULL, NULL) < 0.1);

}

}   // namespace aff
